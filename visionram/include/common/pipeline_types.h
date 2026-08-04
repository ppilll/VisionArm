#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace visionarm {

constexpr std::size_t kMaxFramePlanes = 3;

enum class TimestampOrigin {
    V4L2_MONOTONIC,
    DEQUEUE_MONOTONIC_FALLBACK,
};

struct FrameIdentity {
    // Monotonically increasing for each Start() on the same camera object.
    // It disambiguates frame_id reuse across streaming restarts.
    uint64_t capture_session_id = 0;
    uint64_t frame_id = 0;
    uint32_t v4l2_sequence = 0;

    // Normalized timestamps use CLOCK_MONOTONIC nanoseconds.
    int64_t capture_timestamp_ns = 0;
    int64_t dequeue_timestamp_ns = 0;

    // Raw V4L2 timestamp retained for diagnostics.
    int64_t driver_timestamp_ns = 0;
    TimestampOrigin timestamp_origin =
        TimestampOrigin::DEQUEUE_MONOTONIC_FALLBACK;
};

struct CapturePlaneView {
    // CPU-visible address of the first valid byte for this plane. This is a
    // non-owning view into one dequeued V4L2 MMAP buffer.
    void* mapped_address = nullptr;

    // DMA-BUF fd exported through VIDIOC_EXPBUF. V4L2Camera owns the fd;
    // consumers must never close it.
    int dma_fd = -1;

    // Offset of the valid image data within the exported DMA-BUF allocation.
    std::size_t data_offset = 0;

    // Number of valid bytes after data_offset, as reported by DQBUF.
    std::size_t bytes_used = 0;

    // Total size of the mmap/exported allocation for this plane.
    std::size_t allocation_length = 0;

    uint32_t stride = 0;
    uint32_t size_image = 0;
};

struct CaptureFrameView {
    FrameIdentity identity;

    // The V4L2 buffer remains dequeued while this view is in use. In the
    // current compatibility pipeline the caller eventually calls Requeue();
    // after R1 only the camera owner thread may requeue a broker completion.
    uint32_t buffer_index = 0;

    int width = 0;
    int height = 0;
    uint32_t pixel_format = 0;
    uint32_t buffer_flags = 0;

    uint32_t plane_count = 0;
    std::array<CapturePlaneView, kMaxFramePlanes> planes{};
};

// Temporary compatibility aliases for the pre-R1 inference pipeline. New R0/R1
// code must use CapturePlaneView and CaptureFrameView to make the non-owning
// lifetime explicit. Remove these aliases only after the R1 migration is
// complete and all callers use FrameLease.
using FramePlane = CapturePlaneView;
using FramePacket = CaptureFrameView;

struct ImageShape {
    uint32_t batch = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t channels = 0;
};

struct PreprocessTransform {
    int source_width = 0;
    int source_height = 0;
    int model_width = 0;
    int model_height = 0;

    // Forward mapping:
    // model_x = source_x * scale_x + pad_left
    // model_y = source_y * scale_y + pad_top
    float scale_x = 0.0F;
    float scale_y = 0.0F;
    float uniform_scale = 0.0F;

    int pad_left = 0;
    int pad_top = 0;
    int pad_right = 0;
    int pad_bottom = 0;
    bool letterbox = true;
};

enum class ModelInputMemoryLayout {
    RGB_UINT8_NHWC,
    RGB_INT8_NHWC,
    OPAQUE_NATIVE,
};

struct ModelInputBufferView {
    std::size_t slot_index = 0;

    // cpu_address and dma_fd describe the same persistent RKNN input slot.
    // CPU/OpenCV writes cpu_address; an RGA implementation imports dma_fd.
    void* cpu_address = nullptr;
    int dma_fd = -1;
    std::size_t dma_offset = 0;
    std::size_t capacity_bytes = 0;

    int width = 0;
    int height = 0;
    int channels = 3;
    ModelInputMemoryLayout memory_layout =
        ModelInputMemoryLayout::RGB_UINT8_NHWC;

    // Bytes between two rows. Zero means tightly packed width*channels.
    uint32_t row_stride_bytes = 0;
};

enum class CoordinateSpace {
    MODEL_INPUT,
    ORIGINAL_FRAME,
};

enum class TargetState {
    NO_TARGET,
    DETECTED,
    INVALID,
};

struct Detection {
    float x1 = 0.0F;
    float y1 = 0.0F;
    float x2 = 0.0F;
    float y2 = 0.0F;
    float confidence = 0.0F;
    int class_id = -1;
    CoordinateSpace space = CoordinateSpace::MODEL_INPUT;
};

struct TargetObservation {
    TargetState state = TargetState::NO_TARGET;
    bool valid = false;

    float confidence = 0.0F;
    int class_id = -1;

    float x1 = 0.0F;
    float y1 = 0.0F;
    float x2 = 0.0F;
    float y2 = 0.0F;
    float center_x = 0.0F;
    float center_y = 0.0F;
    float width = 0.0F;
    float height = 0.0F;
    float area = 0.0F;

    int source_width = 0;
    int source_height = 0;
};

struct TargetError {
    TargetState state = TargetState::NO_TARGET;
    bool valid = false;

    float dx_px = 0.0F;
    float dy_px = 0.0F;
    float error_x_normalized = 0.0F;
    float error_y_normalized = 0.0F;
    float confidence = 0.0F;
};

struct BestGridLocation {
    int branch_index = -1;
    int grid_x = -1;
    int grid_y = -1;
    int stride = 0;
    float confidence = 0.0F;
};

struct PostprocessResult {
    std::optional<BestGridLocation> selected_grid;
    std::optional<Detection> model_detection;
    std::optional<Detection> original_detection;
    std::optional<TargetObservation> target;
    TargetError error;
};

struct PerceptionPacket {
    FrameIdentity identity;
    PreprocessTransform transform;

    int64_t preprocess_start_ns = 0;
    int64_t preprocess_end_ns = 0;
    int64_t inference_start_ns = 0;
    int64_t inference_end_ns = 0;
    int64_t postprocess_end_ns = 0;

    PostprocessResult result;
};

enum class InferenceThreadTopology {
    SPLIT_NPU_POSTPROCESS,
    FUSED_NPU_POSTPROCESS,
};

[[nodiscard]] inline const char* InferenceThreadTopologyName(
    InferenceThreadTopology topology) noexcept {
    switch (topology) {
        case InferenceThreadTopology::SPLIT_NPU_POSTPROCESS:
            return "split_npu_postprocess";
        case InferenceThreadTopology::FUSED_NPU_POSTPROCESS:
            return "fused_npu_postprocess";
    }
    return "unknown";
}

struct PipelineStatsSnapshot {
    uint64_t captured_frames = 0;
    uint64_t driver_dropped_frames = 0;
    uint64_t replaced_waiting_frames = 0;
    uint64_t skipped_no_input_slot = 0;
    uint64_t preprocess_failures = 0;
    uint64_t inference_successes = 0;
    uint64_t inference_failures = 0;
    uint64_t postprocess_failures = 0;
    uint64_t requeue_failures = 0;
    uint64_t dmabuf_sync_failures = 0;

    uint64_t video_frames_enqueued = 0;
    uint64_t video_frames_encoded = 0;
    uint64_t video_encode_failures = 0;
    uint64_t video_packets_enqueued = 0;
    uint64_t video_packets_dropped = 0;
    uint64_t video_sink_failures = 0;
    uint64_t postprocess_successes = 0;
    InferenceThreadTopology topology =
        InferenceThreadTopology::FUSED_NPU_POSTPROCESS;
};

}  // namespace visionarm
