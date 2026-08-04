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
    uint64_t capture_session_id = 0;
    uint64_t frame_id = 0;
    uint32_t v4l2_sequence = 0;

    // All normalized timestamps use CLOCK_MONOTONIC nanoseconds.
    int64_t capture_timestamp_ns = 0;
    int64_t dequeue_timestamp_ns = 0;

    // Raw V4L2 timestamp retained for diagnostics.
    int64_t driver_timestamp_ns = 0;
    TimestampOrigin timestamp_origin =
        TimestampOrigin::DEQUEUE_MONOTONIC_FALLBACK;
};

struct CapturePlaneView {
    void* mapped_address = nullptr;
    int dma_fd = -1;
    std::size_t data_offset = 0;
    std::size_t bytes_used = 0;
    std::size_t allocation_length = 0;
    uint32_t stride = 0;
    uint32_t size_image = 0;
};

struct CaptureFrameView {
    FrameIdentity identity;
    uint32_t buffer_index = 0;
    int width = 0;
    int height = 0;
    uint32_t pixel_format = 0;
    uint32_t buffer_flags = 0;
    uint32_t plane_count = 0;
    std::array<CapturePlaneView, kMaxFramePlanes> planes{};
};

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
    void* cpu_address = nullptr;
    int dma_fd = -1;
    std::size_t dma_offset = 0;
    std::size_t capacity_bytes = 0;
    int width = 0;
    int height = 0;
    int channels = 3;
    ModelInputMemoryLayout memory_layout =
        ModelInputMemoryLayout::RGB_UINT8_NHWC;
    uint32_t row_stride_bytes = 0;
};

enum class CoordinateSpace {
    MODEL_INPUT,
    ORIGINAL_FRAME,
};

enum class TargetState : uint8_t {
    NO_TARGET = 0,
    CANDIDATE,
    DETECTED,
    LOST,
    INVALID,
    STALE,
};

[[nodiscard]] inline const char* TargetStateName(TargetState state) noexcept {
    switch (state) {
        case TargetState::NO_TARGET: return "NO_TARGET";
        case TargetState::CANDIDATE: return "CANDIDATE";
        case TargetState::DETECTED: return "DETECTED";
        case TargetState::LOST: return "LOST";
        case TargetState::INVALID: return "INVALID";
        case TargetState::STALE: return "STALE";
    }
    return "UNKNOWN";
}

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

    int64_t inference_enqueue_ns = 0;
    int64_t input_slot_wait_start_ns = 0;
    int64_t input_slot_acquired_ns = 0;
    int64_t latest_frame_dequeue_ns = 0;
    int64_t preprocess_start_ns = 0;
    int64_t preprocess_end_ns = 0;
    int64_t inference_start_ns = 0;
    int64_t inference_end_ns = 0;
    int64_t postprocess_end_ns = 0;
    int64_t generated_timestamp_ns = 0;
    int64_t result_age_ns = 0;

    // RKNN API timing for this frame.
    int64_t rknn_input_submit_ns = 0;
    int64_t rknn_output_bind_ns = 0;
    int64_t rknn_run_ns = 0;
    int64_t rknn_output_get_ns = 0;
    int64_t rknn_output_release_ns = 0;
    int64_t rknn_total_ns = 0;

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

struct LatencyDistributionSnapshot {
    uint64_t total_samples = 0;
    uint64_t retained_samples = 0;
    bool truncated = false;
    double mean_ms = 0.0;
    double p50_ms = 0.0;
    double p95_ms = 0.0;
    double p99_ms = 0.0;
    double maximum_ms = 0.0;
};

struct PipelineTimingSnapshot {
    LatencyDistributionSnapshot input_slot_wait;
    LatencyDistributionSnapshot latest_frame_queue_wait;
    LatencyDistributionSnapshot capture_to_preprocess_start;
    LatencyDistributionSnapshot preprocess;
    LatencyDistributionSnapshot rknn_input_submit;
    LatencyDistributionSnapshot rknn_output_bind;
    LatencyDistributionSnapshot rknn_bind_total;
    LatencyDistributionSnapshot rknn_run;
    LatencyDistributionSnapshot rknn_output_get;
    LatencyDistributionSnapshot rknn_output_release;
    LatencyDistributionSnapshot rknn_total;
    LatencyDistributionSnapshot postprocess;
    LatencyDistributionSnapshot capture_to_result;
    LatencyDistributionSnapshot result_age;
};

struct QueueStatsSnapshot {
    uint64_t pushed = 0;
    uint64_t popped = 0;
    uint64_t replaced_oldest = 0;
    std::size_t current_size = 0;
    std::size_t high_watermark = 0;
    std::size_t capacity = 0;
    bool stopped = false;
};

struct PipelineStatsSnapshot {
    uint64_t captured_frames = 0;
    uint64_t camera_timeouts = 0;
    uint64_t camera_wakes = 0;
    uint64_t driver_dropped_frames = 0;
    uint64_t replaced_waiting_frames = 0;
    uint64_t skipped_no_input_slot = 0;
    uint64_t preprocess_failures = 0;
    uint64_t inference_successes = 0;
    uint64_t inference_failures = 0;
    uint64_t postprocess_successes = 0;
    uint64_t postprocess_failures = 0;
    uint64_t result_publish_failures = 0;
    uint64_t requeue_failures = 0;
    uint64_t dmabuf_sync_failures = 0;

    uint64_t video_frames_enqueued = 0;
    uint64_t video_frames_encoded = 0;
    uint64_t video_encode_failures = 0;
    uint64_t video_packets_enqueued = 0;
    uint64_t video_packets_dropped = 0;
    uint64_t video_sink_failures = 0;

    std::size_t camera_buffer_count_at_start = 0;
    uint32_t camera_outstanding_before_stop = 0;
    uint64_t broker_outstanding_frames_before_camera_stop = 0;
    uint64_t broker_outstanding_leases_before_camera_stop = 0;

    bool fatal_error = false;
    bool graceful_shutdown_completed = false;
    bool split_final_completed_frame_drained = false;

    InferenceThreadTopology topology =
        InferenceThreadTopology::FUSED_NPU_POSTPROCESS;

    QueueStatsSnapshot captured_frame_queue;
    QueueStatsSnapshot prepared_frame_queue;
    QueueStatsSnapshot completed_frame_queue;
    QueueStatsSnapshot video_frame_queue;
    QueueStatsSnapshot encoded_packet_queue;
    PipelineTimingSnapshot timing;
};

}  // namespace visionarm
