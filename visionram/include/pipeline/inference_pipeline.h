#pragma once

#include "camera/camera_source.h"
#include "camera/capture_buffer_contract.h"
#include "common/pipeline_types.h"
#include "inference/rknn_engine.h"
#include "pipeline/bounded_queue.h"
#include "pipeline/latest_result_store.h"
#include "postprocess/yolov8_top1_postprocessor.h"
#include "preprocess/image_preprocessor.h"
#include "video/encoded_packet_sink.h"
#include "video/video_encoder.h"

#include <atomic>
#include <cstddef>
#include <thread>

namespace visionarm {

struct InferencePipelineConfig {
    std::size_t captured_frame_queue_capacity = 1U;
    std::size_t prepared_frame_queue_capacity = 2U;
    std::size_t completed_frame_queue_capacity = 2U;

    // The video queue preserves capture order. The Camera owner may block when
    // the queue is full, providing bounded backpressure rather than silently
    // accumulating raw V4L2 buffers.
    std::size_t video_frame_queue_capacity = 4U;

    // Encoded packets own their compressed bytes, so blocking here never holds
    // a Camera FrameLease.
    std::size_t encoded_packet_queue_capacity = 8U;

    bool enable_video = false;
    InferenceThreadTopology topology =
        InferenceThreadTopology::FUSED_NPU_POSTPROCESS;

    bool require_cpu_dmabuf_sync = true;
    bool require_bound_input_dmabuf_sync = true;
};

class InferencePipeline final {
public:
    InferencePipeline(
        InferencePipelineConfig config,
        ICameraSource* camera,
        ICaptureBufferBroker* broker,
        IImagePreprocessor* preprocessor,
        RknnEngine* engine,
        ITargetPostprocessor* postprocessor,
        IPerceptionSink* result_sink,
        IVideoEncoder* video_encoder = nullptr,
        IEncodedPacketSink* encoded_packet_sink = nullptr);

    ~InferencePipeline();

    InferencePipeline(const InferencePipeline&) = delete;
    InferencePipeline& operator=(const InferencePipeline&) = delete;

    [[nodiscard]] bool Start();
    void Stop() noexcept;

    [[nodiscard]] bool running() const noexcept {
        return running_.load(std::memory_order_acquire);
    }

    [[nodiscard]] PipelineStatsSnapshot stats() const noexcept;

private:
    struct PreparedFrame {
        std::size_t input_slot_index = 0U;
        FrameIdentity identity;
        PreprocessTransform transform;
        int64_t preprocess_start_ns = 0;
        int64_t preprocess_end_ns = 0;
    };

    struct CompletedFrame {
        std::size_t output_slot_index = 0U;
        PreparedFrame prepared;
        int64_t inference_start_ns = 0;
        int64_t inference_end_ns = 0;
    };

    void CaptureLoop() noexcept;
    void VideoLoop() noexcept;
    void EncodedPacketLoop() noexcept;
    void PreprocessLoop() noexcept;
    void NpuLoop() noexcept;
    void PostprocessLoop() noexcept;
    void NpuPostprocessLoop() noexcept;

    [[nodiscard]] bool ProcessCompletedFrame(
        CompletedFrame* completed) noexcept;

    void SignalFailure() noexcept;
    void DrainRequeueRequests() noexcept;
    void DrainRequeueRequestsUntilClosed() noexcept;
    void RequeueDirect(const CaptureFrameView& frame) noexcept;

    InferencePipelineConfig config_;

    ICameraSource* camera_ = nullptr;
    ICaptureBufferBroker* broker_ = nullptr;
    IImagePreprocessor* preprocessor_ = nullptr;
    RknnEngine* engine_ = nullptr;
    ITargetPostprocessor* postprocessor_ = nullptr;
    IPerceptionSink* result_sink_ = nullptr;
    IVideoEncoder* video_encoder_ = nullptr;
    IEncodedPacketSink* encoded_packet_sink_ = nullptr;

    BoundedQueue<FrameLeasePtr> captured_frames_;
    BoundedQueue<FrameLeasePtr> video_frames_;
    BoundedQueue<EncodedPacket> encoded_packets_;
    BoundedQueue<PreparedFrame> prepared_frames_;
    BoundedQueue<CompletedFrame> completed_frames_;
    BoundedQueue<std::size_t> free_input_slots_;
    BoundedQueue<std::size_t> free_output_slots_;

    std::thread capture_thread_;
    std::thread video_thread_;
    std::thread encoded_packet_thread_;
    std::thread preprocess_thread_;
    std::thread npu_thread_;
    std::thread postprocess_thread_;

    std::atomic<bool> running_{false};
    bool started_once_ = false;
    std::atomic<bool> stop_completed_{false};

    std::atomic<uint64_t> captured_count_{0U};
    std::atomic<uint64_t> driver_dropped_count_{0U};
    std::atomic<uint64_t> replaced_count_{0U};
    std::atomic<uint64_t> skipped_no_input_count_{0U};
    std::atomic<uint64_t> preprocess_failure_count_{0U};
    std::atomic<uint64_t> inference_success_count_{0U};
    std::atomic<uint64_t> inference_failure_count_{0U};
    std::atomic<uint64_t> postprocess_success_count_{0U};
    std::atomic<uint64_t> postprocess_failure_count_{0U};
    std::atomic<uint64_t> requeue_failure_count_{0U};
    std::atomic<uint64_t> dmabuf_sync_failure_count_{0U};

    std::atomic<uint64_t> video_frames_enqueued_count_{0U};
    std::atomic<uint64_t> video_frames_encoded_count_{0U};
    std::atomic<uint64_t> video_encode_failure_count_{0U};
    std::atomic<uint64_t> video_packets_enqueued_count_{0U};
    std::atomic<uint64_t> video_packets_dropped_count_{0U};
    std::atomic<uint64_t> video_sink_failure_count_{0U};
};

}  // namespace visionarm
