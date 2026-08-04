#pragma once

#include "camera/camera_source.h"
#include "camera/capture_buffer_contract.h"
#include "common/pipeline_types.h"
#include "inference/rknn_engine.h"
#include "metrics/latency_accumulator.h"
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
    // Final R6 low-latency policy: one pending latest frame. When the single
    // RKNN input slot is busy, Capture replaces this pending item.
    std::size_t captured_frame_queue_capacity = 1U;
    std::size_t prepared_frame_queue_capacity = 1U;
    std::size_t completed_frame_queue_capacity = 1U;

    std::size_t video_frame_queue_capacity = 2U;
    std::size_t encoded_packet_queue_capacity = 16U;

    bool enable_video = false;
    InferenceThreadTopology topology =
        InferenceThreadTopology::FUSED_NPU_POSTPROCESS;

    bool require_cpu_dmabuf_sync = true;
    bool require_bound_input_dmabuf_sync = true;

    // 65536 retains more than 18 minutes at 60 perception results/s.
    std::size_t latency_sample_capacity = 65536U;
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
    struct PendingInferenceFrame {
        FrameLeasePtr lease;
        int64_t enqueue_timestamp_ns = 0;
    };

    struct PreparedFrame {
        std::size_t input_slot_index = 0U;
        FrameIdentity identity;
        PreprocessTransform transform;
        int64_t inference_enqueue_ns = 0;
        int64_t input_slot_wait_start_ns = 0;
        int64_t input_slot_acquired_ns = 0;
        int64_t latest_frame_dequeue_ns = 0;
        int64_t preprocess_start_ns = 0;
        int64_t preprocess_end_ns = 0;
    };

    struct CompletedFrame {
        std::size_t output_slot_index = 0U;
        PreparedFrame prepared;
        int64_t inference_start_ns = 0;
        int64_t inference_end_ns = 0;
        RknnRunTiming rknn_timing;
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
    void RecordCompletedTiming(const PerceptionPacket& packet) noexcept;

    InferencePipelineConfig config_;

    ICameraSource* camera_ = nullptr;
    ICaptureBufferBroker* broker_ = nullptr;
    IImagePreprocessor* preprocessor_ = nullptr;
    RknnEngine* engine_ = nullptr;
    ITargetPostprocessor* postprocessor_ = nullptr;
    IPerceptionSink* result_sink_ = nullptr;
    IVideoEncoder* video_encoder_ = nullptr;
    IEncodedPacketSink* encoded_packet_sink_ = nullptr;

    BoundedQueue<PendingInferenceFrame> captured_frames_;
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
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> fatal_error_{false};
    bool started_once_ = false;
    std::atomic<bool> stop_completed_{false};
    std::atomic<bool> graceful_shutdown_completed_{false};
    std::atomic<bool> split_final_completed_frame_drained_{false};

    std::atomic<uint64_t> captured_count_{0U};
    std::atomic<uint64_t> camera_timeout_count_{0U};
    std::atomic<uint64_t> camera_wake_count_{0U};
    std::atomic<uint64_t> driver_dropped_count_{0U};
    std::atomic<uint64_t> replaced_count_{0U};
    std::atomic<uint64_t> skipped_no_input_count_{0U};
    std::atomic<uint64_t> preprocess_failure_count_{0U};
    std::atomic<uint64_t> inference_success_count_{0U};
    std::atomic<uint64_t> inference_failure_count_{0U};
    std::atomic<uint64_t> postprocess_success_count_{0U};
    std::atomic<uint64_t> postprocess_failure_count_{0U};
    std::atomic<uint64_t> result_publish_failure_count_{0U};
    std::atomic<uint64_t> requeue_failure_count_{0U};
    std::atomic<uint64_t> dmabuf_sync_failure_count_{0U};

    std::atomic<uint64_t> video_frames_enqueued_count_{0U};
    std::atomic<uint64_t> video_frames_encoded_count_{0U};
    std::atomic<uint64_t> video_encode_failure_count_{0U};
    std::atomic<uint64_t> video_packets_enqueued_count_{0U};
    std::atomic<uint64_t> video_packets_dropped_count_{0U};
    std::atomic<uint64_t> video_sink_failure_count_{0U};

    std::atomic<std::size_t> camera_buffer_count_at_start_{0U};
    std::atomic<uint32_t> camera_outstanding_before_stop_{0U};
    std::atomic<uint64_t> broker_outstanding_frames_before_stop_{0U};
    std::atomic<uint64_t> broker_outstanding_leases_before_stop_{0U};

    LatencyAccumulator input_slot_wait_;
    LatencyAccumulator latest_frame_queue_wait_;
    LatencyAccumulator capture_to_preprocess_start_;
    LatencyAccumulator preprocess_duration_;
    LatencyAccumulator rknn_input_submit_;
    LatencyAccumulator rknn_output_bind_;
    LatencyAccumulator rknn_bind_total_;
    LatencyAccumulator rknn_run_;
    LatencyAccumulator rknn_output_get_;
    LatencyAccumulator rknn_output_release_;
    LatencyAccumulator rknn_total_;
    LatencyAccumulator postprocess_duration_;
    LatencyAccumulator capture_to_result_;
    LatencyAccumulator result_age_;
};

}  // namespace visionarm
