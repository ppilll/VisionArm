#include "pipeline/inference_pipeline.h"

#include "camera/dmabuf_cpu_sync.h"
#include "common/monotonic_clock.h"

#include <algorithm>
#include <exception>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

namespace visionarm {

InferencePipeline::InferencePipeline(
    InferencePipelineConfig config,
    ICameraSource* camera,
    ICaptureBufferBroker* broker,
    IImagePreprocessor* preprocessor,
    RknnEngine* engine,
    ITargetPostprocessor* postprocessor,
    IPerceptionSink* result_sink,
    IVideoEncoder* video_encoder,
    IEncodedPacketSink* encoded_packet_sink)
    : config_(config),
      camera_(camera),
      broker_(broker),
      preprocessor_(preprocessor),
      engine_(engine),
      postprocessor_(postprocessor),
      result_sink_(result_sink),
      video_encoder_(video_encoder),
      encoded_packet_sink_(encoded_packet_sink),
      captured_frames_(std::max<std::size_t>(
          1U, config.captured_frame_queue_capacity)),
      video_frames_(std::max<std::size_t>(
          1U, config.video_frame_queue_capacity)),
      encoded_packets_(std::max<std::size_t>(
          1U, config.encoded_packet_queue_capacity)),
      prepared_frames_(std::max<std::size_t>(
          1U, config.prepared_frame_queue_capacity)),
      completed_frames_(std::max<std::size_t>(
          1U, config.completed_frame_queue_capacity)),
      free_input_slots_(std::max<std::size_t>(
          1U, engine != nullptr ? engine->input_slot_count() : 0U)),
      free_output_slots_(std::max<std::size_t>(
          1U, engine != nullptr ? engine->output_slot_count() : 0U)),
      input_slot_wait_(config.latency_sample_capacity),
      latest_frame_queue_wait_(config.latency_sample_capacity),
      capture_to_preprocess_start_(config.latency_sample_capacity),
      preprocess_duration_(config.latency_sample_capacity),
      rknn_input_submit_(config.latency_sample_capacity),
      rknn_output_bind_(config.latency_sample_capacity),
      rknn_bind_total_(config.latency_sample_capacity),
      rknn_run_(config.latency_sample_capacity),
      rknn_output_get_(config.latency_sample_capacity),
      rknn_output_release_(config.latency_sample_capacity),
      rknn_total_(config.latency_sample_capacity),
      postprocess_duration_(config.latency_sample_capacity),
      capture_to_result_(config.latency_sample_capacity),
      result_age_(config.latency_sample_capacity) {}

InferencePipeline::~InferencePipeline() {
    Stop();
}

bool InferencePipeline::Start() {
    if (started_once_) {
        std::cerr << "InferencePipeline supports one Start/Stop lifecycle\n";
        return false;
    }

    const bool video_dependencies_valid =
        !config_.enable_video ||
        (video_encoder_ != nullptr && encoded_packet_sink_ != nullptr &&
         video_encoder_->initialized() &&
         config_.video_frame_queue_capacity > 0U &&
         config_.encoded_packet_queue_capacity > 0U);

    if (camera_ == nullptr || broker_ == nullptr || preprocessor_ == nullptr ||
        engine_ == nullptr || postprocessor_ == nullptr ||
        result_sink_ == nullptr || !engine_->initialized() ||
        engine_->input_slot_count() == 0U ||
        engine_->output_slot_count() == 0U ||
        config_.captured_frame_queue_capacity == 0U ||
        config_.prepared_frame_queue_capacity == 0U ||
        config_.completed_frame_queue_capacity == 0U ||
        config_.latency_sample_capacity == 0U ||
        broker_->GetSnapshot().closed || !video_dependencies_valid) {
        std::cerr << "InferencePipeline dependencies/config are invalid\n";
        return false;
    }

    if (preprocessor_->destination_access() ==
            PreprocessorDestinationAccess::DMA_DEVICE_WRITE &&
        engine_->io_mode() == RknnIoMode::INPUTS_SET_PREALLOC_OUTPUT) {
        std::cerr
            << "RGA/device-written input requires an rknn_set_io_mem mode\n";
        return false;
    }

    for (std::size_t index = 0U;
         index < engine_->input_slot_count(); ++index) {
        if (!free_input_slots_.TryPush(index)) {
            return false;
        }
    }
    for (std::size_t index = 0U;
         index < engine_->output_slot_count(); ++index) {
        if (!free_output_slots_.TryPush(index)) {
            return false;
        }
    }

    if (config_.enable_video) {
        const std::vector<EncodedPacket> headers =
            video_encoder_->CodecConfigPackets();
        for (const EncodedPacket& header : headers) {
            if (!encoded_packets_.TryPush(header)) {
                std::cerr << "encoded packet queue cannot hold codec header\n";
                return false;
            }
            video_packets_enqueued_count_.fetch_add(
                1U, std::memory_order_relaxed);
        }
    }

    try {
        camera_->Start();
        camera_buffer_count_at_start_.store(
            camera_->buffer_count(), std::memory_order_release);
        stop_requested_.store(false, std::memory_order_release);
        fatal_error_.store(false, std::memory_order_release);
        running_.store(true, std::memory_order_release);
        started_once_ = true;

        if (config_.enable_video) {
            encoded_packet_thread_ =
                std::thread(&InferencePipeline::EncodedPacketLoop, this);
            video_thread_ =
                std::thread(&InferencePipeline::VideoLoop, this);
        }

        if (config_.topology ==
            InferenceThreadTopology::FUSED_NPU_POSTPROCESS) {
            npu_thread_ =
                std::thread(&InferencePipeline::NpuPostprocessLoop, this);
        } else {
            postprocess_thread_ =
                std::thread(&InferencePipeline::PostprocessLoop, this);
            npu_thread_ = std::thread(&InferencePipeline::NpuLoop, this);
        }

        preprocess_thread_ =
            std::thread(&InferencePipeline::PreprocessLoop, this);
        capture_thread_ =
            std::thread(&InferencePipeline::CaptureLoop, this);
        return true;
    } catch (const std::exception& error) {
        std::cerr << "pipeline start failed: " << error.what() << '\n';
    } catch (...) {
        std::cerr << "pipeline start failed with an unknown exception\n";
    }

    SignalFailure();
    Stop();
    return false;
}

void InferencePipeline::SignalFailure() noexcept {
    fatal_error_.store(true, std::memory_order_release);
    stop_requested_.store(true, std::memory_order_release);
    running_.store(false, std::memory_order_release);
    if (broker_ != nullptr) {
        broker_->Close();
    }
    if (camera_ != nullptr) {
        camera_->RequestStop();
    }
    captured_frames_.Stop();
    video_frames_.Stop();
    encoded_packets_.Stop();
    prepared_frames_.Stop();
    completed_frames_.Stop();
    free_input_slots_.Stop();
    free_output_slots_.Stop();
}

void InferencePipeline::Stop() noexcept {
    if (!started_once_ ||
        stop_completed_.exchange(true, std::memory_order_acq_rel)) {
        return;
    }

    // Normal shutdown closes only the producer edge first. Each producer then
    // closes its downstream queue after draining, so split mode cannot lose the
    // final CompletedFrame.
    stop_requested_.store(true, std::memory_order_release);
    running_.store(false, std::memory_order_release);
    if (camera_ != nullptr) {
        camera_->RequestStop();
    }

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (video_thread_.joinable()) {
        video_thread_.join();
    }
    if (preprocess_thread_.joinable()) {
        preprocess_thread_.join();
    }
    if (npu_thread_.joinable()) {
        npu_thread_.join();
    }
    if (postprocess_thread_.joinable()) {
        postprocess_thread_.join();
    }

    // No more encoded packets can be produced after the video thread exits.
    encoded_packets_.Stop();
    if (encoded_packet_thread_.joinable()) {
        encoded_packet_thread_.join();
    }

    if (encoded_packet_sink_ != nullptr) {
        encoded_packet_sink_->Flush();
    }

    if (broker_ != nullptr) {
        const CaptureBufferBrokerSnapshot broker_snapshot =
            broker_->GetSnapshot();
        broker_outstanding_frames_before_stop_.store(
            broker_snapshot.outstanding_frames, std::memory_order_release);
        broker_outstanding_leases_before_stop_.store(
            broker_snapshot.outstanding_leases, std::memory_order_release);
    }
    if (camera_ != nullptr) {
        camera_outstanding_before_stop_.store(
            camera_->outstanding_buffers(), std::memory_order_release);
        camera_->Stop();
    }

    graceful_shutdown_completed_.store(
        !fatal_error_.load(std::memory_order_acquire) &&
            camera_outstanding_before_stop_.load(std::memory_order_acquire) == 0U &&
            broker_outstanding_frames_before_stop_.load(
                std::memory_order_acquire) == 0U &&
            broker_outstanding_leases_before_stop_.load(
                std::memory_order_acquire) == 0U,
        std::memory_order_release);
}

void InferencePipeline::RequeueDirect(
    const CaptureFrameView& frame) noexcept {
    if (camera_ == nullptr || !camera_->Requeue(MakeRequeueRequest(frame))) {
        requeue_failure_count_.fetch_add(1U, std::memory_order_relaxed);
    }
}

void InferencePipeline::DrainRequeueRequests() noexcept {
    if (broker_ == nullptr || camera_ == nullptr) {
        return;
    }

    RequeueRequest request;
    while (broker_->TryPopRequeue(&request)) {
        if (!camera_->Requeue(request)) {
            requeue_failure_count_.fetch_add(1U, std::memory_order_relaxed);
            SignalFailure();
            break;
        }
    }
}

void InferencePipeline::DrainRequeueRequestsUntilClosed() noexcept {
    if (broker_ == nullptr || camera_ == nullptr) {
        return;
    }

    RequeueRequest request;
    while (broker_->WaitPopRequeue(&request)) {
        if (!camera_->Requeue(request)) {
            requeue_failure_count_.fetch_add(1U, std::memory_order_relaxed);
        }
    }
}

void InferencePipeline::CaptureLoop() noexcept {
    try {
        while (!stop_requested_.load(std::memory_order_acquire)) {
            DrainRequeueRequests();

            CaptureFrameView frame;
            const CaptureResult result = camera_->Capture(&frame);
            if (result == CaptureResult::TIMEOUT) {
                camera_timeout_count_.fetch_add(1U, std::memory_order_relaxed);
                continue;
            }
            if (result == CaptureResult::WAKE) {
                camera_wake_count_.fetch_add(1U, std::memory_order_relaxed);
                DrainRequeueRequests();
                continue;
            }
            if (result == CaptureResult::DROPPED) {
                driver_dropped_count_.fetch_add(1U, std::memory_order_relaxed);
                continue;
            }
            if (result == CaptureResult::STOPPED) {
                break;
            }

            captured_count_.fetch_add(1U, std::memory_order_relaxed);
            const CaptureConsumerMask consumers =
                kInferenceConsumer |
                (config_.enable_video ? kVideoEncoderConsumer : 0U);

            CaptureDispatch dispatch;
            if (!broker_->Publish(frame, consumers, &dispatch)) {
                RequeueDirect(frame);
                if (broker_->GetSnapshot().closed) {
                    break;
                }
                continue;
            }

            FrameLeasePtr inference_lease = std::move(dispatch.inference);
            if (!inference_lease) {
                SignalFailure();
                break;
            }

            PendingInferenceFrame pending;
            pending.lease = std::move(inference_lease);
            pending.enqueue_timestamp_ns = MonotonicNowNs();
            std::optional<PendingInferenceFrame> evicted;
            if (!captured_frames_.PushLatest(
                    std::move(pending), &evicted)) {
                if (evicted.has_value() && evicted->lease) {
                    (void)evicted->lease->Release(
                        FrameReleaseReason::PIPELINE_STOP);
                }
                if (dispatch.video_encoder) {
                    (void)dispatch.video_encoder->Release(
                        FrameReleaseReason::PIPELINE_STOP);
                }
                break;
            }

            if (evicted.has_value() && evicted->lease) {
                replaced_count_.fetch_add(1U, std::memory_order_relaxed);
                (void)evicted->lease->Release(
                    FrameReleaseReason::REPLACED_BY_NEWER_FRAME);
            }

            if (config_.enable_video) {
                if (!dispatch.video_encoder ||
                    !video_frames_.WaitPush(std::move(dispatch.video_encoder))) {
                    if (dispatch.video_encoder) {
                        (void)dispatch.video_encoder->Release(
                            FrameReleaseReason::PIPELINE_STOP);
                    }
                    break;
                }
                video_frames_enqueued_count_.fetch_add(
                    1U, std::memory_order_relaxed);
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "capture thread failed: " << error.what() << '\n';
        SignalFailure();
    } catch (...) {
        std::cerr << "capture thread failed with an unknown exception\n";
        SignalFailure();
    }

    // Producer-owned close chain.
    captured_frames_.Stop();
    video_frames_.Stop();
    broker_->Close();
    DrainRequeueRequests();
    DrainRequeueRequestsUntilClosed();
}

void InferencePipeline::VideoLoop() noexcept {
    FrameLeasePtr source;
    try {
        while (video_frames_.WaitPop(&source)) {
            if (!source) {
                continue;
            }
            if (fatal_error_.load(std::memory_order_acquire)) {
                (void)source->Release(FrameReleaseReason::PIPELINE_STOP);
                source.reset();
                continue;
            }

            std::vector<EncodedPacket> packets;
            const bool encoded =
                video_encoder_->Encode(source->frame(), &packets);
            (void)source->Release(
                encoded ? FrameReleaseReason::COMPLETED
                        : FrameReleaseReason::PROCESSING_ERROR);
            source.reset();

            if (!encoded) {
                video_encode_failure_count_.fetch_add(
                    1U, std::memory_order_relaxed);
                SignalFailure();
                break;
            }
            video_frames_encoded_count_.fetch_add(
                1U, std::memory_order_relaxed);

            for (EncodedPacket& packet : packets) {
                if (!encoded_packets_.WaitPush(std::move(packet))) {
                    if (!fatal_error_.load(std::memory_order_acquire)) {
                        video_packets_dropped_count_.fetch_add(
                            1U, std::memory_order_relaxed);
                    }
                    break;
                }
                video_packets_enqueued_count_.fetch_add(
                    1U, std::memory_order_relaxed);
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "video thread failed: " << error.what() << '\n';
        if (source && source->valid()) {
            (void)source->Release(FrameReleaseReason::PROCESSING_ERROR);
        }
        SignalFailure();
    } catch (...) {
        std::cerr << "video thread failed with an unknown exception\n";
        if (source && source->valid()) {
            (void)source->Release(FrameReleaseReason::PROCESSING_ERROR);
        }
        SignalFailure();
    }

    source.reset();
    FrameLeasePtr pending;
    while (video_frames_.TryPop(&pending)) {
        if (pending && pending->valid()) {
            (void)pending->Release(FrameReleaseReason::PIPELINE_STOP);
        }
        pending.reset();
    }
}

void InferencePipeline::EncodedPacketLoop() noexcept {
    EncodedPacket packet;
    try {
        while (encoded_packets_.WaitPop(&packet)) {
            if (!encoded_packet_sink_->Write(packet)) {
                video_sink_failure_count_.fetch_add(
                    1U, std::memory_order_relaxed);
                SignalFailure();
                break;
            }
        }
        encoded_packet_sink_->Flush();
    } catch (const std::exception& error) {
        std::cerr << "encoded packet sink failed: " << error.what() << '\n';
        SignalFailure();
    } catch (...) {
        std::cerr << "encoded packet sink failed with unknown exception\n";
        SignalFailure();
    }
}

void InferencePipeline::PreprocessLoop() noexcept {
    PendingInferenceFrame pending;
    try {
        while (true) {
            // Final R6 policy: wait for an input slot first. While this thread
            // waits, Capture keeps replacing the single pending latest frame.
            const int64_t wait_start_ns = MonotonicNowNs();
            std::size_t input_slot_index = 0U;
            if (!free_input_slots_.WaitPop(&input_slot_index)) {
                break;
            }
            const int64_t slot_acquired_ns = MonotonicNowNs();

            if (!captured_frames_.WaitPop(&pending)) {
                (void)free_input_slots_.TryPush(input_slot_index);
                break;
            }
            const int64_t latest_dequeue_ns = MonotonicNowNs();

            if (!pending.lease) {
                (void)free_input_slots_.TryPush(input_slot_index);
                continue;
            }
            if (fatal_error_.load(std::memory_order_acquire)) {
                (void)pending.lease->Release(
                    FrameReleaseReason::PIPELINE_STOP);
                pending = {};
                (void)free_input_slots_.TryPush(input_slot_index);
                continue;
            }

            const ModelInputBufferView* destination =
                engine_->input_buffer(input_slot_index);
            if (destination == nullptr) {
                preprocess_failure_count_.fetch_add(
                    1U, std::memory_order_relaxed);
                (void)pending.lease->Release(
                    FrameReleaseReason::PROCESSING_ERROR);
                pending = {};
                (void)free_input_slots_.TryPush(input_slot_index);
                continue;
            }

            const CaptureFrameView& frame = pending.lease->frame();
            PreparedFrame prepared;
            prepared.input_slot_index = input_slot_index;
            prepared.identity = frame.identity;
            prepared.inference_enqueue_ns = pending.enqueue_timestamp_ns;
            prepared.input_slot_wait_start_ns = wait_start_ns;
            prepared.input_slot_acquired_ns = slot_acquired_ns;
            prepared.latest_frame_dequeue_ns = latest_dequeue_ns;
            prepared.preprocess_start_ns = MonotonicNowNs();

            bool source_sync_ok = true;
            DmabufCpuAccessGuard source_guard;
            if (preprocessor_->source_access() ==
                PreprocessorSourceAccess::CPU_MMAP_READ) {
                source_sync_ok = source_guard.Begin(
                    frame, DmabufCpuAccessMode::READ);
                if (!source_sync_ok) {
                    dmabuf_sync_failure_count_.fetch_add(
                        1U, std::memory_order_relaxed);
                }
            }

            const bool needs_bound_input_sync =
                preprocessor_->destination_access() ==
                    PreprocessorDestinationAccess::CPU_MMAP_WRITE &&
                engine_->io_mode() !=
                    RknnIoMode::INPUTS_SET_PREALLOC_OUTPUT;
            bool destination_sync_ok = true;
            DmabufCpuAccessGuard destination_guard;
            if (needs_bound_input_sync) {
                destination_sync_ok = destination_guard.BeginFd(
                    destination->dma_fd,
                    DmabufCpuAccessMode::WRITE);
                if (!destination_sync_ok) {
                    dmabuf_sync_failure_count_.fetch_add(
                        1U, std::memory_order_relaxed);
                }
            }

            const bool source_ready =
                source_sync_ok || !config_.require_cpu_dmabuf_sync;
            const bool destination_ready =
                destination_sync_ok ||
                !config_.require_bound_input_dmabuf_sync;

            bool process_ok = false;
            if (source_ready && destination_ready) {
                process_ok = preprocessor_->Process(
                    frame, *destination, &prepared.transform);
            }

            if (destination_guard.active() && !destination_guard.End()) {
                dmabuf_sync_failure_count_.fetch_add(
                    1U, std::memory_order_relaxed);
                destination_sync_ok = false;
            }
            if (source_guard.active() && !source_guard.End()) {
                dmabuf_sync_failure_count_.fetch_add(
                    1U, std::memory_order_relaxed);
                source_sync_ok = false;
            }

            prepared.preprocess_end_ns = MonotonicNowNs();
            const bool processing_ok =
                process_ok &&
                (source_sync_ok || !config_.require_cpu_dmabuf_sync) &&
                (destination_sync_ok ||
                 !config_.require_bound_input_dmabuf_sync);
            (void)pending.lease->Release(
                processing_ok
                    ? FrameReleaseReason::COMPLETED
                    : FrameReleaseReason::PROCESSING_ERROR);
            pending = {};

            if (!processing_ok) {
                preprocess_failure_count_.fetch_add(
                    1U, std::memory_order_relaxed);
                (void)free_input_slots_.TryPush(input_slot_index);
                continue;
            }

            if (!prepared_frames_.WaitPush(std::move(prepared))) {
                (void)free_input_slots_.TryPush(input_slot_index);
                break;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "preprocess thread failed: " << error.what() << '\n';
        if (pending.lease && pending.lease->valid()) {
            (void)pending.lease->Release(
                FrameReleaseReason::PROCESSING_ERROR);
        }
        SignalFailure();
    } catch (...) {
        std::cerr << "preprocess thread failed with unknown exception\n";
        if (pending.lease && pending.lease->valid()) {
            (void)pending.lease->Release(
                FrameReleaseReason::PROCESSING_ERROR);
        }
        SignalFailure();
    }

    pending = {};
    PendingInferenceFrame queued;
    while (captured_frames_.TryPop(&queued)) {
        if (queued.lease && queued.lease->valid()) {
            (void)queued.lease->Release(FrameReleaseReason::PIPELINE_STOP);
        }
        queued = {};
    }
    prepared_frames_.Stop();
}

void InferencePipeline::NpuLoop() noexcept {
    PreparedFrame prepared;
    try {
        while (prepared_frames_.WaitPop(&prepared)) {
            std::size_t output_slot_index = 0U;
            if (!free_output_slots_.WaitPop(&output_slot_index)) {
                (void)free_input_slots_.TryPush(prepared.input_slot_index);
                break;
            }

            CompletedFrame completed;
            completed.output_slot_index = output_slot_index;
            completed.prepared = prepared;
            completed.inference_start_ns = MonotonicNowNs();
            const bool ok = engine_->Run(
                prepared.input_slot_index,
                output_slot_index,
                &completed.rknn_timing);
            completed.inference_end_ns = MonotonicNowNs();
            (void)free_input_slots_.TryPush(prepared.input_slot_index);

            if (!ok) {
                inference_failure_count_.fetch_add(
                    1U, std::memory_order_relaxed);
                (void)free_output_slots_.TryPush(output_slot_index);
                continue;
            }
            inference_success_count_.fetch_add(
                1U, std::memory_order_relaxed);

            if (!completed_frames_.WaitPush(std::move(completed))) {
                (void)free_output_slots_.TryPush(output_slot_index);
                break;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "NPU thread failed: " << error.what() << '\n';
        SignalFailure();
    } catch (...) {
        std::cerr << "NPU thread failed with unknown exception\n";
        SignalFailure();
    }

    // This close is the split-mode fix: Postprocess drains every completed
    // frame, including the final frame produced during normal shutdown.
    completed_frames_.Stop();
}

bool InferencePipeline::ProcessCompletedFrame(
    CompletedFrame* completed) noexcept {
    if (completed == nullptr) {
        return false;
    }

    const std::vector<TensorView>* outputs =
        engine_->output_views(completed->output_slot_index);
    if (outputs == nullptr) {
        postprocess_failure_count_.fetch_add(
            1U, std::memory_order_relaxed);
        (void)free_output_slots_.TryPush(completed->output_slot_index);
        return false;
    }

    DmabufCpuAccessGuard output_guard;
    if (engine_->io_mode() != RknnIoMode::INPUTS_SET_PREALLOC_OUTPUT) {
        const std::vector<int>* output_fds =
            engine_->output_dma_fds(completed->output_slot_index);
        if (output_fds == nullptr ||
            !output_guard.BeginFds(
                output_fds->data(), output_fds->size(),
                DmabufCpuAccessMode::READ)) {
            dmabuf_sync_failure_count_.fetch_add(
                1U, std::memory_order_relaxed);
            (void)free_output_slots_.TryPush(completed->output_slot_index);
            return false;
        }
    }

    PerceptionPacket packet;
    packet.identity = completed->prepared.identity;
    packet.transform = completed->prepared.transform;
    packet.inference_enqueue_ns = completed->prepared.inference_enqueue_ns;
    packet.input_slot_wait_start_ns =
        completed->prepared.input_slot_wait_start_ns;
    packet.input_slot_acquired_ns =
        completed->prepared.input_slot_acquired_ns;
    packet.latest_frame_dequeue_ns =
        completed->prepared.latest_frame_dequeue_ns;
    packet.preprocess_start_ns = completed->prepared.preprocess_start_ns;
    packet.preprocess_end_ns = completed->prepared.preprocess_end_ns;
    packet.inference_start_ns = completed->inference_start_ns;
    packet.inference_end_ns = completed->inference_end_ns;
    packet.rknn_input_submit_ns = completed->rknn_timing.input_submit_ns;
    packet.rknn_output_bind_ns = completed->rknn_timing.output_bind_ns;
    packet.rknn_run_ns = completed->rknn_timing.run_ns;
    packet.rknn_output_get_ns = completed->rknn_timing.output_get_ns;
    packet.rknn_output_release_ns = completed->rknn_timing.output_release_ns;
    packet.rknn_total_ns = completed->rknn_timing.total_ns;

    bool postprocess_ok = false;
    try {
        packet.result = postprocessor_->Process(*outputs, packet.transform);
        packet.postprocess_end_ns = MonotonicNowNs();
        packet.generated_timestamp_ns = packet.postprocess_end_ns;
        if (packet.identity.capture_timestamp_ns > 0 &&
            packet.generated_timestamp_ns >=
                packet.identity.capture_timestamp_ns) {
            packet.result_age_ns =
                packet.generated_timestamp_ns -
                packet.identity.capture_timestamp_ns;
        }
        postprocess_ok = true;
    } catch (const std::exception& error) {
        std::cerr << "postprocess failed: " << error.what() << '\n';
        postprocess_failure_count_.fetch_add(
            1U, std::memory_order_relaxed);
    } catch (...) {
        std::cerr << "postprocess failed with unknown exception\n";
        postprocess_failure_count_.fetch_add(
            1U, std::memory_order_relaxed);
    }

    bool output_sync_ok = true;
    if (output_guard.active() && !output_guard.End()) {
        dmabuf_sync_failure_count_.fetch_add(
            1U, std::memory_order_relaxed);
        output_sync_ok = false;
    }

    (void)free_output_slots_.TryPush(completed->output_slot_index);
    if (!postprocess_ok || !output_sync_ok) {
        return false;
    }

    RecordCompletedTiming(packet);
    postprocess_success_count_.fetch_add(
        1U, std::memory_order_relaxed);
    try {
        result_sink_->Publish(std::move(packet));
    } catch (const std::exception& error) {
        std::cerr << "result sink failed: " << error.what() << '\n';
        result_publish_failure_count_.fetch_add(
            1U, std::memory_order_relaxed);
        return false;
    } catch (...) {
        std::cerr << "result sink failed with unknown exception\n";
        result_publish_failure_count_.fetch_add(
            1U, std::memory_order_relaxed);
        return false;
    }
    return true;
}

void InferencePipeline::PostprocessLoop() noexcept {
    CompletedFrame completed;
    try {
        while (completed_frames_.WaitPop(&completed)) {
            (void)ProcessCompletedFrame(&completed);
        }
        split_final_completed_frame_drained_.store(
            completed_frames_.Snapshot().current_size == 0U,
            std::memory_order_release);
    } catch (const std::exception& error) {
        std::cerr << "postprocess thread failed: " << error.what() << '\n';
        SignalFailure();
    } catch (...) {
        std::cerr << "postprocess thread failed with unknown exception\n";
        SignalFailure();
    }
}

void InferencePipeline::NpuPostprocessLoop() noexcept {
    PreparedFrame prepared;
    try {
        while (prepared_frames_.WaitPop(&prepared)) {
            std::size_t output_slot_index = 0U;
            if (!free_output_slots_.WaitPop(&output_slot_index)) {
                (void)free_input_slots_.TryPush(prepared.input_slot_index);
                break;
            }

            CompletedFrame completed;
            completed.output_slot_index = output_slot_index;
            completed.prepared = prepared;
            completed.inference_start_ns = MonotonicNowNs();
            const bool ok = engine_->Run(
                prepared.input_slot_index,
                output_slot_index,
                &completed.rknn_timing);
            completed.inference_end_ns = MonotonicNowNs();
            (void)free_input_slots_.TryPush(prepared.input_slot_index);

            if (!ok) {
                inference_failure_count_.fetch_add(
                    1U, std::memory_order_relaxed);
                (void)free_output_slots_.TryPush(output_slot_index);
                continue;
            }
            inference_success_count_.fetch_add(
                1U, std::memory_order_relaxed);
            (void)ProcessCompletedFrame(&completed);
        }
    } catch (const std::exception& error) {
        std::cerr << "fused NPU/postprocess thread failed: "
                  << error.what() << '\n';
        SignalFailure();
    } catch (...) {
        std::cerr << "fused NPU/postprocess thread failed\n";
        SignalFailure();
    }
}

void InferencePipeline::RecordCompletedTiming(
    const PerceptionPacket& packet) noexcept {
    input_slot_wait_.Add(
        packet.input_slot_acquired_ns - packet.input_slot_wait_start_ns);
    latest_frame_queue_wait_.Add(
        packet.latest_frame_dequeue_ns - packet.inference_enqueue_ns);
    capture_to_preprocess_start_.Add(
        packet.preprocess_start_ns - packet.identity.capture_timestamp_ns);
    preprocess_duration_.Add(
        packet.preprocess_end_ns - packet.preprocess_start_ns);
    rknn_input_submit_.Add(packet.rknn_input_submit_ns);
    rknn_output_bind_.Add(packet.rknn_output_bind_ns);
    rknn_bind_total_.Add(
        packet.rknn_input_submit_ns + packet.rknn_output_bind_ns);
    rknn_run_.Add(packet.rknn_run_ns);
    rknn_output_get_.Add(packet.rknn_output_get_ns);
    rknn_output_release_.Add(packet.rknn_output_release_ns);
    rknn_total_.Add(packet.rknn_total_ns);
    postprocess_duration_.Add(
        packet.postprocess_end_ns - packet.inference_end_ns);
    capture_to_result_.Add(
        packet.postprocess_end_ns - packet.identity.capture_timestamp_ns);
    result_age_.Add(packet.result_age_ns);
}

PipelineStatsSnapshot InferencePipeline::stats() const noexcept {
    PipelineStatsSnapshot snapshot;
    snapshot.captured_frames =
        captured_count_.load(std::memory_order_relaxed);
    snapshot.camera_timeouts =
        camera_timeout_count_.load(std::memory_order_relaxed);
    snapshot.camera_wakes =
        camera_wake_count_.load(std::memory_order_relaxed);
    snapshot.driver_dropped_frames =
        driver_dropped_count_.load(std::memory_order_relaxed);
    snapshot.replaced_waiting_frames =
        replaced_count_.load(std::memory_order_relaxed);
    snapshot.skipped_no_input_slot =
        skipped_no_input_count_.load(std::memory_order_relaxed);
    snapshot.preprocess_failures =
        preprocess_failure_count_.load(std::memory_order_relaxed);
    snapshot.inference_successes =
        inference_success_count_.load(std::memory_order_relaxed);
    snapshot.inference_failures =
        inference_failure_count_.load(std::memory_order_relaxed);
    snapshot.postprocess_successes =
        postprocess_success_count_.load(std::memory_order_relaxed);
    snapshot.postprocess_failures =
        postprocess_failure_count_.load(std::memory_order_relaxed);
    snapshot.result_publish_failures =
        result_publish_failure_count_.load(std::memory_order_relaxed);
    snapshot.requeue_failures =
        requeue_failure_count_.load(std::memory_order_relaxed);
    snapshot.dmabuf_sync_failures =
        dmabuf_sync_failure_count_.load(std::memory_order_relaxed);
    snapshot.video_frames_enqueued =
        video_frames_enqueued_count_.load(std::memory_order_relaxed);
    snapshot.video_frames_encoded =
        video_frames_encoded_count_.load(std::memory_order_relaxed);
    snapshot.video_encode_failures =
        video_encode_failure_count_.load(std::memory_order_relaxed);
    snapshot.video_packets_enqueued =
        video_packets_enqueued_count_.load(std::memory_order_relaxed);
    snapshot.video_packets_dropped =
        video_packets_dropped_count_.load(std::memory_order_relaxed);
    snapshot.video_sink_failures =
        video_sink_failure_count_.load(std::memory_order_relaxed);

    snapshot.camera_buffer_count_at_start =
        camera_buffer_count_at_start_.load(std::memory_order_relaxed);
    snapshot.camera_outstanding_before_stop =
        camera_outstanding_before_stop_.load(std::memory_order_relaxed);
    snapshot.broker_outstanding_frames_before_camera_stop =
        broker_outstanding_frames_before_stop_.load(std::memory_order_relaxed);
    snapshot.broker_outstanding_leases_before_camera_stop =
        broker_outstanding_leases_before_stop_.load(std::memory_order_relaxed);
    snapshot.fatal_error = fatal_error_.load(std::memory_order_relaxed);
    snapshot.graceful_shutdown_completed =
        graceful_shutdown_completed_.load(std::memory_order_relaxed);
    snapshot.split_final_completed_frame_drained =
        config_.topology == InferenceThreadTopology::FUSED_NPU_POSTPROCESS ||
        split_final_completed_frame_drained_.load(std::memory_order_relaxed);
    snapshot.topology = config_.topology;

    snapshot.captured_frame_queue = captured_frames_.Snapshot();
    snapshot.prepared_frame_queue = prepared_frames_.Snapshot();
    snapshot.completed_frame_queue = completed_frames_.Snapshot();
    snapshot.video_frame_queue = video_frames_.Snapshot();
    snapshot.encoded_packet_queue = encoded_packets_.Snapshot();

    snapshot.timing.input_slot_wait = input_slot_wait_.Snapshot();
    snapshot.timing.latest_frame_queue_wait =
        latest_frame_queue_wait_.Snapshot();
    snapshot.timing.capture_to_preprocess_start =
        capture_to_preprocess_start_.Snapshot();
    snapshot.timing.preprocess = preprocess_duration_.Snapshot();
    snapshot.timing.rknn_input_submit = rknn_input_submit_.Snapshot();
    snapshot.timing.rknn_output_bind = rknn_output_bind_.Snapshot();
    snapshot.timing.rknn_bind_total = rknn_bind_total_.Snapshot();
    snapshot.timing.rknn_run = rknn_run_.Snapshot();
    snapshot.timing.rknn_output_get = rknn_output_get_.Snapshot();
    snapshot.timing.rknn_output_release = rknn_output_release_.Snapshot();
    snapshot.timing.rknn_total = rknn_total_.Snapshot();
    snapshot.timing.postprocess = postprocess_duration_.Snapshot();
    snapshot.timing.capture_to_result = capture_to_result_.Snapshot();
    snapshot.timing.result_age = result_age_.Snapshot();
    return snapshot;
}

}  // namespace visionarm
