#include "runtime/runtime_report.h"

#include "runtime/runtime_app.h"
#include "runtime_internal.h"

#include "camera/capture_buffer.h"
#include "control/target_state_machine.h"
#include "observability/logger.h"
#include "observability/runtime_report.h"
#include "perception/rga_preprocess.h"
#include "video/video_support.h"
#include "video/video_types.h"

#if defined(VISIONARM_HAS_ALSA_AUDIO)
#include "audio/audio_capture.h"
#include "pipeline/bounded_queue.h"
#endif

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <ostream>
#include <sstream>
#include <utility>

namespace visionarm::runtime {

std::int64_t detail::ReadVmRssKb() noexcept {
    std::ifstream stream("/proc/self/status");
    std::string key;
    while (stream >> key) {
        if (key == "VmRSS:") {
            std::int64_t value = 0;
            std::string unit;
            stream >> value >> unit;
            return value;
        }
        stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return -1;
}

void detail::RssSamples::Add(std::int64_t value) noexcept {
    if (value < 0) return;
    if (samples == 0U) {
        first_kb = value;
        minimum_kb = value;
        maximum_kb = value;
    }
    last_kb = value;
    minimum_kb = std::min(minimum_kb, value);
    maximum_kb = std::max(maximum_kb, value);
    ++samples;
}

std::int64_t detail::RssSamples::GrowthKb() const noexcept {
    return first_kb >= 0 && last_kb >= 0 ? last_kb - first_kb : 0;
}

#if defined(VISIONARM_HAS_ALSA_AUDIO)
void detail::AudioTimelineStats::Consume(
    const TimedAudioChunk& chunk) noexcept {
    ++chunks;
    frames += chunk.raw.timing.frame_count;
    bytes += chunk.raw.pcm.size();
    if (chunk.media.reanchored) ++reanchors;
    if (chunk.raw.timing.discontinuity_before) ++discontinuities;
    if (first_pts_ns < 0) first_pts_ns = chunk.media.pts_ns;
    if (last_end_pts_ns >= 0) {
        if (chunk.media.pts_ns < last_end_pts_ns) {
            ++pts_regressions;
        } else {
            const std::int64_t gap_ns = chunk.media.pts_ns - last_end_pts_ns;
            maximum_forward_gap_ns = std::max(maximum_forward_gap_ns, gap_ns);
            if (!chunk.media.reanchored && gap_ns != 0) {
                ++continuous_pts_mismatches;
            }
        }
    }
    const std::int64_t abs_error_ns = chunk.media.timing_error_ns < 0
        ? -chunk.media.timing_error_ns : chunk.media.timing_error_ns;
    maximum_abs_timing_error_ns =
        std::max(maximum_abs_timing_error_ns, abs_error_ns);
    last_pts_ns = chunk.media.pts_ns;
    last_end_pts_ns = chunk.media.pts_ns + chunk.media.duration_ns;
}

void detail::DrainAudioQueue(
    BoundedQueue<TimedAudioChunk>* queue,
    AudioTimelineStats* stats) {
    if (queue == nullptr || stats == nullptr) return;
    TimedAudioChunk chunk;
    while (queue->TryPop(&chunk)) stats->Consume(chunk);
}
#endif

double detail::RatePerSecond(
    std::uint64_t count, double seconds) noexcept {
    return seconds > 0.0 ? static_cast<double>(count) / seconds : 0.0;
}

bool detail::EvaluateRuntimePass(
    bool completed_requested_duration,
    bool terminated_by_signal,
    const PipelineStatsSnapshot& stats,
    const CaptureBufferBrokerSnapshot& broker_stats,
    const VideoEncoderSnapshot& encoder_stats,
    const H265FileSinkSnapshot& sink_stats,
    const RgaPreprocessorSnapshot& rga_stats,
    const TargetStateMachineSnapshot& state_stats,
    bool control_ok,
    bool auxiliary_media_runtime_fault,
    bool audio_ok,
    bool audio_encode_ok,
    bool local_av_mux_ok,
    bool network_mux_ok,
    bool telemetry_ok,
    bool queues_ok,
    bool throughput_ok,
    bool timing_ok,
    bool rss_ok) noexcept {
    return completed_requested_duration && !terminated_by_signal &&
        stats.captured_frames > 0U && stats.video_frames_encoded > 0U &&
        stats.inference_successes > 0U && stats.postprocess_successes > 0U &&
        stats.preprocess_failures == 0U && stats.inference_failures == 0U &&
        stats.postprocess_failures == 0U &&
        stats.result_publish_failures == 0U &&
        stats.video_encode_failures == 0U && !stats.video_branch_failed &&
        stats.video_frames_dropped == 0U &&
        stats.video_packets_dropped == 0U &&
        stats.video_sink_failures == 0U && stats.requeue_failures == 0U &&
        stats.dmabuf_sync_failures == 0U &&
        stats.camera_buffer_count_at_start > 0U &&
        stats.camera_outstanding_before_stop == 0U &&
        stats.broker_outstanding_frames_before_camera_stop == 0U &&
        stats.broker_outstanding_leases_before_camera_stop == 0U &&
        !stats.fatal_error && stats.graceful_shutdown_completed &&
        stats.split_final_completed_frame_drained &&
        broker_stats.outstanding_frames == 0U &&
        broker_stats.outstanding_leases == 0U &&
        encoder_stats.encode_failures == 0U &&
        encoder_stats.source_buffer_reimports == 0U &&
        sink_stats.write_failures == 0U && sink_stats.bytes_written > 0U &&
        rga_stats.process_successes > 0U &&
        state_stats.control_sink_failures == 0U &&
        state_stats.perception_sink_failures == 0U &&
        state_stats.invalid_timestamp_packets == 0U && control_ok &&
        !auxiliary_media_runtime_fault && audio_ok && audio_encode_ok &&
        local_av_mux_ok && network_mux_ok && telemetry_ok && queues_ok &&
        throughput_ok && timing_ok && rss_ok;
}

void WriteLatency(std::ostream& stream,
                  const char* name,
                  const LatencyDistributionSnapshot& value) {
    stream << name << ".total_samples=" << value.total_samples << '\n'
           << name << ".retained_samples=" << value.retained_samples << '\n'
           << name << ".truncated=" << (value.truncated ? 1 : 0) << '\n'
           << name << ".mean_ms=" << value.mean_ms << '\n'
           << name << ".p50_ms=" << value.p50_ms << '\n'
           << name << ".p95_ms=" << value.p95_ms << '\n'
           << name << ".p99_ms=" << value.p99_ms << '\n'
           << name << ".maximum_ms=" << value.maximum_ms << '\n';
}

void WriteQueue(std::ostream& stream,
                const char* name,
                const QueueStatsSnapshot& value) {
    stream << name << ".capacity=" << value.capacity << '\n'
           << name << ".high_watermark=" << value.high_watermark << '\n'
           << name << ".current_size=" << value.current_size << '\n'
           << name << ".pushed=" << value.pushed << '\n'
           << name << ".popped=" << value.popped << '\n'
           << name << ".replaced_oldest=" << value.replaced_oldest << '\n'
           << name << ".stopped=" << (value.stopped ? 1 : 0) << '\n';
}

bool QueueBounded(const QueueStatsSnapshot& value) noexcept {
    return value.capacity > 0U &&
        value.high_watermark <= value.capacity &&
        value.current_size <= value.capacity;
}

bool QueueDrained(const QueueStatsSnapshot& value) noexcept {
    return QueueBounded(value) && value.stopped && value.current_size == 0U &&
        value.pushed == value.popped + value.replaced_oldest &&
        value.replaced_oldest == 0U;
}

bool LatencyComplete(const LatencyDistributionSnapshot& value,
                     std::uint64_t expected_samples) noexcept {
    return expected_samples > 0U &&
        value.total_samples == expected_samples &&
        value.retained_samples == value.total_samples &&
        !value.truncated;
}

int PublishRuntimeReport(
    const RuntimeOptions& options,
    bool passed,
    double observed_duration_seconds,
    const PipelineStatsSnapshot& stats,
    std::string diagnostic_report) {
    logging::Log(
        passed ? logging::LogLevel::INFO : logging::LogLevel::ERROR,
        "runtime", "shutdown pipeline_result=", (passed ? "PASS" : "FAIL"),
        " duration_sec=", observed_duration_seconds,
        " captured_frames=", stats.captured_frames,
        " inference_successes=", stats.inference_successes,
        " video_frames_encoded=", stats.video_frames_encoded);
    const bool logger_flush_ok = logging::FlushGlobalLogger();
    const logging::LoggerSnapshot logger_stats = logging::GlobalLoggerSnapshot();
    const bool product_passed =
        passed && logger_flush_ok && logger_stats.current_size == 0U &&
        logger_stats.emitted == logger_stats.accepted &&
        logger_stats.dropped ==
            logger_stats.dropped_contention + logger_stats.dropped_overflow &&
        logger_stats.high_watermark <=
            logger_stats.queue_capacity + logger_stats.critical_queue_capacity &&
        logger_stats.dropped_critical == 0U &&
        logger_stats.sink_failures == 0U;

    std::ostringstream completed;
    completed << std::move(diagnostic_report)
              << "log.queue_capacity=" << logger_stats.queue_capacity << '\n'
              << "log.critical_queue_capacity="
              << logger_stats.critical_queue_capacity << '\n'
              << "log.current_size=" << logger_stats.current_size << '\n'
              << "log.high_watermark=" << logger_stats.high_watermark << '\n'
              << "log.accepted=" << logger_stats.accepted << '\n'
              << "log.emitted=" << logger_stats.emitted << '\n'
              << "log.dropped=" << logger_stats.dropped << '\n'
              << "log.dropped_contention="
              << logger_stats.dropped_contention << '\n'
              << "log.dropped_overflow=" << logger_stats.dropped_overflow << '\n'
              << "log.dropped_critical=" << logger_stats.dropped_critical << '\n'
              << "log.sink_failures=" << logger_stats.sink_failures << '\n'
              << "log.flush_ok=" << (logger_flush_ok ? 1 : 0) << '\n'
              << "result=" << (product_passed ? "PASS" : "FAIL") << '\n';

    std::string report_error;
    const bool report_ok = options.report.empty()
        ? report::WriteRuntimeReport(
              completed.str(), options.report_level, std::cout, &report_error)
        : report::WriteRuntimeReportFile(
              options.report, completed.str(), options.report_level,
              &report_error);
    if (!report_ok) {
        logging::Log(logging::LogLevel::FATAL, "report",
                     "report write failed: ", report_error);
        (void)logging::FlushGlobalLogger();
    }
    logging::ShutdownGlobalLogger();
    return product_passed && report_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

int WriteFaultRuntimeReport(
    const RuntimeOptions* options,
    int argc,
    char** argv,
    std::string_view error_message) {
    logging::Log(logging::LogLevel::FATAL, "runtime",
                 "fatal startup/runtime exception: ", error_message);
    (void)logging::FlushGlobalLogger();
    const logging::LoggerSnapshot logger_stats = logging::GlobalLoggerSnapshot();

    report::ReportLevel report_level = report::ReportLevel::SUMMARY;
    std::string report_path = FindOptionValue(argc, argv, "--report");
    if (options != nullptr) {
        report_path = options->report;
        report_level = options->report_level;
    } else {
        const std::string level_text =
            FindOptionValue(argc, argv, "--report-level");
        (void)report::ParseReportLevel(level_text, &report_level);
    }

    std::ostringstream fault;
    fault << "module.camera.enabled=0\n"
          << "module.inference.enabled=0\n"
          << "module.video.enabled=0\n"
          << "module.audio.enabled=0\n"
          << "module.audio_encoder.enabled=0\n"
          << "module.recorder.enabled=0\n"
          << "module.network.enabled=0\n"
          << "module.telemetry.enabled=0\n"
          << "module.uart.enabled=0\n"
          << "requested_duration_seconds="
          << (options != nullptr ? options->duration_seconds : 0) << '\n'
          << "observed_duration_seconds=0\n"
          << "completed_requested_duration=0\n"
          << "terminated_by_signal=0\n"
          << "fatal_error=1\n"
          << "fatal_message=" << report::EscapeReportValue(error_message) << '\n'
          << "graceful_shutdown_completed=0\n"
          << "log.accepted=" << logger_stats.accepted << '\n'
          << "log.emitted=" << logger_stats.emitted << '\n'
          << "log.dropped=" << logger_stats.dropped << '\n'
          << "log.dropped_critical=" << logger_stats.dropped_critical << '\n'
          << "log.sink_failures=" << logger_stats.sink_failures << '\n'
          << "result=FAIL\n";
    std::string report_error;
    const bool fault_report_ok = report_path.empty()
        ? report::WriteRuntimeReport(
              fault.str(), report_level, std::cout, &report_error)
        : report::WriteRuntimeReportFile(
              report_path, fault.str(), report_level, &report_error);
    if (!fault_report_ok) {
        logging::Log(logging::LogLevel::FATAL, "report",
                     "fault report write failed: ", report_error);
        (void)logging::FlushGlobalLogger();
    }
    logging::ShutdownGlobalLogger();
    return EXIT_FAILURE;
}

}  // namespace visionarm::runtime
