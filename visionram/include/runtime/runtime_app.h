#pragma once

#include "observability/logger.h"
#include "pipeline/inference_pipeline.h"
#include "observability/runtime_report.h"

#include <cstdint>
#include <string>

namespace visionarm::runtime {

enum class ControlBackend { MOCK, UART };

struct RuntimeOptions {
    std::string device;
    std::string sensor_subdev = "/dev/v4l-subdev2";
    std::string model;
    std::string output;
    std::string report;
    report::ReportLevel report_level = report::ReportLevel::SUMMARY;
    logging::LoggerConfig log_config;
    std::string log_config_path;
    std::string input_dma_heap = "/dev/dma_heap/system-uncached-dma32";
    int width = 0;
    int height = 0;
    int fps = 0;
    int buffers = 6;
    int duration_seconds = 600;
    int timeout_ms = 2000;
    int bitrate = 0;
    int gop = 0;
    int vertical_stride = 0;
    int input_slots = 1;
    int output_slots = 1;
    int video_queue = 2;
#if defined(VISIONARM_HAS_ALSA_AUDIO)
    bool audio_enabled = true;
#else
    bool audio_enabled = false;
#endif
    std::string audio_device = "hw:1,0";
    int audio_rate = 48'000;
    int audio_channels = 2;
    int audio_period_frames = 1'024;
    int audio_buffer_frames = 4'096;
    int audio_queue = 16;
    int audio_encoded_queue = 32;
    int audio_bitrate = 128'000;
    std::string av_output;
    std::string network_url;
    int network_queue = 256;
    int network_io_timeout_ms = 1'000;
    int network_rate_bps = 0;
    int network_burst_bits = 0;
    int network_packet_size = 1'316;
    int network_send_buffer_bytes = 4 * 1'024 * 1'024;
    std::string telemetry_host;
    int telemetry_port = 5'001;
    int telemetry_interval_ms = 100;
    int telemetry_send_buffer_bytes = 1 * 1'024 * 1'024;
    int acquire_hits = 2;
    int lost_misses = 3;
    int max_result_age_ms = 100;
    int latency_samples = 65536;
    int max_rss_growth_kb = 0;
    float confidence = 0.25F;
    ControlBackend control_backend = ControlBackend::MOCK;
    std::string uart_device = "/dev/ttyS3";
    int uart_baud = 115200;
    int uart_ready_timeout_ms = 5000;
    InferenceThreadTopology topology =
        InferenceThreadTopology::FUSED_NPU_POSTPROCESS;
};

[[nodiscard]] RuntimeOptions ParseRuntimeOptions(int argc, char** argv);
[[nodiscard]] const char* ControlBackendName(ControlBackend backend) noexcept;
[[nodiscard]] std::int64_t EffectiveNetworkRateBps(
    const RuntimeOptions& options) noexcept;
[[nodiscard]] std::int64_t EffectiveNetworkBurstBits(
    const RuntimeOptions& options) noexcept;
[[nodiscard]] bool LoadEffectiveLoggerConfig(
    const RuntimeOptions& options,
    logging::LoggerConfig* config,
    std::string* error);
void ReloadLoggerFromSupervisor(const RuntimeOptions& options) noexcept;
[[nodiscard]] std::string FindOptionValue(
    int argc, char** argv, const std::string& option);

// Installs the process-level stop and log-reload handlers used by RuntimeMain.
// Keep this separate from runtime composition so the executable remains a
// signal/bootstrap-only entry point.
void InstallSignalHandlers() noexcept;

// Parses the board-runtime CLI, composes the runtime, supervises it, and
// returns a process exit code.
int RuntimeMain(int argc, char** argv);

}  // namespace visionarm::runtime
