#include "camera/capture_buffer_broker.h"
#include "camera/v4l2_camera.h"
#include "camera/v4l2_sensor_controller.h"
#include "common/monotonic_clock.h"
#include "media/media_clock.h"
#if defined(VISIONARM_HAS_ALSA_AUDIO)
#include "audio/audio_capture_worker.h"
#include "pipeline/bounded_queue.h"
#endif
#if defined(VISIONARM_HAS_AUDIO_ENCODE)
#include "audio/audio_encode_worker.h"
#include "audio/encoded_audio_sink_worker.h"
#include "media/ffmpeg_log_control.h"
#endif
#if defined(VISIONARM_HAS_AV_MUX)
#include "media/ffmpeg_mp4_muxer.h"
#endif
#if defined(VISIONARM_HAS_NETWORK_MUX)
#include "media/ffmpeg_mpegts_udp_sink.h"
#endif
#include "control/control_sink.h"
#if defined(VISIONARM_HAS_UART_CONTROL)
#include "control/uart_control_sink.h"
#include "uart/uart_link.h"
#endif
#include "inference/rknn_engine.h"
#include "logging/logger.h"
#include "pipeline/inference_pipeline.h"
#include "pipeline/latest_result_store.h"
#include "pipeline/target_state_machine.h"
#include "postprocess/yolov8_top1_postprocessor.h"
#include "preprocess/rga_letterbox_preprocessor.h"
#include "report/runtime_report.h"
#include "video/h265_file_sink.h"
#include "video/mpp_h265_encoder.h"
#include "telemetry/udp_telemetry_sink.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace {

std::atomic<bool> g_stop{false};
std::atomic<bool> g_reload_log_config{false};

constexpr int kModelWidth = 960;
constexpr int kModelHeight = 544;
constexpr int kClassCount = 1;
constexpr int kTargetClassId = 0;

void SignalHandler(int signal_number) {
#if defined(SIGHUP)
    if (signal_number == SIGHUP) {
        g_reload_log_config.store(true, std::memory_order_release);
        return;
    }
#endif
    g_stop.store(true, std::memory_order_release);
}

enum class ControlBackend {
    MOCK,
    UART,
};

const char* ControlBackendName(ControlBackend backend) noexcept {
    switch (backend) {
        case ControlBackend::MOCK: return "mock";
        case ControlBackend::UART: return "uart";
    }
    return "unknown";
}

struct Options {
    std::string device;
    std::string sensor_subdev = "/dev/v4l-subdev2";
    std::string model;
    std::string output;
    std::string report;
    visionarm::report::ReportLevel report_level =
        visionarm::report::ReportLevel::SUMMARY;
    visionarm::logging::LoggerConfig log_config;
    std::string log_config_path;
    std::string input_dma_heap =
        "/dev/dma_heap/system-uncached-dma32";
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
    visionarm::InferenceThreadTopology topology =
        visionarm::InferenceThreadTopology::FUSED_NPU_POSTPROCESS;
};

[[nodiscard]] std::int64_t EffectiveNetworkRateBps(
    const Options& options) noexcept {
    if (options.network_rate_bps > 0) {
        return options.network_rate_bps;
    }
    const std::int64_t encoded_rate =
        static_cast<std::int64_t>(options.bitrate) + options.audio_bitrate;
    return (encoded_rate * 6LL + 4LL) / 5LL;
}

[[nodiscard]] std::int64_t EffectiveNetworkBurstBits(
    const Options& options) noexcept {
    if (options.network_burst_bits > 0) {
        return options.network_burst_bits;
    }
    const std::int64_t eight_datagrams =
        static_cast<std::int64_t>(options.network_packet_size) * 8LL * 8LL;
    return std::max(EffectiveNetworkRateBps(options) /
                        static_cast<std::int64_t>(100),
                eight_datagrams);
}

[[noreturn]] void Usage(const char* program) {
    std::cerr
        << "Usage: " << program << " \\\n"
        << "  --device /dev/videoX --sensor-subdev /dev/v4l-subdevX \\\n"
        << "  --model model.rknn --output stream.h265 \\\n"
        << "  --width W --height H --fps FPS --bitrate BPS --gop N \\\n"
        << "  [--duration-sec N] [--buffers N] [--video-queue N] \\\n"
        << "  [--audio-device hw:1,0] [--audio-rate 48000] \\\n"
        << "  [--audio-channels 2] [--audio-period-frames 1024] \\\n"
        << "  [--audio-buffer-frames 4096] [--audio-queue 16] [--audio-disable] \\\n"
        << "  [--av-output recording.mp4] [--audio-bitrate 128000] \\\n"
        << "  [--audio-encoded-queue 32] \\\n"
        << "  [--network-url 'udp://PC:5000'] \\\n"
        << "  [--network-queue 256] [--network-io-timeout-ms 1000] \\\n"
        << "  [--network-rate-bps 0] [--network-burst-bits 0] \\\n"
        << "  [--network-packet-size 1316] \\\n"
        << "  [--network-send-buffer-bytes 4194304] \\\n"
        << "  [--telemetry-host PC] [--telemetry-port 5001] \\\n"
        << "  [--telemetry-interval-ms 100] \\\n"
        << "  [--telemetry-send-buffer-bytes 1048576] \\\n"
        << "  [--topology fused|split] [--input-slots N] [--output-slots N] \\\n"
        << "  [--acquire-hits N] [--lost-misses N] \\\n"
        << "  [--max-result-age-ms N] [--latency-samples N] \\\n"
        << "  [--max-rss-growth-kb N] [--vertical-stride N] \\\n"
        << "  [--input-dma-heap PATH] [--confidence F] [--report PATH] \\\n"
        << "  [--report-level summary|performance|diagnostic] \\\n"
        << "  [--log-level LEVEL] [--log-module MODULE=LEVEL] \\\n"
        << "  [--log-config PATH] \\\n"
        << "  [--control-backend mock|uart] [--uart-device /dev/ttyS3] \\\n"
        << "  [--uart-baud 115200] [--uart-ready-timeout-ms 5000]\n";
    throw std::invalid_argument("invalid or incomplete command line");
}

int ParsePositiveInt(const char* text, const char* name) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value <= 0 ||
        value > 2'000'000'000L) {
        throw std::invalid_argument(std::string("invalid ") + name);
    }
    return static_cast<int>(value);
}

int ParseNonnegativeInt(const char* text, const char* name) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value < 0 ||
        value > 2'000'000'000L) {
        throw std::invalid_argument(std::string("invalid ") + name);
    }
    return static_cast<int>(value);
}

int ParseCameraFps(const char* text) {
    const int fps = ParsePositiveInt(text, "fps");
    if (!visionarm::V4L2SensorController::IsAllowedProductFps(
            static_cast<uint32_t>(fps))) {
        throw std::invalid_argument(
            "unsupported fps; allowed values are 30, 60, 90");
    }
    return fps;
}

struct ProductCameraMode {
    int width;
    int height;
    int fps;
};

constexpr std::array<ProductCameraMode, 7> kEnabledCameraModes{{
    {1920, 1080, 30},
    {2560, 1440, 30},
    {3840, 2160, 30},
    {1920, 1080, 60},
    {2560, 1440, 60},
    {3840, 2160, 60},
    // V8.1 P0 high-frame-rate product mode. The sensor uses the
    // 3864x2192 RAW10 90fps profile; RKISP produces 1920x1080 NV12.
    {1920, 1080, 90},
}};

[[nodiscard]] bool IsProductOutputResolution(int width, int height) noexcept {
    return
        (width == 1920 && height == 1080) ||
        (width == 2560 && height == 1440) ||
        (width == 3840 && height == 2160);
}

[[nodiscard]] bool IsEnabledCameraMode(
    int width,
    int height,
    int fps) noexcept {

    return std::any_of(
        kEnabledCameraModes.begin(),
        kEnabledCameraModes.end(),
        [&](const ProductCameraMode& mode) {
            return mode.width == width &&
                   mode.height == height &&
                   mode.fps == fps;
        });
}

void ValidateCameraMode(const Options& options) {
    if (!IsProductOutputResolution(options.width, options.height)) {
        throw std::invalid_argument(
            "unsupported resolution; supported output resolutions are "
            "1920x1080, 2560x1440, 3840x2160");
    }

    if (!IsEnabledCameraMode(options.width, options.height, options.fps)) {
        throw std::invalid_argument(
            "unsupported camera mode; enabled modes are "
            "1920x1080@30/60/90, 2560x1440@30/60, 3840x2160@30/60");
    }
}

Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string key = argv[index];
        auto next = [&]() -> const char* {
            if (++index >= argc) Usage(argv[0]);
            return argv[index];
        };

        if (key == "--device") options.device = next();
        else if (key == "--sensor-subdev") options.sensor_subdev = next();
        else if (key == "--model") options.model = next();
        else if (key == "--output") options.output = next();
        else if (key == "--report") options.report = next();
        else if (key == "--report-level") {
            const std::string value = next();
            if (!visionarm::report::ParseReportLevel(
                    value, &options.report_level)) {
                throw std::invalid_argument(
                    "report level must be summary, performance, or diagnostic");
            }
        }
        else if (key == "--log-level") {
            const std::string value = next();
            if (!visionarm::logging::ParseLogLevel(
                    value, &options.log_config.default_level)) {
                throw std::invalid_argument(
                    "log level must be TRACE, DEBUG, INFO, WARN, ERROR, FATAL, or OFF");
            }
        }
        else if (key == "--log-module") {
            const std::string value = next();
            std::string error;
            if (!visionarm::logging::ApplyModuleLogLevel(
                    value, &options.log_config, &error)) {
                throw std::invalid_argument(error);
            }
        }
        else if (key == "--log-config") options.log_config_path = next();
        else if (key == "--width") options.width = ParsePositiveInt(next(), "width");
        else if (key == "--height") options.height = ParsePositiveInt(next(), "height");
        else if (key == "--fps") options.fps = ParseCameraFps(next());
        else if (key == "--buffers") options.buffers = ParsePositiveInt(next(), "buffers");
        else if (key == "--duration-sec") options.duration_seconds = ParsePositiveInt(next(), "duration");
        else if (key == "--timeout-ms") options.timeout_ms = ParsePositiveInt(next(), "timeout");
        else if (key == "--bitrate") options.bitrate = ParsePositiveInt(next(), "bitrate");
        else if (key == "--gop") options.gop = ParsePositiveInt(next(), "gop");
        else if (key == "--vertical-stride") options.vertical_stride = ParsePositiveInt(next(), "vertical stride");
        else if (key == "--video-queue") options.video_queue = ParsePositiveInt(next(), "video queue");
        else if (key == "--audio-device") options.audio_device = next();
        else if (key == "--audio-rate") options.audio_rate = ParsePositiveInt(next(), "audio rate");
        else if (key == "--audio-channels") options.audio_channels = ParsePositiveInt(next(), "audio channels");
        else if (key == "--audio-period-frames") options.audio_period_frames = ParsePositiveInt(next(), "audio period frames");
        else if (key == "--audio-buffer-frames") options.audio_buffer_frames = ParsePositiveInt(next(), "audio buffer frames");
        else if (key == "--audio-queue") options.audio_queue = ParsePositiveInt(next(), "audio queue");
        else if (key == "--audio-encoded-queue") options.audio_encoded_queue = ParsePositiveInt(next(), "encoded audio queue");
        else if (key == "--audio-bitrate") options.audio_bitrate = ParsePositiveInt(next(), "audio bitrate");
        else if (key == "--av-output") options.av_output = next();
        else if (key == "--network-url") options.network_url = next();
        else if (key == "--network-queue") options.network_queue = ParsePositiveInt(next(), "network queue");
        else if (key == "--network-io-timeout-ms") options.network_io_timeout_ms = ParsePositiveInt(next(), "network I/O timeout");
        else if (key == "--network-rate-bps") options.network_rate_bps = ParseNonnegativeInt(next(), "network rate");
        else if (key == "--network-burst-bits") options.network_burst_bits = ParseNonnegativeInt(next(), "network burst");
        else if (key == "--network-packet-size") options.network_packet_size = ParsePositiveInt(next(), "network packet size");
        else if (key == "--network-send-buffer-bytes") options.network_send_buffer_bytes = ParsePositiveInt(next(), "network send buffer");
        else if (key == "--telemetry-host") options.telemetry_host = next();
        else if (key == "--telemetry-port") options.telemetry_port = ParsePositiveInt(next(), "telemetry port");
        else if (key == "--telemetry-interval-ms") options.telemetry_interval_ms = ParsePositiveInt(next(), "telemetry interval");
        else if (key == "--telemetry-send-buffer-bytes") options.telemetry_send_buffer_bytes = ParsePositiveInt(next(), "telemetry send buffer");
        else if (key == "--audio-disable") options.audio_enabled = false;
        else if (key == "--input-slots") options.input_slots = ParsePositiveInt(next(), "input slots");
        else if (key == "--output-slots") options.output_slots = ParsePositiveInt(next(), "output slots");
        else if (key == "--acquire-hits") options.acquire_hits = ParsePositiveInt(next(), "acquire hits");
        else if (key == "--lost-misses") options.lost_misses = ParsePositiveInt(next(), "lost misses");
        else if (key == "--max-result-age-ms") options.max_result_age_ms = ParsePositiveInt(next(), "max result age");
        else if (key == "--latency-samples") options.latency_samples = ParsePositiveInt(next(), "latency samples");
        else if (key == "--max-rss-growth-kb") options.max_rss_growth_kb = ParseNonnegativeInt(next(), "max RSS growth");
        else if (key == "--input-dma-heap") options.input_dma_heap = next();
        else if (key == "--confidence") options.confidence = std::stof(next());
        else if (key == "--control-backend") {
            const std::string value = next();
            if (value == "mock") {
                options.control_backend = ControlBackend::MOCK;
            } else if (value == "uart") {
                options.control_backend = ControlBackend::UART;
            } else {
                throw std::invalid_argument(
                    "control backend must be mock or uart");
            }
        } else if (key == "--uart-device") {
            options.uart_device = next();
        } else if (key == "--uart-baud") {
            options.uart_baud = ParsePositiveInt(next(), "UART baud");
        } else if (key == "--uart-ready-timeout-ms") {
            options.uart_ready_timeout_ms =
                ParsePositiveInt(next(), "UART ready timeout");
        } else if (key == "--topology") {
            const std::string value = next();
            if (value == "fused") {
                options.topology =
                    visionarm::InferenceThreadTopology::FUSED_NPU_POSTPROCESS;
            } else if (value == "split") {
                options.topology =
                    visionarm::InferenceThreadTopology::SPLIT_NPU_POSTPROCESS;
            } else {
                throw std::invalid_argument("topology must be fused or split");
            }
        } else {
            Usage(argv[0]);
        }
    }

    if (options.device.empty() || options.model.empty() ||
        options.output.empty() || options.width <= 0 ||
        options.height <= 0 || options.fps <= 0 ||
        options.bitrate <= 0 || options.gop <= 0) {
        Usage(argv[0]);
    }
    ValidateCameraMode(options);
    if (options.sensor_subdev.empty()) {
        throw std::invalid_argument("sensor subdev must not be empty");
    }
    if (options.control_backend == ControlBackend::UART &&
        options.uart_device.empty()) {
        throw std::invalid_argument("UART device must not be empty");
    }
    if (options.audio_enabled) {
        if (options.audio_device.rfind("hw:", 0) != 0) {
            throw std::invalid_argument(
                "audio device must be an hw: PCM for V8.3");
        }
        if (options.audio_rate != 48'000) {
            throw std::invalid_argument(
                "V8.3 frozen audio baseline requires 48000 Hz");
        }
        if (options.audio_channels != 2) {
            throw std::invalid_argument(
                "V8.3 frozen audio baseline requires 2 capture channels");
        }
        if (options.audio_buffer_frames < options.audio_period_frames) {
            throw std::invalid_argument(
                "audio buffer frames must be >= period frames");
        }
    }
#if !defined(VISIONARM_HAS_ALSA_AUDIO)
    if (options.audio_enabled) {
        throw std::invalid_argument(
            "this binary was built without VISIONARM_ENABLE_ALSA_AUDIO");
    }
#endif
    if (!options.av_output.empty() && !options.audio_enabled) {
        throw std::invalid_argument(
            "--av-output requires the frozen V8.3 audio path to be enabled");
    }
    if (!options.network_url.empty() && !options.audio_enabled) {
        throw std::invalid_argument(
            "--network-url requires the frozen V8.3 audio path to be enabled");
    }
    if (!options.network_url.empty() &&
        options.network_url.rfind("udp://", 0U) != 0U) {
        throw std::invalid_argument(
            "V8.4 P0 network URL must start with udp://");
    }
    if (options.network_packet_size > 65'507 ||
        options.network_packet_size % 188 != 0) {
        throw std::invalid_argument(
            "network packet size must be a multiple of 188 and <= 65507");
    }
    if (options.telemetry_port > 65'535) {
        throw std::invalid_argument("telemetry port must be <= 65535");
    }
    if (options.telemetry_interval_ms < 20 ||
        options.telemetry_interval_ms > 60'000) {
        throw std::invalid_argument(
            "telemetry interval must be in [20, 60000] ms");
    }
#if !defined(VISIONARM_HAS_AV_MUX)
    if (!options.av_output.empty()) {
        throw std::invalid_argument(
            "this binary was built without VISIONARM_ENABLE_FFMPEG_MP4_MUX");
    }
#endif
#if !defined(VISIONARM_HAS_NETWORK_MUX)
    if (!options.network_url.empty()) {
        throw std::invalid_argument(
            "this binary was built without VISIONARM_ENABLE_FFMPEG_MPEGTS_NETWORK");
    }
#endif
#if !defined(VISIONARM_HAS_UART_CONTROL)
    if (options.control_backend == ControlBackend::UART) {
        throw std::invalid_argument(
            "this binary was built without VISIONARM_ENABLE_UART_CONTROL");
    }
#endif

    const int maximum_video_queue = options.buffers > 4
        ? options.buffers - 4
        : 1;
    if (options.video_queue > maximum_video_queue) {
        throw std::invalid_argument(
            "video queue is too large for the V4L2 pool");
    }
    return options;
}

[[nodiscard]] bool LoadEffectiveLoggerConfig(
    const Options& options,
    visionarm::logging::LoggerConfig* config,
    std::string* error) {
    if (config == nullptr) return false;
    if (options.log_config_path.empty()) {
        *config = options.log_config;
        return true;
    }
    return visionarm::logging::LoadLoggerConfigFile(
        options.log_config_path, options.log_config, config, error);
}

void ReloadLoggerFromSupervisor(const Options& options) noexcept {
    try {
        if (options.log_config_path.empty()) {
            visionarm::logging::Log(
                visionarm::logging::LogLevel::WARN, "runtime",
                "SIGHUP ignored because --log-config was not supplied");
            return;
        }
        visionarm::logging::LoggerConfig config;
        std::string error;
        if (!LoadEffectiveLoggerConfig(options, &config, &error) ||
            !visionarm::logging::ConfigureGlobalLogger(config, &error)) {
            visionarm::logging::Log(
                visionarm::logging::LogLevel::ERROR, "runtime",
                "log config reload failed; keeping previous config: ", error);
            return;
        }
#if defined(VISIONARM_HAS_AUDIO_ENCODE)
        visionarm::ApplyFfmpegLogLevel();
#endif
        visionarm::logging::Log(
            visionarm::logging::LogLevel::INFO, "runtime",
            "log config reloaded from ", options.log_config_path,
            " default_level=",
            visionarm::logging::LogLevelName(config.default_level));
    } catch (const std::exception& error) {
        visionarm::logging::Log(
            visionarm::logging::LogLevel::ERROR, "runtime",
            "log config reload failed; keeping previous config: ",
            error.what());
    } catch (...) {
        visionarm::logging::Log(
            visionarm::logging::LogLevel::ERROR, "runtime",
            "log config reload failed with unknown exception");
    }
}

std::string FindOptionValue(
    int argc, char** argv, std::string_view option) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (argv[index] != nullptr && option == argv[index] &&
            argv[index + 1] != nullptr) {
            return argv[index + 1];
        }
    }
    return {};
}

#if defined(VISIONARM_HAS_UART_CONTROL)
bool WaitForUartReady(
    visionarm::uart::UartLink& link,
    int timeout_ms) noexcept {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);
    while (!g_stop.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
        const visionarm::uart::LinkState state = link.GetLinkState();
        if (state == visionarm::uart::LinkState::READY) {
            return true;
        }
        if (state == visionarm::uart::LinkState::FAILED) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}
#endif

int DeriveVerticalStride(
    const visionarm::CameraFormat& format,
    int override_value) {
    if (override_value > 0) return override_value;
    if (format.bytes_per_line.empty() || format.size_image.empty() ||
        format.bytes_per_line[0] == 0U) {
        throw std::runtime_error("Camera did not report NV12 stride/size_image");
    }
    const uint64_t numerator =
        static_cast<uint64_t>(format.size_image[0]) * 2U;
    const uint64_t denominator =
        static_cast<uint64_t>(format.bytes_per_line[0]) * 3U;
    if (denominator == 0U || numerator % denominator != 0U) {
        throw std::runtime_error(
            "cannot derive MPP vertical stride; pass --vertical-stride");
    }
    return static_cast<int>(numerator / denominator);
}

const rknn_tensor_attr& FindOutputAttribute(
    const visionarm::RknnModelInfo& model_info,
    uint32_t tensor_index) {
    const auto iterator = std::find_if(
        model_info.output_attributes.begin(),
        model_info.output_attributes.end(),
        [tensor_index](const rknn_tensor_attr& attr) {
            return attr.index == tensor_index;
        });
    if (iterator == model_info.output_attributes.end()) {
        throw std::runtime_error(
            "RKNN model is missing output tensor index " +
            std::to_string(tensor_index));
    }
    return *iterator;
}

void ValidateOutputAttribute(
    const visionarm::RknnModelInfo& model_info,
    uint32_t tensor_index,
    uint32_t channels,
    uint32_t height,
    uint32_t width) {
    const rknn_tensor_attr& attr =
        FindOutputAttribute(model_info, tensor_index);
    if (attr.n_dims != 4U || attr.dims[0] != 1U ||
        attr.dims[1] != channels || attr.dims[2] != height ||
        attr.dims[3] != width || attr.fmt != RKNN_TENSOR_NCHW ||
        attr.type != RKNN_TENSOR_INT8 ||
        attr.qnt_type != RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC) {
        throw std::runtime_error(
            "RKNN output tensor " + std::to_string(tensor_index) +
            " does not match the optimized YOLOv8 contract");
    }
}

void ValidateYoloV8OutputContract(
    const visionarm::RknnModelInfo& model_info) {
    if (model_info.output_attributes.size() != 9U) {
        throw std::runtime_error(
            "the optimized YOLOv8 model must expose exactly 9 outputs");
    }
    constexpr std::array<uint32_t, 3> strides{8U, 16U, 32U};
    constexpr std::array<uint32_t, 3> box_indices{0U, 3U, 6U};
    constexpr std::array<uint32_t, 3> class_indices{1U, 4U, 7U};
    constexpr std::array<uint32_t, 3> sum_indices{2U, 5U, 8U};
    for (std::size_t branch = 0U; branch < strides.size(); ++branch) {
        const uint32_t height =
            static_cast<uint32_t>(kModelHeight) / strides[branch];
        const uint32_t width =
            static_cast<uint32_t>(kModelWidth) / strides[branch];
        ValidateOutputAttribute(
            model_info, box_indices[branch], 64U, height, width);
        ValidateOutputAttribute(
            model_info, class_indices[branch],
            static_cast<uint32_t>(kClassCount), height, width);
        ValidateOutputAttribute(
            model_info, sum_indices[branch], 1U, height, width);
    }
}

int64_t ReadVmRssKb() noexcept {
    std::ifstream stream("/proc/self/status");
    std::string key;
    while (stream >> key) {
        if (key == "VmRSS:") {
            int64_t value = 0;
            std::string unit;
            stream >> value >> unit;
            return value;
        }
        stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return -1;
}

struct RssSamples {
    int64_t first_kb = -1;
    int64_t last_kb = -1;
    int64_t minimum_kb = -1;
    int64_t maximum_kb = -1;
    uint64_t samples = 0U;

    void Add(int64_t value) noexcept {
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

    [[nodiscard]] int64_t GrowthKb() const noexcept {
        return first_kb >= 0 && last_kb >= 0 ? last_kb - first_kb : 0;
    }
};


#if defined(VISIONARM_HAS_ALSA_AUDIO)
struct AudioTimelineProbeStats {
    std::uint64_t chunks = 0U;
    std::uint64_t frames = 0U;
    std::uint64_t bytes = 0U;
    std::uint64_t reanchors = 0U;
    std::uint64_t discontinuities = 0U;
    std::uint64_t pts_regressions = 0U;
    std::uint64_t continuous_pts_mismatches = 0U;

    std::int64_t first_pts_ns = -1;
    std::int64_t last_pts_ns = -1;
    std::int64_t last_end_pts_ns = -1;
    std::int64_t maximum_forward_gap_ns = 0;
    std::int64_t maximum_abs_timing_error_ns = 0;

    void Consume(const visionarm::TimedAudioChunk& chunk) noexcept {
        ++chunks;
        frames += chunk.raw.timing.frame_count;
        bytes += chunk.raw.pcm.size();
        if (chunk.media.reanchored) {
            ++reanchors;
        }
        if (chunk.raw.timing.discontinuity_before) {
            ++discontinuities;
        }

        if (first_pts_ns < 0) {
            first_pts_ns = chunk.media.pts_ns;
        }

        if (last_end_pts_ns >= 0) {
            if (chunk.media.pts_ns < last_end_pts_ns) {
                ++pts_regressions;
            } else {
                const std::int64_t gap_ns =
                    chunk.media.pts_ns - last_end_pts_ns;
                maximum_forward_gap_ns =
                    std::max(maximum_forward_gap_ns, gap_ns);
                if (!chunk.media.reanchored && gap_ns != 0) {
                    ++continuous_pts_mismatches;
                }
            }
        }

        const std::int64_t abs_error_ns =
            chunk.media.timing_error_ns < 0
                ? -chunk.media.timing_error_ns
                : chunk.media.timing_error_ns;
        maximum_abs_timing_error_ns =
            std::max(maximum_abs_timing_error_ns, abs_error_ns);
        last_pts_ns = chunk.media.pts_ns;
        last_end_pts_ns =
            chunk.media.pts_ns + chunk.media.duration_ns;
    }
};

void DrainAudioQueue(
    visionarm::BoundedQueue<visionarm::TimedAudioChunk>* queue,
    AudioTimelineProbeStats* stats) {
    if (queue == nullptr || stats == nullptr) {
        return;
    }
    visionarm::TimedAudioChunk chunk;
    while (queue->TryPop(&chunk)) {
        stats->Consume(chunk);
    }
}
#endif

void WriteLatency(
    std::ostream& stream,
    const char* name,
    const visionarm::LatencyDistributionSnapshot& value) {
    stream << name << ".total_samples=" << value.total_samples << '\n'
           << name << ".retained_samples=" << value.retained_samples << '\n'
           << name << ".truncated=" << (value.truncated ? 1 : 0) << '\n'
           << name << ".mean_ms=" << value.mean_ms << '\n'
           << name << ".p50_ms=" << value.p50_ms << '\n'
           << name << ".p95_ms=" << value.p95_ms << '\n'
           << name << ".p99_ms=" << value.p99_ms << '\n'
           << name << ".maximum_ms=" << value.maximum_ms << '\n';
}

void WriteQueue(
    std::ostream& stream,
    const char* name,
    const visionarm::QueueStatsSnapshot& value) {
    stream << name << ".capacity=" << value.capacity << '\n'
           << name << ".high_watermark=" << value.high_watermark << '\n'
           << name << ".current_size=" << value.current_size << '\n'
           << name << ".pushed=" << value.pushed << '\n'
           << name << ".popped=" << value.popped << '\n'
           << name << ".replaced_oldest=" << value.replaced_oldest << '\n'
           << name << ".stopped=" << (value.stopped ? 1 : 0) << '\n';
}

bool QueueBounded(const visionarm::QueueStatsSnapshot& value) noexcept {
    return value.capacity > 0U &&
        value.high_watermark <= value.capacity &&
        value.current_size <= value.capacity;
}

bool QueueDrained(
    const visionarm::QueueStatsSnapshot& value) noexcept {
    return QueueBounded(value) && value.stopped && value.current_size == 0U &&
        value.pushed == value.popped + value.replaced_oldest &&
        value.replaced_oldest == 0U;
}

double RatePerSecond(std::uint64_t count, double seconds) noexcept {
    return seconds > 0.0 ? static_cast<double>(count) / seconds : 0.0;
}

bool LatencyComplete(
    const visionarm::LatencyDistributionSnapshot& value,
    std::uint64_t expected_samples) noexcept {
    return expected_samples > 0U &&
        value.total_samples == expected_samples &&
        value.retained_samples == value.total_samples &&
        !value.truncated;
}

#if defined(VISIONARM_HAS_AV_MUX) || defined(VISIONARM_HAS_NETWORK_MUX)
class FanoutEncodedVideoSink final : public visionarm::IEncodedPacketSink {
public:
    explicit FanoutEncodedVideoSink(
        std::vector<visionarm::IEncodedPacketSink*> sinks)
        : sinks_(std::move(sinks)) {
        if (sinks_.size() < 2U ||
            std::any_of(sinks_.begin(), sinks_.end(),
                        [](const auto* sink) { return sink == nullptr; })) {
            throw std::invalid_argument(
                "FanoutEncodedVideoSink requires at least two valid sinks");
        }
    }

    [[nodiscard]] bool Write(
        const visionarm::EncodedPacket& packet) noexcept override {
        bool ok = true;
        for (visionarm::IEncodedPacketSink* sink : sinks_) {
            ok = sink->Write(packet) && ok;
        }
        return ok;
    }

    void Flush() noexcept override {
        for (visionarm::IEncodedPacketSink* sink : sinks_) {
            sink->Flush();
        }
    }

private:
    std::vector<visionarm::IEncodedPacketSink*> sinks_;
};

class FanoutEncodedAudioSink final :
    public visionarm::IEncodedAudioPacketSink {
public:
    explicit FanoutEncodedAudioSink(
        std::vector<visionarm::IEncodedAudioPacketSink*> sinks)
        : sinks_(std::move(sinks)) {
        if (sinks_.size() < 2U ||
            std::any_of(sinks_.begin(), sinks_.end(),
                        [](const auto* sink) { return sink == nullptr; })) {
            throw std::invalid_argument(
                "FanoutEncodedAudioSink requires at least two valid sinks");
        }
    }

    [[nodiscard]] bool WriteAudio(
        const visionarm::EncodedAudioPacket& packet) noexcept override {
        bool ok = true;
        for (visionarm::IEncodedAudioPacketSink* sink : sinks_) {
            ok = sink->WriteAudio(packet) && ok;
        }
        return ok;
    }

    void FlushAudio() noexcept override {
        for (visionarm::IEncodedAudioPacketSink* sink : sinks_) {
            sink->FlushAudio();
        }
    }

private:
    std::vector<visionarm::IEncodedAudioPacketSink*> sinks_;
};

std::vector<std::uint8_t> ConcatenateCodecConfig(
    const std::vector<visionarm::EncodedPacket>& packets) {
    std::vector<std::uint8_t> result;
    for (const auto& packet : packets) {
        if (!packet.codec_config || packet.bytes.empty()) {
            continue;
        }
        result.insert(result.end(), packet.bytes.begin(), packet.bytes.end());
    }
    if (result.empty()) {
        throw std::runtime_error("MPP did not provide HEVC VPS/SPS/PPS codec config");
    }
    return result;
}
#endif

}  // namespace

int main(int argc, char** argv) {
    Options options;
    bool options_parsed = false;
    try {
        options = ParseOptions(argc, argv);
        options_parsed = true;
        visionarm::logging::LoggerConfig logger_config;
        std::string logger_error;
        if (!LoadEffectiveLoggerConfig(
                options, &logger_config, &logger_error) ||
            !visionarm::logging::ConfigureGlobalLogger(
                logger_config, &logger_error)) {
            throw std::runtime_error(
                "failed to configure logger: " + logger_error);
        }
        std::signal(SIGINT, SignalHandler);
        std::signal(SIGTERM, SignalHandler);
#if defined(SIGHUP)
        std::signal(SIGHUP, SignalHandler);
#endif
        visionarm::logging::Log(
            visionarm::logging::LogLevel::INFO, "runtime",
            "startup duration_sec=", options.duration_seconds,
            " report_level=",
            visionarm::report::ReportLevelName(options.report_level),
            " log_level=",
            visionarm::logging::LogLevelName(logger_config.default_level));

        visionarm::V4L2CameraConfig camera_config;
        camera_config.device = options.device;
        camera_config.width = static_cast<uint32_t>(options.width);
        camera_config.height = static_cast<uint32_t>(options.height);
        camera_config.pixel_format = V4L2_PIX_FMT_NV12;
        camera_config.buffer_count = static_cast<uint32_t>(options.buffers);
        camera_config.timeout_ms = options.timeout_ms;
        camera_config.export_dmabuf = true;
        camera_config.require_dmabuf_export = true;

        visionarm::V4L2Camera camera(camera_config);
        camera.Open();

        visionarm::V4L2SensorControllerConfig sensor_config;
        sensor_config.device = options.sensor_subdev;
        sensor_config.pad = 0U;
        const visionarm::V4L2SensorController sensor_controller(sensor_config);
        const visionarm::SensorFrameRate configured_sensor_fps =
            sensor_controller.ConfigureFrameRate(
                static_cast<uint32_t>(options.fps));

        const visionarm::CameraFormat camera_format = camera.format();
        if (camera_format.pixel_format != V4L2_PIX_FMT_NV12 ||
            camera_format.plane_count != 1U ||
            camera_format.bytes_per_line.empty()) {
            throw std::runtime_error(
                "R7/R8 requires frozen single-plane linear NV12");
        }

        visionarm::logging::Log(
            visionarm::logging::LogLevel::INFO, "camera",
            "requested=", options.width, 'x', options.height, '@', options.fps,
            " sensor_subdev=", options.sensor_subdev,
            " configured_sensor_fps=", configured_sensor_fps.numerator, '/',
            configured_sensor_fps.denominator, " isp_output=",
            camera_format.width, 'x', camera_format.height, ' ',
            visionarm::FourccToString(camera_format.pixel_format));

        visionarm::RknnEngineConfig engine_config;
        engine_config.model_path = options.model;
        engine_config.input_slot_count =
            static_cast<std::size_t>(options.input_slots);
        engine_config.output_slot_count =
            static_cast<std::size_t>(options.output_slots);
        engine_config.io_mode = visionarm::RknnIoMode::BOUND_HOST_IO;
        engine_config.input_dma_heap_path = options.input_dma_heap;

        visionarm::RknnEngine engine;
        engine.Initialize(engine_config);
        if (engine.input_shape().width !=
                static_cast<uint32_t>(kModelWidth) ||
            engine.input_shape().height !=
                static_cast<uint32_t>(kModelHeight)) {
            throw std::runtime_error(
                "R7/R8 requires logical RKNN input 960x544");
        }
        ValidateYoloV8OutputContract(engine.model_info());

        visionarm::RgaLetterboxConfig rga_config;
        rga_config.model_width = kModelWidth;
        rga_config.model_height = kModelHeight;
        rga_config.padding_value = 114U;
        rga_config.resize_policy.stretch_matching_source_aspect_ratio = false;
        rga_config.max_source_buffers = camera.buffer_count();
        rga_config.max_destination_slots = engine.input_slot_count();
        visionarm::RgaLetterboxPreprocessor preprocessor(rga_config);

        visionarm::YoloV8Top1PostprocessConfig postprocess_config;
        postprocess_config.decoder.model_width = kModelWidth;
        postprocess_config.decoder.model_height = kModelHeight;
        postprocess_config.decoder.class_count = kClassCount;
        postprocess_config.decoder.target_class_id = kTargetClassId;
        postprocess_config.decoder.dfl_bins = 16;
        postprocess_config.decoder.confidence_threshold = options.confidence;
        visionarm::YoloV8Top1Postprocessor postprocessor(postprocess_config);

        visionarm::CaptureBufferBrokerConfig broker_config;
        broker_config.max_buffer_count = camera.buffer_count();
        broker_config.require_dmabuf = true;
        broker_config.requeue_ready_notifier = [&camera] { camera.Wake(); };
        visionarm::CaptureBufferBroker broker(broker_config);

        visionarm::LatestResultStore latest_perception;
        visionarm::MockControlSink mock_control_sink;
        visionarm::IControlSink* selected_control_sink = &mock_control_sink;
        std::unique_ptr<visionarm::UdpTelemetrySink> telemetry_sink;

#if defined(VISIONARM_HAS_UART_CONTROL)
        std::unique_ptr<visionarm::uart::UartLink> uart_link;
        std::unique_ptr<visionarm::UartControlSink> uart_control_sink;
        if (options.control_backend == ControlBackend::UART) {
            visionarm::uart::UartLinkConfig uart_config;
            uart_config.device_path = options.uart_device;
            uart_config.baud_rate = static_cast<uint32_t>(options.uart_baud);

            uart_link =
                std::make_unique<visionarm::uart::UartLink>(uart_config);
            std::string uart_error;
            if (!uart_link->Start(&uart_error)) {
                throw std::runtime_error(
                    "UART Start failed: " + uart_error);
            }
            if (!WaitForUartReady(*uart_link, options.uart_ready_timeout_ms)) {
                const visionarm::uart::UartModuleSnapshot failed =
                    uart_link->GetSnapshot();
                uart_link->Stop();
                throw std::runtime_error(
                    std::string("UART did not reach READY; state=") +
                    visionarm::uart::LinkStateName(failed.state) +
                    " last_error=" + failed.last_error);
            }

            uart_control_sink =
                std::make_unique<visionarm::UartControlSink>(*uart_link);
            selected_control_sink = uart_control_sink.get();
            visionarm::logging::Log(
                visionarm::logging::LogLevel::INFO, "uart",
                "link READY device=", options.uart_device,
                " baud=", options.uart_baud);
        }
#endif

        const bool telemetry_enabled = !options.telemetry_host.empty();
        bool telemetry_start_ok = true;
        std::string telemetry_start_error;
        if (telemetry_enabled) {
            visionarm::UdpTelemetrySinkConfig telemetry_config;
            telemetry_config.host = options.telemetry_host;
            telemetry_config.port =
                static_cast<std::uint16_t>(options.telemetry_port);
            telemetry_config.interval_ms =
                static_cast<std::uint32_t>(options.telemetry_interval_ms);
            telemetry_config.send_buffer_bytes =
                options.telemetry_send_buffer_bytes;
            telemetry_config.control_backend =
                ControlBackendName(options.control_backend);
            telemetry_sink =
                std::make_unique<visionarm::UdpTelemetrySink>(telemetry_config);
            if (!telemetry_sink->Start(&telemetry_start_error)) {
                telemetry_start_ok = false;
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::WARN, "telemetry",
                    "disabled after Start failure: ", telemetry_start_error);
            } else {
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::INFO, "telemetry",
                    "enabled destination=", options.telemetry_host, ':',
                    options.telemetry_port, " interval_ms=",
                    options.telemetry_interval_ms, " send_buffer_bytes=",
                    options.telemetry_send_buffer_bytes);
            }
        }

        visionarm::TargetStateMachineConfig state_config;
        state_config.acquire_hits =
            static_cast<uint32_t>(options.acquire_hits);
        state_config.lost_misses =
            static_cast<uint32_t>(options.lost_misses);
        state_config.max_result_age_ns =
            static_cast<int64_t>(options.max_result_age_ms) * 1'000'000LL;
        visionarm::TargetStateMachine state_machine(
            state_config, selected_control_sink, &latest_perception);

        // V8.3 common CLOCK_MONOTONIC media epoch. Both MPP video PTS and
        // audio sample-clock PTS are expressed relative to this value.
        visionarm::MediaClock media_clock(visionarm::MonotonicNowNs());

        visionarm::MppH265EncoderConfig encoder_config;
        encoder_config.width = static_cast<int>(camera_format.width);
        encoder_config.height = static_cast<int>(camera_format.height);
        encoder_config.horizontal_stride =
            static_cast<int>(camera_format.bytes_per_line[0]);
        encoder_config.vertical_stride =
            DeriveVerticalStride(camera_format, options.vertical_stride);
        if (configured_sensor_fps.numerator >
                static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
            configured_sensor_fps.denominator >
                static_cast<uint32_t>(std::numeric_limits<int>::max())) {
            throw std::runtime_error("configured sensor fps exceeds MPP range");
        }
        encoder_config.fps_numerator =
            static_cast<int>(configured_sensor_fps.numerator);
        encoder_config.fps_denominator =
            static_cast<int>(configured_sensor_fps.denominator);
        encoder_config.media_epoch_monotonic_ns =
            media_clock.epoch_monotonic_ns();
        encoder_config.bitrate_bps = options.bitrate;
        encoder_config.gop_length = options.gop;
        encoder_config.max_source_buffers = camera.buffer_count();

        visionarm::MppH265Encoder encoder;
        encoder.Initialize(encoder_config);
        visionarm::logging::Log(
            visionarm::logging::LogLevel::DEBUG, "runtime",
            "startup stage=mpp_ready");
        visionarm::H265FileSink file_sink(options.output);
        if (!file_sink.opened()) {
            throw std::runtime_error("failed to open output H.265 file");
        }
        visionarm::logging::Log(
            visionarm::logging::LogLevel::DEBUG, "runtime",
            "startup stage=h265_sink_ready");
        visionarm::IEncodedPacketSink* selected_video_sink = &file_sink;

#if defined(VISIONARM_HAS_ALSA_AUDIO)
        visionarm::BoundedQueue<visionarm::TimedAudioChunk> audio_pcm_queue(
            static_cast<std::size_t>(options.audio_queue));
        AudioTimelineProbeStats audio_timeline_stats;
        std::unique_ptr<visionarm::AudioCaptureWorker> audio_worker;
#endif

#if defined(VISIONARM_HAS_AUDIO_ENCODE)
#if defined(VISIONARM_HAS_AV_MUX)
        const bool av_mux_enabled = !options.av_output.empty();
#else
        const bool av_mux_enabled = false;
#endif
#if defined(VISIONARM_HAS_NETWORK_MUX)
        const bool network_mux_enabled = !options.network_url.empty();
#else
        const bool network_mux_enabled = false;
#endif
        const bool encoded_audio_enabled =
            av_mux_enabled || network_mux_enabled;
        std::unique_ptr<visionarm::BoundedQueue<visionarm::EncodedAudioPacket>>
            audio_encoded_queue;
        std::unique_ptr<visionarm::AudioEncodeWorker> audio_encode_worker;
        std::unique_ptr<visionarm::EncodedAudioSinkWorker> audio_sink_worker;
        std::unique_ptr<FanoutEncodedAudioSink> fanout_audio_sink;
        std::unique_ptr<FanoutEncodedVideoSink> fanout_video_sink;
#if defined(VISIONARM_HAS_AV_MUX)
        std::unique_ptr<visionarm::FfmpegMp4Muxer> av_muxer;
#endif
#if defined(VISIONARM_HAS_NETWORK_MUX)
        std::unique_ptr<visionarm::FfmpegMpegTsUdpSink> network_muxer;
#endif

        if (encoded_audio_enabled) {
            visionarm::logging::Log(
                visionarm::logging::LogLevel::DEBUG, "runtime",
                "startup stage=encoded_audio_path_begin");
            audio_encoded_queue = std::make_unique<
                visionarm::BoundedQueue<visionarm::EncodedAudioPacket>>(
                    static_cast<std::size_t>(options.audio_encoded_queue));

            visionarm::FfmpegAacEncoderConfig audio_encoder_config;
            audio_encoder_config.input_format.sample_rate_hz =
                static_cast<std::uint32_t>(options.audio_rate);
            audio_encoder_config.input_format.channels =
                static_cast<std::uint16_t>(options.audio_channels);
            audio_encoder_config.input_format.sample_format =
                visionarm::AudioSampleFormat::kS16LE;
            audio_encoder_config.bit_rate_bps = options.audio_bitrate;

            visionarm::logging::Log(
                visionarm::logging::LogLevel::DEBUG, "runtime",
                "startup stage=audio_encode_worker_construct");
            audio_encode_worker = std::make_unique<visionarm::AudioEncodeWorker>(
                audio_encoder_config,
                &audio_pcm_queue,
                audio_encoded_queue.get(),
                [&audio_timeline_stats](const visionarm::TimedAudioChunk& chunk) {
                    audio_timeline_stats.Consume(chunk);
                });
            std::string audio_encode_error;
            visionarm::logging::Log(
                visionarm::logging::LogLevel::DEBUG, "runtime",
                "startup stage=audio_encode_worker_start");
            if (!audio_encode_worker->Start(&audio_encode_error)) {
                throw std::runtime_error(
                    "audio encoder Start failed: " + audio_encode_error);
            }

            visionarm::logging::Log(
                visionarm::logging::LogLevel::DEBUG, "runtime",
                "startup stage=audio_encode_worker_ready");
            const std::vector<std::uint8_t> hevc_codec_config =
                ConcatenateCodecConfig(encoder.CodecConfigPackets());

            std::vector<visionarm::IEncodedAudioPacketSink*> audio_sinks;
            std::vector<visionarm::IEncodedPacketSink*> video_sinks{&file_sink};
#if defined(VISIONARM_HAS_AV_MUX)
            if (av_mux_enabled) {
                visionarm::FfmpegMp4MuxerConfig mux_config;
                mux_config.path = options.av_output;
                mux_config.video_width =
                    static_cast<std::int32_t>(camera_format.width);
                mux_config.video_height =
                    static_cast<std::int32_t>(camera_format.height);
                mux_config.video_bit_rate_bps = options.bitrate;
                mux_config.video_fps_numerator =
                    static_cast<std::int32_t>(configured_sensor_fps.numerator);
                mux_config.video_fps_denominator =
                    static_cast<std::int32_t>(configured_sensor_fps.denominator);
                mux_config.hevc_annexb_codec_config = hevc_codec_config;
                mux_config.audio = audio_encode_worker->stream_info();
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::DEBUG, "runtime",
                    "startup stage=mux_config_ready hevc_extradata_bytes=",
                    mux_config.hevc_annexb_codec_config.size(),
                    " aac_extradata_bytes=",
                    mux_config.audio.codec_config.size());

                av_muxer = std::make_unique<visionarm::FfmpegMp4Muxer>();
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::DEBUG, "runtime",
                    "startup stage=mp4_mux_initialize");
                av_muxer->Initialize(mux_config);
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::DEBUG, "runtime",
                    "startup stage=mp4_mux_ready");
                audio_sinks.push_back(av_muxer.get());
                video_sinks.push_back(av_muxer.get());
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::INFO, "media.mp4",
                    "enabled output=", options.av_output,
                    " audio_bitrate=", options.audio_bitrate,
                    " encoded_audio_queue=", options.audio_encoded_queue);
            }
#endif

#if defined(VISIONARM_HAS_NETWORK_MUX)
            if (network_mux_enabled) {
                visionarm::FfmpegMpegTsUdpSinkConfig network_config;
                network_config.url = options.network_url;
                network_config.video_width =
                    static_cast<std::int32_t>(camera_format.width);
                network_config.video_height =
                    static_cast<std::int32_t>(camera_format.height);
                network_config.video_bit_rate_bps = options.bitrate;
                network_config.video_fps_numerator =
                    static_cast<std::int32_t>(configured_sensor_fps.numerator);
                network_config.video_fps_denominator =
                    static_cast<std::int32_t>(configured_sensor_fps.denominator);
                network_config.hevc_annexb_codec_config = hevc_codec_config;
                network_config.audio = audio_encode_worker->stream_info();
                network_config.packet_queue_capacity =
                    static_cast<std::size_t>(options.network_queue);
                network_config.io_timeout_us =
                    static_cast<std::int64_t>(options.network_io_timeout_ms) *
                    1'000LL;
                network_config.udp_packet_size = options.network_packet_size;
                network_config.udp_send_buffer_bytes =
                    options.network_send_buffer_bytes;
                network_config.udp_bit_rate_bps =
                    EffectiveNetworkRateBps(options);
                network_config.udp_burst_bits =
                    EffectiveNetworkBurstBits(options);

                network_muxer =
                    std::make_unique<visionarm::FfmpegMpegTsUdpSink>();
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::DEBUG, "runtime",
                    "startup stage=mpegts_udp_initialize");
                network_muxer->Initialize(network_config);
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::DEBUG, "runtime",
                    "startup stage=mpegts_udp_ready");
                audio_sinks.push_back(network_muxer.get());
                video_sinks.push_back(network_muxer.get());
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::INFO, "media.network",
                    "enabled url=", options.network_url,
                    " packet_queue=", options.network_queue,
                    " io_timeout_ms=", options.network_io_timeout_ms,
                    " rate_bps=", network_config.udp_bit_rate_bps,
                    " burst_bits=", network_config.udp_burst_bits,
                    " packet_size=", network_config.udp_packet_size,
                    " send_buffer_bytes=",
                    network_config.udp_send_buffer_bytes);
            }
#endif

            visionarm::IEncodedAudioPacketSink* selected_audio_sink = nullptr;
            if (audio_sinks.size() == 1U) {
                selected_audio_sink = audio_sinks.front();
            } else {
                fanout_audio_sink =
                    std::make_unique<FanoutEncodedAudioSink>(audio_sinks);
                selected_audio_sink = fanout_audio_sink.get();
            }
            audio_sink_worker =
                std::make_unique<visionarm::EncodedAudioSinkWorker>(
                    audio_encoded_queue.get(), selected_audio_sink);
            std::string audio_sink_error;
            if (!audio_sink_worker->Start(&audio_sink_error)) {
                throw std::runtime_error(
                    "encoded audio sink Start failed: " + audio_sink_error);
            }

            fanout_video_sink =
                std::make_unique<FanoutEncodedVideoSink>(video_sinks);
            selected_video_sink = fanout_video_sink.get();
        }
#endif

        visionarm::InferencePipelineConfig pipeline_config;
        pipeline_config.enable_video = true;
        pipeline_config.topology = options.topology;
        pipeline_config.captured_frame_queue_capacity = 1U;
        pipeline_config.prepared_frame_queue_capacity =
            static_cast<std::size_t>(options.input_slots);
        pipeline_config.completed_frame_queue_capacity =
            static_cast<std::size_t>(options.output_slots);
        pipeline_config.video_frame_queue_capacity =
            static_cast<std::size_t>(options.video_queue);
        pipeline_config.encoded_packet_queue_capacity = 16U;
        pipeline_config.latency_sample_capacity =
            static_cast<std::size_t>(options.latency_samples);

        visionarm::InferencePipeline pipeline(
            pipeline_config,
            &camera,
            &broker,
            &preprocessor,
            &engine,
            &postprocessor,
            &state_machine,
            &encoder,
            selected_video_sink);

#if defined(VISIONARM_HAS_ALSA_AUDIO)
        if (options.audio_enabled) {
            visionarm::AlsaCaptureConfig audio_config;
            audio_config.device = options.audio_device;
            audio_config.sample_rate_hz =
                static_cast<std::uint32_t>(options.audio_rate);
            audio_config.channels =
                static_cast<std::uint16_t>(options.audio_channels);
            audio_config.sample_format =
                visionarm::AudioSampleFormat::kS16LE;
            audio_config.period_frames =
                static_cast<std::uint32_t>(options.audio_period_frames);
            audio_config.buffer_frames =
                static_cast<std::uint32_t>(options.audio_buffer_frames);
            audio_config.require_exact_hw_params = true;

            audio_worker = std::make_unique<visionarm::AudioCaptureWorker>(
                audio_config, &media_clock, &audio_pcm_queue);
            std::string audio_error;
            if (!audio_worker->Start(&audio_error)) {
                throw std::runtime_error(
                    "audio capture Start failed: " + audio_error);
            }
            const auto audio_start = audio_worker->Snapshot();
            visionarm::logging::Log(
                visionarm::logging::LogLevel::INFO, "audio",
                "device=", audio_start.capture_info.device,
                " rate=", audio_start.capture_info.format.sample_rate_hz,
                " channels=", audio_start.capture_info.format.channels,
                " period_frames=", audio_start.capture_info.period_frames,
                " buffer_frames=", audio_start.capture_info.buffer_frames,
                " timestamp=", audio_start.capture_info.timestamp_type,
                " queue_capacity=", options.audio_queue,
                " media_epoch_ns=", media_clock.epoch_monotonic_ns());
        }
#endif

        if (!pipeline.Start()) {
            throw std::runtime_error("pipeline Start failed");
        }

        const std::int64_t telemetry_rate_epoch_ns =
            visionarm::MonotonicNowNs();
        std::uint64_t last_telemetry_capture_session_id = 0U;
        std::uint64_t last_telemetry_frame_id = 0U;
        bool have_telemetry_control = false;
        auto update_telemetry = [&]() noexcept {
            if (!telemetry_enabled) return true;
            if (!telemetry_start_ok) return false;
            const auto latest = latest_perception.GetCopy();
            if (latest.has_value() && latest->result.target.has_value() &&
                (!have_telemetry_control ||
                 latest->identity.capture_session_id !=
                     last_telemetry_capture_session_id ||
                 latest->identity.frame_id != last_telemetry_frame_id)) {
                visionarm::ControlResult control;
                control.identity = latest->identity;
                control.state = latest->result.target->state;
                control.valid = latest->result.error.valid;
                control.observation = *latest->result.target;
                control.error = latest->result.error;
                control.capture_timestamp_ns =
                    latest->identity.capture_timestamp_ns;
                control.generated_timestamp_ns =
                    latest->generated_timestamp_ns;
                control.age_ns = latest->result_age_ns;
                const auto state = state_machine.Snapshot();
                control.consecutive_hits = state.consecutive_hits;
                control.consecutive_misses = state.consecutive_misses;
                if (!telemetry_sink->UpdateControl(control)) return false;
                last_telemetry_capture_session_id =
                    latest->identity.capture_session_id;
                last_telemetry_frame_id = latest->identity.frame_id;
                have_telemetry_control = true;
            }
            const visionarm::PipelineRuntimeCounters counters =
                pipeline.runtime_counters();
            visionarm::TelemetryRuntimeStatus status;
            status.sample_monotonic_ns = visionarm::MonotonicNowNs();
            status.media_epoch_monotonic_ns =
                media_clock.epoch_monotonic_ns();
            status.pipeline_running = counters.running;
            status.pipeline_fatal_error = counters.fatal_error;
            status.captured_frames = counters.captured_frames;
            status.inference_results = counters.inference_successes;
            status.video_frames_encoded = counters.video_frames_encoded;
            const std::int64_t elapsed_ns =
                status.sample_monotonic_ns - telemetry_rate_epoch_ns;
            if (elapsed_ns > 0) {
                const double seconds =
                    static_cast<double>(elapsed_ns) / 1'000'000'000.0;
                status.camera_fps =
                    static_cast<double>(counters.captured_frames) / seconds;
                status.inference_fps =
                    static_cast<double>(counters.inference_successes) / seconds;
            }
#if defined(VISIONARM_HAS_AV_MUX)
            if (av_mux_enabled) {
                const auto recording = av_muxer->Snapshot();
                status.recording_state = recording.fatal_error
                    ? visionarm::TelemetryOutputState::FATAL
                    : (recording.finalized
                        ? visionarm::TelemetryOutputState::FINALIZED
                        : (recording.header_written
                            ? visionarm::TelemetryOutputState::RUNNING
                            : visionarm::TelemetryOutputState::STARTING));
                status.recording_video_samples =
                    recording.video_samples_written;
                status.recording_audio_packets =
                    recording.audio_packets_written;
            }
#endif
#if defined(VISIONARM_HAS_NETWORK_MUX)
            if (network_mux_enabled) {
                const auto network = network_muxer->Snapshot();
                status.network_state = network.fatal_error
                    ? visionarm::TelemetryOutputState::FATAL
                    : (network.finalized
                        ? visionarm::TelemetryOutputState::FINALIZED
                        : (network.running
                            ? visionarm::TelemetryOutputState::RUNNING
                            : visionarm::TelemetryOutputState::STARTING));
                status.network_video_access_units =
                    network.video_access_units_written;
                status.network_audio_packets = network.audio_packets_written;
            }
#endif
            return telemetry_sink->UpdateRuntime(status);
        };
        bool telemetry_runtime_fault = !telemetry_start_ok;
        if (!telemetry_runtime_fault && !update_telemetry()) {
            telemetry_runtime_fault = true;
            visionarm::logging::Log(
                visionarm::logging::LogLevel::WARN, "telemetry",
                "initial update failed; pipeline remains active");
        }

        RssSamples rss;
        rss.Add(ReadVmRssKb());
        const auto run_start = std::chrono::steady_clock::now();
        const auto deadline = run_start +
            std::chrono::seconds(options.duration_seconds);
        auto next_rss_sample = std::chrono::steady_clock::now() +
            std::chrono::seconds(1);
        bool auxiliary_media_runtime_fault = false;
        while (!g_stop.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < deadline &&
               pipeline.running()) {
            if (g_reload_log_config.exchange(
                    false, std::memory_order_acq_rel)) {
                ReloadLoggerFromSupervisor(options);
            }
#if defined(VISIONARM_HAS_ALSA_AUDIO)
            if (options.audio_enabled) {
#if defined(VISIONARM_HAS_AUDIO_ENCODE)
                if (!encoded_audio_enabled) {
                    DrainAudioQueue(&audio_pcm_queue, &audio_timeline_stats);
                }
                bool encoded_path_fatal = false;
                if (encoded_audio_enabled) {
                    encoded_path_fatal =
                        audio_encode_worker->Snapshot().fatal_error ||
                        audio_sink_worker->Snapshot().fatal_error;
#if defined(VISIONARM_HAS_AV_MUX)
                    encoded_path_fatal = encoded_path_fatal ||
                        (av_mux_enabled && av_muxer->Snapshot().fatal_error);
#endif
#if defined(VISIONARM_HAS_NETWORK_MUX)
                    encoded_path_fatal = encoded_path_fatal ||
                        (network_mux_enabled &&
                         network_muxer->Snapshot().fatal_error);
#endif
                }
                if (encoded_path_fatal) {
                    if (!auxiliary_media_runtime_fault) {
                        visionarm::logging::Log(
                            visionarm::logging::LogLevel::ERROR, "media",
                            "audio/mux/network runtime fault; inference remains active");
                    }
                    auxiliary_media_runtime_fault = true;
                }
#else
                DrainAudioQueue(&audio_pcm_queue, &audio_timeline_stats);
#endif
                if (audio_worker->Snapshot().fatal_error) {
                    if (!auxiliary_media_runtime_fault) {
                        visionarm::logging::Log(
                            visionarm::logging::LogLevel::ERROR, "audio",
                            "runtime fault; inference remains active");
                    }
                    auxiliary_media_runtime_fault = true;
                }
            }
#endif
            if (telemetry_enabled && !telemetry_runtime_fault &&
                (!update_telemetry() ||
                 telemetry_sink->Snapshot().fatal_error)) {
                telemetry_runtime_fault = true;
                visionarm::logging::Log(
                    visionarm::logging::LogLevel::WARN, "telemetry",
                    "runtime fault; pipeline remains active");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (std::chrono::steady_clock::now() >= next_rss_sample) {
                rss.Add(ReadVmRssKb());
                next_rss_sample += std::chrono::seconds(1);
            }
        }
        const auto run_end = std::chrono::steady_clock::now();
        const bool completed_requested_duration = run_end >= deadline;
        const bool terminated_by_signal =
            !completed_requested_duration &&
            g_stop.load(std::memory_order_acquire);
        const double observed_duration_seconds =
            std::chrono::duration<double>(run_end - run_start).count();
        pipeline.Stop();
#if defined(VISIONARM_HAS_ALSA_AUDIO)
        std::optional<visionarm::AudioCaptureWorkerSnapshot> audio_worker_stats;
#if defined(VISIONARM_HAS_AUDIO_ENCODE)
        std::optional<visionarm::AudioEncodeWorkerSnapshot> audio_encode_stats;
        std::optional<visionarm::EncodedAudioSinkWorkerSnapshot> audio_sink_stats;
        visionarm::QueueStatsSnapshot audio_encoded_queue_stats;
#if defined(VISIONARM_HAS_AV_MUX)
        std::optional<visionarm::FfmpegMp4MuxerSnapshot> av_mux_stats;
        bool av_mux_finalize_ok = true;
#endif
#if defined(VISIONARM_HAS_NETWORK_MUX)
        std::optional<visionarm::FfmpegMpegTsUdpSinkSnapshot> network_mux_stats;
        bool network_stop_ok = true;
#endif
#endif
        if (options.audio_enabled) {
            audio_worker->Stop();
#if defined(VISIONARM_HAS_AUDIO_ENCODE)
            if (encoded_audio_enabled) {
                audio_encode_worker->Stop();
                audio_sink_worker->Stop();
                audio_encode_stats = audio_encode_worker->Snapshot();
                audio_sink_stats = audio_sink_worker->Snapshot();
                audio_encoded_queue_stats = audio_encoded_queue->Snapshot();
#if defined(VISIONARM_HAS_AV_MUX)
                if (av_mux_enabled) {
                    av_mux_finalize_ok = av_muxer->Finalize();
                    av_mux_stats = av_muxer->Snapshot();
                }
#endif
#if defined(VISIONARM_HAS_NETWORK_MUX)
                if (network_mux_enabled) {
                    network_stop_ok = network_muxer->Stop();
                    network_mux_stats = network_muxer->Snapshot();
                }
#endif
            } else {
                DrainAudioQueue(&audio_pcm_queue, &audio_timeline_stats);
            }
#else
            DrainAudioQueue(&audio_pcm_queue, &audio_timeline_stats);
#endif
            audio_worker_stats = audio_worker->Snapshot();
        }
        const visionarm::QueueStatsSnapshot audio_queue_stats =
            audio_pcm_queue.Snapshot();
#endif
        std::optional<visionarm::UdpTelemetrySinkSnapshot> telemetry_stats;
        bool telemetry_final_update_ok = true;
        bool telemetry_stop_ok = true;
        if (telemetry_enabled) {
            telemetry_final_update_ok =
                !telemetry_runtime_fault && update_telemetry();
            telemetry_stop_ok = telemetry_start_ok
                ? telemetry_sink->Stop() : false;
            telemetry_stats = telemetry_sink->Snapshot();
        }
        const visionarm::MediaClockSnapshot media_clock_stats =
            media_clock.Snapshot();
        rss.Add(ReadVmRssKb());

        const visionarm::PipelineStatsSnapshot stats = pipeline.stats();
        const visionarm::RgaPreprocessorSnapshot rga_stats =
            preprocessor.snapshot();
        const visionarm::CaptureBufferBrokerSnapshot broker_stats =
            broker.GetSnapshot();
        const visionarm::VideoEncoderSnapshot encoder_stats =
            encoder.snapshot();
        const visionarm::H265FileSinkSnapshot sink_stats =
            file_sink.snapshot();
        const visionarm::TargetStateMachineSnapshot state_stats =
            state_machine.Snapshot();
        const visionarm::MockControlSinkSnapshot mock_control_stats =
            mock_control_sink.Snapshot();

#if defined(VISIONARM_HAS_UART_CONTROL)
        std::optional<visionarm::UartControlSinkSnapshot> uart_control_stats;
        std::optional<visionarm::uart::UartModuleSnapshot> uart_link_stats;
        if (options.control_backend == ControlBackend::UART) {
            uart_control_stats = uart_control_sink->Snapshot();
            uart_link_stats = uart_link->GetSnapshot();
            uart_link->Stop();
        }
#endif

        // Collect the diagnostic superset in memory after all realtime workers
        // have stopped. The product serializer filters this into the requested
        // stable level and performs the only report I/O on this control path.
        std::ostringstream diagnostic_report;
        std::ostream& report = diagnostic_report;
        report << std::fixed << std::setprecision(3);
        report << "module.camera.enabled=1\n";
        report << "module.inference.enabled=1\n";
        report << "module.video.enabled=1\n";
        report << "module.audio.enabled="
               << (options.audio_enabled ? 1 : 0) << '\n';
#if defined(VISIONARM_HAS_AUDIO_ENCODE)
        report << "module.audio_encoder.enabled="
               << (encoded_audio_enabled ? 1 : 0) << '\n';
#else
        report << "module.audio_encoder.enabled=0\n";
#endif
#if defined(VISIONARM_HAS_AV_MUX)
        report << "module.recorder.enabled="
               << (av_mux_enabled ? 1 : 0) << '\n';
#else
        report << "module.recorder.enabled=0\n";
#endif
#if defined(VISIONARM_HAS_NETWORK_MUX)
        report << "module.network.enabled="
               << (network_mux_enabled ? 1 : 0) << '\n';
#else
        report << "module.network.enabled=0\n";
#endif
        report << "module.telemetry.enabled="
               << (telemetry_enabled ? 1 : 0) << '\n';
        report << "module.uart.enabled="
               << (options.control_backend == ControlBackend::UART ? 1 : 0)
               << '\n';
        report << "topology="
               << visionarm::InferenceThreadTopologyName(stats.topology)
               << '\n';
        report << "camera_requested_width=" << options.width << '\n';
        report << "camera_requested_height=" << options.height << '\n';
        report << "camera_requested_fps=" << options.fps << '\n';
        report << "camera_sensor_subdev="
               << visionarm::report::EscapeReportValue(options.sensor_subdev)
               << '\n';
        report << "camera_configured_fps_numerator="
               << configured_sensor_fps.numerator << '\n';
        report << "camera_configured_fps_denominator="
               << configured_sensor_fps.denominator << '\n';
        report << "camera_isp_width=" << camera_format.width << '\n';
        report << "camera_isp_height=" << camera_format.height << '\n';
        report << "media_epoch_monotonic_ns="
               << media_clock_stats.media_epoch_monotonic_ns << '\n';
        report << "requested_duration_seconds="
               << options.duration_seconds << '\n';
        report << "observed_duration_seconds="
               << observed_duration_seconds << '\n';
        report << "camera_fps="
               << RatePerSecond(stats.captured_frames,
                                observed_duration_seconds) << '\n';
        report << "inference_fps="
               << RatePerSecond(stats.inference_successes,
                                observed_duration_seconds) << '\n';
        report << "postprocess_fps="
               << RatePerSecond(stats.postprocess_successes,
                                observed_duration_seconds) << '\n';
        report << "video_fps="
               << RatePerSecond(stats.video_frames_encoded,
                                observed_duration_seconds) << '\n';
        report << "completed_requested_duration="
               << (completed_requested_duration ? 1 : 0) << '\n';
        report << "terminated_by_signal="
               << (terminated_by_signal ? 1 : 0) << '\n';
        report << "auxiliary_media_runtime_fault="
               << (auxiliary_media_runtime_fault ? 1 : 0) << '\n';
#if defined(VISIONARM_HAS_ALSA_AUDIO)
        report << "audio_enabled=" << (options.audio_enabled ? 1 : 0) << '\n';
        if (options.audio_enabled && audio_worker_stats.has_value()) {
            const auto& audio = *audio_worker_stats;
            report << "audio_device="
                   << visionarm::report::EscapeReportValue(
                          audio.capture_info.device)
                   << '\n';
            report << "audio_rate_hz="
                   << audio.capture_info.format.sample_rate_hz << '\n';
            report << "audio_channels="
                   << audio.capture_info.format.channels << '\n';
            report << "audio_period_frames="
                   << audio.capture_info.period_frames << '\n';
            report << "audio_buffer_frames="
                   << audio.capture_info.buffer_frames << '\n';
            report << "audio_timestamp_type="
                   << visionarm::report::EscapeReportValue(
                          audio.capture_info.timestamp_type)
                   << '\n';
            report << "audio_worker_started=" << (audio.started ? 1 : 0) << '\n';
            report << "audio_worker_fatal_error="
                   << (audio.fatal_error ? 1 : 0) << '\n';
            report << "audio_worker_last_error="
                   << visionarm::report::EscapeReportValue(audio.last_error)
                   << '\n';
            report << "audio_timed_chunks=" << audio.timed_chunks << '\n';
            report << "audio_timed_frames=" << audio.timed_frames << '\n';
            report << "audio_timed_bytes=" << audio.timed_bytes << '\n';
            report << "audio_observed_rate_hz="
                   << RatePerSecond(audio.timed_frames,
                                    observed_duration_seconds) << '\n';
            report << "audio_queue_push_failures="
                   << audio.queue_push_failures << '\n';
            report << "audio_xruns=" << audio.capture.xrun_count << '\n';
            report << "audio_suspends=" << audio.capture.suspend_count << '\n';
            report << "audio_recoveries="
                   << audio.capture.recovery_count << '\n';
            report << "audio_short_reads="
                   << audio.capture.short_read_count << '\n';
            report << "audio_status_errors="
                   << audio.capture.status_error_count << '\n';
        }
        WriteQueue(report, "queue.audio_pcm", audio_queue_stats);
#if defined(VISIONARM_HAS_AUDIO_ENCODE)
        report << "encoded_audio_enabled="
               << (encoded_audio_enabled ? 1 : 0) << '\n';
        if (encoded_audio_enabled) {
            report << "audio_encode_bitrate_bps=" << options.audio_bitrate << '\n';
            if (audio_encode_stats.has_value()) {
                const auto& encode = *audio_encode_stats;
                report << "audio_encode_worker_started=" << (encode.started ? 1 : 0) << '\n';
                report << "audio_encode_worker_fatal_error=" << (encode.fatal_error ? 1 : 0) << '\n';
                report << "audio_encode_worker_last_error="
                       << visionarm::report::EscapeReportValue(
                              encode.last_error)
                       << '\n';
                report << "audio_encode_chunks_consumed=" << encode.chunks_consumed << '\n';
                report << "audio_encode_packets_pushed=" << encode.packets_pushed << '\n';
                report << "audio_encode_bytes_pushed=" << encode.bytes_pushed << '\n';
                report << "audio_encode_queue_push_failures=" << encode.queue_push_failures << '\n';
                report << "audio_encoder_input_frames=" << encode.encoder.input_frames << '\n';
                report << "audio_encoder_submitted_codec_frames=" << encode.encoder.submitted_codec_frames << '\n';
                report << "audio_encoder_emitted_packets=" << encode.encoder.emitted_packets << '\n';
                report << "audio_encoder_emitted_bytes=" << encode.encoder.emitted_bytes << '\n';
                report << "audio_encoder_padding_packets=" << encode.encoder.encoder_padding_packets << '\n';
                report << "audio_encoder_failures=" << encode.encoder.encode_failures << '\n';
                report << "audio_encoder_buffered_input_frames=" << encode.encoder.buffered_input_frames << '\n';
                report << "audio_encoder_drained=" << (encode.encoder.drained ? 1 : 0) << '\n';
            }
            WriteQueue(report, "queue.audio_encoded", audio_encoded_queue_stats);
            if (audio_sink_stats.has_value()) {
                const auto& sink = *audio_sink_stats;
                report << "audio_encoded_sink_started=" << (sink.started ? 1 : 0) << '\n';
                report << "audio_encoded_sink_fatal_error=" << (sink.fatal_error ? 1 : 0) << '\n';
                report << "audio_encoded_sink_packets_written=" << sink.packets_written << '\n';
                report << "audio_encoded_sink_bytes_written=" << sink.bytes_written << '\n';
                report << "audio_encoded_sink_failures=" << sink.sink_failures << '\n';
            }
        }
#endif
#if defined(VISIONARM_HAS_AV_MUX)
        report << "local_av_mux_enabled=" << (av_mux_enabled ? 1 : 0) << '\n';
        if (av_mux_enabled) {
            report << "local_av_output="
                   << visionarm::report::EscapeReportValue(options.av_output)
                   << '\n';
            report << "local_av_finalize_ok=" << (av_mux_finalize_ok ? 1 : 0) << '\n';
            if (av_mux_stats.has_value()) {
                const auto& mux = *av_mux_stats;
                report << "local_av_header_written=" << (mux.header_written ? 1 : 0) << '\n';
                report << "local_av_finalized=" << (mux.finalized ? 1 : 0) << '\n';
                report << "local_av_fatal_error=" << (mux.fatal_error ? 1 : 0) << '\n';
                report << "local_av_last_error="
                       << visionarm::report::EscapeReportValue(mux.last_error)
                       << '\n';
                report << "local_av_video_fragments_received=" << mux.video_fragments_received << '\n';
                report << "local_av_video_samples_written=" << mux.video_samples_written << '\n';
                report << "local_av_video_bytes_written=" << mux.video_bytes_written << '\n';
                report << "local_av_audio_packets_written=" << mux.audio_packets_written << '\n';
                report << "local_av_audio_bytes_written=" << mux.audio_bytes_written << '\n';
                report << "local_av_write_failures=" << mux.write_failures << '\n';
                report << "local_av_first_video_pts_us=" << mux.first_video_pts_us << '\n';
                report << "local_av_last_video_pts_us=" << mux.last_video_pts_us << '\n';
                report << "local_av_first_audio_pts_ns=" << mux.first_audio_pts_ns << '\n';
                report << "local_av_last_audio_pts_ns=" << mux.last_audio_pts_ns << '\n';
            }
        }
#endif
#if defined(VISIONARM_HAS_NETWORK_MUX)
        report << "network_mux_enabled="
               << (network_mux_enabled ? 1 : 0) << '\n';
        if (network_mux_enabled) {
            report << "network_url="
                   << visionarm::report::EscapeReportValue(options.network_url)
                   << '\n';
            report << "network_queue_capacity=" << options.network_queue << '\n';
            report << "network_io_timeout_ms="
                   << options.network_io_timeout_ms << '\n';
            report << "network_rate_bps="
                   << EffectiveNetworkRateBps(options) << '\n';
            report << "network_burst_bits="
                   << EffectiveNetworkBurstBits(options) << '\n';
            report << "network_packet_size="
                   << options.network_packet_size << '\n';
            report << "network_send_buffer_bytes="
                   << options.network_send_buffer_bytes << '\n';
            report << "network_stop_ok=" << (network_stop_ok ? 1 : 0) << '\n';
            if (network_mux_stats.has_value()) {
                const auto& network = *network_mux_stats;
                report << "network_started=" << (network.started ? 1 : 0) << '\n';
                report << "network_finalized=" << (network.finalized ? 1 : 0) << '\n';
                report << "network_fatal_error="
                       << (network.fatal_error ? 1 : 0) << '\n';
                report << "network_last_error="
                       << visionarm::report::EscapeReportValue(
                              network.last_error)
                       << '\n';
                report << "network_video_fragments_received="
                       << network.video_fragments_received << '\n';
                report << "network_video_access_units_enqueued="
                       << network.video_access_units_enqueued << '\n';
                report << "network_video_access_units_written="
                       << network.video_access_units_written << '\n';
                report << "network_video_bytes_written="
                       << network.video_bytes_written << '\n';
                report << "network_video_parameter_set_injections="
                       << network.video_parameter_set_injections << '\n';
                report << "network_audio_packets_enqueued="
                       << network.audio_packets_enqueued << '\n';
                report << "network_audio_packets_written="
                       << network.audio_packets_written << '\n';
                report << "network_audio_bytes_written="
                       << network.audio_bytes_written << '\n';
                report << "network_queue_overload_failures="
                       << network.queue_overload_failures << '\n';
                report << "network_write_failures="
                       << network.write_failures << '\n';
                report << "network_first_video_pts_us="
                       << network.first_video_pts_us << '\n';
                report << "network_last_video_pts_us="
                       << network.last_video_pts_us << '\n';
                report << "network_first_audio_pts_ns="
                       << network.first_audio_pts_ns << '\n';
                report << "network_last_audio_pts_ns="
                       << network.last_audio_pts_ns << '\n';
                WriteQueue(report, "queue.network", network.packet_queue);
            }
        }
#endif
        report << "audio_timeline_chunks=" << audio_timeline_stats.chunks << '\n';
        report << "audio_timeline_frames=" << audio_timeline_stats.frames << '\n';
        report << "audio_timeline_bytes=" << audio_timeline_stats.bytes << '\n';
        report << "audio_timeline_reanchors="
               << audio_timeline_stats.reanchors << '\n';
        report << "audio_timeline_discontinuities="
               << audio_timeline_stats.discontinuities << '\n';
        report << "audio_timeline_pts_regressions="
               << audio_timeline_stats.pts_regressions << '\n';
        report << "audio_timeline_continuous_pts_mismatches="
               << audio_timeline_stats.continuous_pts_mismatches << '\n';
        report << "audio_timeline_first_pts_ns="
               << audio_timeline_stats.first_pts_ns << '\n';
        report << "audio_timeline_last_pts_ns="
               << audio_timeline_stats.last_pts_ns << '\n';
        report << "audio_timeline_last_end_pts_ns="
               << audio_timeline_stats.last_end_pts_ns << '\n';
        report << "audio_timeline_max_forward_gap_ns="
               << audio_timeline_stats.maximum_forward_gap_ns << '\n';
        report << "audio_timeline_max_abs_timing_error_ns="
               << audio_timeline_stats.maximum_abs_timing_error_ns << '\n';
#endif
        report << "media_audio_anchor_valid="
               << (media_clock_stats.audio_anchor_valid ? 1 : 0) << '\n';
        report << "media_audio_chunks_stamped="
               << media_clock_stats.audio_chunks_stamped << '\n';
        report << "media_audio_reanchors="
               << media_clock_stats.audio_reanchors << '\n';
        report << "media_audio_discontinuities="
               << media_clock_stats.audio_discontinuities << '\n';
        report << "media_audio_timestamp_failures="
               << media_clock_stats.audio_timestamp_failures << '\n';
        report << "media_audio_latest_timing_error_ns="
               << media_clock_stats.latest_audio_timing_error_ns << '\n';
        report << "media_audio_max_abs_timing_error_ns="
               << media_clock_stats.maximum_abs_audio_timing_error_ns << '\n';
        report << "media_audio_last_pts_ns="
               << media_clock_stats.last_audio_pts_ns << '\n';
        report << "media_audio_last_end_pts_ns="
               << media_clock_stats.last_audio_end_pts_ns << '\n';
        report << "model_input_width=" << kModelWidth << '\n';
        report << "model_input_height=" << kModelHeight << '\n';
        report << "input_slots=" << options.input_slots << '\n';
        report << "output_slots=" << options.output_slots << '\n';
        report << "latest_frame_queue_capacity=1\n";
        report << "acquire_hits=" << options.acquire_hits << '\n';
        report << "lost_misses=" << options.lost_misses << '\n';
        report << "max_result_age_ms=" << options.max_result_age_ms << '\n';
        report << "control_backend="
               << visionarm::report::EscapeReportValue(
                      ControlBackendName(options.control_backend))
               << '\n';
        if (options.control_backend == ControlBackend::UART) {
            report << "uart_device="
                   << visionarm::report::EscapeReportValue(options.uart_device)
                   << '\n';
            report << "uart_baud=" << options.uart_baud << '\n';
            report << "uart_ready_timeout_ms="
                   << options.uart_ready_timeout_ms << '\n';
        }
        report << "telemetry_enabled=" << (telemetry_enabled ? 1 : 0) << '\n';
        if (telemetry_enabled) {
            report << "telemetry_host="
                   << visionarm::report::EscapeReportValue(
                          options.telemetry_host)
                   << '\n';
            report << "telemetry_port=" << options.telemetry_port << '\n';
            report << "telemetry_interval_ms="
                   << options.telemetry_interval_ms << '\n';
            report << "telemetry_send_buffer_bytes="
                   << options.telemetry_send_buffer_bytes << '\n';
            report << "telemetry_start_ok="
                   << (telemetry_start_ok ? 1 : 0) << '\n';
            report << "telemetry_start_error="
                   << visionarm::report::EscapeReportValue(
                          telemetry_start_error)
                   << '\n';
            report << "telemetry_runtime_fault="
                   << (telemetry_runtime_fault ? 1 : 0) << '\n';
            report << "telemetry_final_update_ok="
                   << (telemetry_final_update_ok ? 1 : 0) << '\n';
            report << "telemetry_stop_ok="
                   << (telemetry_stop_ok ? 1 : 0) << '\n';
            if (telemetry_stats.has_value()) {
                const auto& telemetry = *telemetry_stats;
                report << "telemetry_started="
                       << (telemetry.started ? 1 : 0) << '\n';
                report << "telemetry_stopped_cleanly="
                       << (telemetry.stopped_cleanly ? 1 : 0) << '\n';
                report << "telemetry_fatal_error="
                       << (telemetry.fatal_error ? 1 : 0) << '\n';
                report << "telemetry_last_error="
                       << visionarm::report::EscapeReportValue(
                              telemetry.last_error)
                       << '\n';
                report << "telemetry_control_updates="
                       << telemetry.control_updates << '\n';
                report << "telemetry_runtime_updates="
                       << telemetry.runtime_updates << '\n';
                report << "telemetry_datagrams_attempted="
                       << telemetry.datagrams_attempted << '\n';
                report << "telemetry_datagrams_sent="
                       << telemetry.datagrams_sent << '\n';
                report << "telemetry_bytes_sent="
                       << telemetry.bytes_sent << '\n';
                report << "telemetry_send_failures="
                       << telemetry.send_failures << '\n';
                report << "telemetry_serialization_failures="
                       << telemetry.serialization_failures << '\n';
                report << "telemetry_oversized_datagrams="
                       << telemetry.oversized_datagrams << '\n';
                report << "telemetry_final_sequence="
                       << telemetry.final_sequence << '\n';
            }
        }

        report << "captured_frames=" << stats.captured_frames << '\n';
        report << "camera_timeouts=" << stats.camera_timeouts << '\n';
        report << "camera_wakes=" << stats.camera_wakes << '\n';
        report << "driver_dropped_frames="
               << stats.driver_dropped_frames << '\n';
        report << "replaced_waiting_frames="
               << stats.replaced_waiting_frames << '\n';
        report << "skipped_no_input_slot="
               << stats.skipped_no_input_slot << '\n';
        report << "preprocess_failures=" << stats.preprocess_failures << '\n';
        report << "inference_successes=" << stats.inference_successes << '\n';
        report << "inference_failures=" << stats.inference_failures << '\n';
        report << "postprocess_successes="
               << stats.postprocess_successes << '\n';
        report << "postprocess_failures="
               << stats.postprocess_failures << '\n';
        report << "result_publish_failures="
               << stats.result_publish_failures << '\n';
        report << "requeue_failures=" << stats.requeue_failures << '\n';
        report << "dmabuf_sync_failures="
               << stats.dmabuf_sync_failures << '\n';

        report << "video_frames_encoded="
               << stats.video_frames_encoded << '\n';
        report << "video_frames_dropped="
               << stats.video_frames_dropped << '\n';
        report << "video_branch_failed="
               << (stats.video_branch_failed ? 1 : 0) << '\n';
        report << "video_encode_failures="
               << stats.video_encode_failures << '\n';
        report << "video_packets_dropped="
               << stats.video_packets_dropped << '\n';
        report << "video_sink_failures="
               << stats.video_sink_failures << '\n';

        report << "camera_buffer_count_at_start="
               << stats.camera_buffer_count_at_start << '\n';
        report << "camera_outstanding_before_stop="
               << stats.camera_outstanding_before_stop << '\n';
        report << "broker_outstanding_frames_before_camera_stop="
               << stats.broker_outstanding_frames_before_camera_stop << '\n';
        report << "broker_outstanding_leases_before_camera_stop="
               << stats.broker_outstanding_leases_before_camera_stop << '\n';
        report << "broker_outstanding_frames_after_stop="
               << broker_stats.outstanding_frames << '\n';
        report << "broker_outstanding_leases_after_stop="
               << broker_stats.outstanding_leases << '\n';
        report << "fatal_error=" << (stats.fatal_error ? 1 : 0) << '\n';
        report << "graceful_shutdown_completed="
               << (stats.graceful_shutdown_completed ? 1 : 0) << '\n';
        report << "split_final_completed_frame_drained="
               << (stats.split_final_completed_frame_drained ? 1 : 0)
               << '\n';

        report << "rga_process_calls=" << rga_stats.process_calls << '\n';
        report << "rga_process_successes="
               << rga_stats.process_successes << '\n';
        report << "mpp_encoded_frames="
               << encoder_stats.encoded_frames << '\n';
        report << "mpp_encode_failures="
               << encoder_stats.encode_failures << '\n';
        report << "mpp_source_reimports="
               << encoder_stats.source_buffer_reimports << '\n';
        report << "h265_bytes_written=" << sink_stats.bytes_written << '\n';
        report << "h265_write_failures="
               << sink_stats.write_failures << '\n';

        report << "state_processed_packets="
               << state_stats.processed_packets << '\n';
        report << "state_valid_controls="
               << state_stats.valid_controls << '\n';
        report << "state_control_sink_failures="
               << state_stats.control_sink_failures << '\n';
        report << "state_perception_sink_failures="
               << state_stats.perception_sink_failures << '\n';
        report << "state_invalid_timestamp_packets="
               << state_stats.invalid_timestamp_packets << '\n';
        for (std::size_t index = 0U;
             index < state_stats.state_counts.size(); ++index) {
            report << "state_count."
                   << visionarm::TargetStateName(
                          static_cast<visionarm::TargetState>(index))
                   << '=' << state_stats.state_counts[index] << '\n';
        }
        if (options.control_backend == ControlBackend::MOCK) {
            report << "control_submissions="
                   << mock_control_stats.submissions << '\n';
            report << "control_valid="
                   << mock_control_stats.valid_controls << '\n';
            report << "control_invalid="
                   << mock_control_stats.invalid_controls << '\n';
        }
#if defined(VISIONARM_HAS_UART_CONTROL)
        else if (uart_control_stats.has_value() &&
                 uart_link_stats.has_value()) {
            const auto& adapter = *uart_control_stats;
            const auto& link = *uart_link_stats;
            report << "uart.adapter.submissions="
                   << adapter.metrics.submissions << '\n';
            report << "uart.adapter.accepted="
                   << adapter.metrics.accepted << '\n';
            report << "uart.adapter.rejected="
                   << adapter.metrics.rejected << '\n';
            report << "uart.adapter.valid_inputs="
                   << adapter.metrics.valid_inputs << '\n';
            report << "uart.adapter.invalid_inputs="
                   << adapter.metrics.invalid_inputs << '\n';
            report << "uart.adapter.nonfinite_invalidations="
                   << adapter.metrics.nonfinite_invalidations << '\n';
            report << "uart.adapter.invalid_timestamp_invalidations="
                   << adapter.metrics.invalid_timestamp_invalidations << '\n';
            report << "uart.adapter.invalid_state_invalidations="
                   << adapter.metrics.invalid_state_invalidations << '\n';
            report << "uart.adapter.identity_truncations="
                   << adapter.metrics.identity_truncations << '\n';
            report << "uart.link_state_before_stop="
                   << visionarm::report::EscapeReportValue(
                          visionarm::uart::LinkStateName(link.state))
                   << '\n';
            report << "uart.peer_boot_id_valid="
                   << (link.peer_boot_id_valid ? 1 : 0) << '\n';
            report << "uart.peer_boot_id=" << link.peer_boot_id << '\n';
            report << "uart.tx_bytes=" << link.metrics.tx_bytes << '\n';
            report << "uart.rx_bytes=" << link.metrics.rx_bytes << '\n';
            report << "uart.control_accepted="
                   << link.metrics.control_accepted << '\n';
            report << "uart.control_overwritten="
                   << link.metrics.control_overwritten << '\n';
            report << "uart.control_sent="
                   << link.metrics.control_sent << '\n';
            report << "uart.valid_control_sent="
                   << link.metrics.valid_control_sent << '\n';
            report << "uart.invalid_control_sent="
                   << link.metrics.invalid_control_sent << '\n';
            report << "uart.hello_ack_received="
                   << link.metrics.hello_ack_received << '\n';
            report << "uart.status_received="
                   << link.metrics.status_received << '\n';
            report << "uart.poll_errors="
                   << link.metrics.poll_errors << '\n';
            report << "uart.read_errors="
                   << link.metrics.read_errors << '\n';
            report << "uart.write_errors="
                   << link.metrics.write_errors << '\n';
            report << "uart.message_decode_errors="
                   << link.metrics.message_decode_errors << '\n';
            report << "uart.unexpected_responses="
                   << link.metrics.unexpected_responses << '\n';
            report << "uart.parser_crc_errors="
                   << link.metrics.parser.crc_errors << '\n';
            report << "uart.parser_length_errors="
                   << link.metrics.parser.length_errors << '\n';
            report << "uart.parser_version_errors="
                   << link.metrics.parser.version_errors << '\n';
            report << "uart.parser_escape_errors="
                   << link.metrics.parser.escape_errors << '\n';
            report << "uart.parser_oversize_errors="
                   << link.metrics.parser.oversize_errors << '\n';
        }
#endif

        WriteQueue(report, "queue.captured", stats.captured_frame_queue);
        WriteQueue(report, "queue.prepared", stats.prepared_frame_queue);
        WriteQueue(report, "queue.completed", stats.completed_frame_queue);
        WriteQueue(report, "queue.video", stats.video_frame_queue);
        WriteQueue(report, "queue.encoded", stats.encoded_packet_queue);

        WriteLatency(report, "latency.input_slot_wait",
                     stats.timing.input_slot_wait);
        WriteLatency(report, "latency.latest_frame_queue_wait",
                     stats.timing.latest_frame_queue_wait);
        WriteLatency(report, "latency.capture_to_preprocess_start",
                     stats.timing.capture_to_preprocess_start);
        WriteLatency(report, "latency.preprocess",
                     stats.timing.preprocess);
        WriteLatency(report, "latency.rknn_input_submit",
                     stats.timing.rknn_input_submit);
        WriteLatency(report, "latency.rknn_output_bind",
                     stats.timing.rknn_output_bind);
        WriteLatency(report, "latency.rknn_bind_total",
                     stats.timing.rknn_bind_total);
        WriteLatency(report, "latency.rknn_run",
                     stats.timing.rknn_run);
        WriteLatency(report, "latency.rknn_output_get",
                     stats.timing.rknn_output_get);
        WriteLatency(report, "latency.rknn_output_release",
                     stats.timing.rknn_output_release);
        WriteLatency(report, "latency.rknn_total",
                     stats.timing.rknn_total);
        WriteLatency(report, "latency.postprocess",
                     stats.timing.postprocess);
        WriteLatency(report, "latency.capture_to_result",
                     stats.timing.capture_to_result);
        WriteLatency(report, "latency.result_age",
                     stats.timing.result_age);

        report << "rss.samples=" << rss.samples << '\n';
        report << "rss.first_kb=" << rss.first_kb << '\n';
        report << "rss.last_kb=" << rss.last_kb << '\n';
        report << "rss.minimum_kb=" << rss.minimum_kb << '\n';
        report << "rss.maximum_kb=" << rss.maximum_kb << '\n';
        report << "rss.growth_kb=" << rss.GrowthKb() << '\n';
        report << "rss.enforced_growth_limit_kb="
               << options.max_rss_growth_kb << '\n';

        const bool rss_ok = options.max_rss_growth_kb == 0 ||
            rss.GrowthKb() <= options.max_rss_growth_kb;
        const bool timing_ok =
            LatencyComplete(stats.timing.input_slot_wait,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.latest_frame_queue_wait,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.capture_to_preprocess_start,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.preprocess,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.rknn_input_submit,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.rknn_output_bind,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.rknn_bind_total,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.rknn_run,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.rknn_output_get,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.rknn_output_release,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.rknn_total,
                            stats.inference_successes) &&
            LatencyComplete(stats.timing.postprocess,
                            stats.postprocess_successes) &&
            LatencyComplete(stats.timing.capture_to_result,
                            stats.postprocess_successes) &&
            LatencyComplete(stats.timing.result_age,
                            stats.postprocess_successes);
        const bool completed_queue_ok =
            stats.topology ==
                visionarm::InferenceThreadTopology::FUSED_NPU_POSTPROCESS ||
            QueueDrained(stats.completed_frame_queue);
        const bool queues_ok =
            QueueDrained(stats.captured_frame_queue) &&
            QueueDrained(stats.prepared_frame_queue) &&
            completed_queue_ok &&
            QueueDrained(stats.video_frame_queue) &&
            QueueDrained(stats.encoded_packet_queue);
        const bool throughput_ok =
            stats.camera_timeouts == 0U &&
            stats.driver_dropped_frames == 0U &&
            stats.replaced_waiting_frames == 0U &&
            stats.skipped_no_input_slot == 0U &&
            stats.captured_frames == stats.inference_successes &&
            stats.inference_successes == stats.postprocess_successes &&
            stats.captured_frames == stats.video_frames_encoded &&
            rga_stats.process_calls == rga_stats.process_successes &&
            rga_stats.process_successes == stats.inference_successes &&
            encoder_stats.encoded_frames == stats.video_frames_encoded;

        bool audio_ok = true;
#if defined(VISIONARM_HAS_ALSA_AUDIO)
        if (options.audio_enabled) {
            audio_ok =
                audio_worker_stats.has_value() &&
                audio_worker_stats->started &&
                !audio_worker_stats->fatal_error &&
                audio_worker_stats->timed_chunks > 0U &&
                audio_worker_stats->timed_frames > 0U &&
                audio_worker_stats->queue_push_failures == 0U &&
                audio_worker_stats->capture.xrun_count == 0U &&
                audio_worker_stats->capture.suspend_count == 0U &&
                audio_worker_stats->capture.recovery_count == 0U &&
                audio_worker_stats->capture.short_read_count == 0U &&
                audio_worker_stats->capture.status_error_count == 0U &&
                QueueDrained(audio_queue_stats) &&
                audio_timeline_stats.chunks ==
                    audio_worker_stats->timed_chunks &&
                audio_timeline_stats.frames ==
                    audio_worker_stats->timed_frames &&
                audio_timeline_stats.bytes ==
                    audio_worker_stats->timed_bytes &&
                audio_worker_stats->timed_bytes ==
                    audio_worker_stats->timed_frames *
                    audio_worker_stats->capture_info.format.channels * 2U &&
                audio_timeline_stats.reanchors > 0U &&
                audio_timeline_stats.discontinuities == 0U &&
                audio_timeline_stats.pts_regressions == 0U &&
                audio_timeline_stats.continuous_pts_mismatches == 0U &&
                audio_timeline_stats.maximum_forward_gap_ns == 0 &&
                audio_timeline_stats.first_pts_ns <
                    audio_timeline_stats.last_pts_ns &&
                audio_timeline_stats.last_pts_ns <
                    audio_timeline_stats.last_end_pts_ns &&
                media_clock_stats.audio_anchor_valid &&
                (media_clock_stats.audio_chunks_stamped ==
                     audio_worker_stats->timed_chunks ||
                 media_clock_stats.audio_chunks_stamped ==
                     audio_worker_stats->timed_chunks + 1U) &&
                media_clock_stats.audio_reanchors ==
                    audio_timeline_stats.reanchors &&
                media_clock_stats.audio_discontinuities == 0U &&
                media_clock_stats.audio_timestamp_failures == 0U;
        }
#endif
        report << "audio_path_ok=" << (audio_ok ? 1 : 0) << '\n';

        bool audio_encode_ok = true;
        bool local_av_mux_ok = true;
#if defined(VISIONARM_HAS_AUDIO_ENCODE)
        if (encoded_audio_enabled) {
            audio_encode_ok =
                audio_encode_stats.has_value() &&
                audio_encode_stats->started &&
                !audio_encode_stats->fatal_error &&
                audio_encode_stats->chunks_consumed > 0U &&
                audio_encode_stats->chunks_consumed ==
                    audio_timeline_stats.chunks &&
                audio_encode_stats->packets_pushed > 0U &&
                audio_encode_stats->queue_push_failures == 0U &&
                audio_encode_stats->encoder.input_frames ==
                    audio_timeline_stats.frames &&
                audio_encode_stats->encoder.emitted_packets ==
                    audio_encode_stats->packets_pushed &&
                audio_encode_stats->encoder.emitted_bytes ==
                    audio_encode_stats->bytes_pushed &&
                audio_encode_stats->encoder.encode_failures == 0U &&
                audio_encode_stats->encoder.buffered_input_frames == 0U &&
                audio_encode_stats->encoder.drained &&
                QueueDrained(audio_encoded_queue_stats) &&
                audio_sink_stats.has_value() &&
                audio_sink_stats->started &&
                !audio_sink_stats->fatal_error &&
                audio_sink_stats->sink_failures == 0U &&
                audio_sink_stats->packets_written ==
                    audio_encode_stats->packets_pushed &&
                audio_sink_stats->bytes_written ==
                    audio_encode_stats->bytes_pushed;
        }
#endif

#if defined(VISIONARM_HAS_AV_MUX)
        if (av_mux_enabled) {
            local_av_mux_ok =
                av_mux_finalize_ok &&
                av_mux_stats.has_value() &&
                av_mux_stats->header_written &&
                av_mux_stats->finalized &&
                !av_mux_stats->fatal_error &&
                av_mux_stats->write_failures == 0U &&
                av_mux_stats->video_samples_written > 0U &&
                av_mux_stats->video_fragments_received ==
                    stats.video_frames_encoded &&
                av_mux_stats->video_samples_written ==
                    stats.video_frames_encoded &&
                av_mux_stats->audio_packets_written > 0U &&
                audio_sink_stats.has_value() &&
                av_mux_stats->audio_packets_written ==
                    audio_sink_stats->packets_written &&
                av_mux_stats->audio_bytes_written ==
                    audio_sink_stats->bytes_written &&
                av_mux_stats->first_video_pts_us <
                    av_mux_stats->last_video_pts_us &&
                av_mux_stats->first_audio_pts_ns <
                    av_mux_stats->last_audio_pts_ns;
        }
#endif
        report << "audio_encode_path_ok=" << (audio_encode_ok ? 1 : 0) << '\n';
        report << "local_av_mux_ok=" << (local_av_mux_ok ? 1 : 0) << '\n';

        bool network_mux_ok = true;
#if defined(VISIONARM_HAS_NETWORK_MUX)
        if (network_mux_enabled) {
            network_mux_ok =
                network_stop_ok &&
                network_mux_stats.has_value() &&
                network_mux_stats->started &&
                network_mux_stats->finalized &&
                !network_mux_stats->fatal_error &&
                network_mux_stats->queue_overload_failures == 0U &&
                network_mux_stats->write_failures == 0U &&
                network_mux_stats->video_fragments_received ==
                    stats.video_frames_encoded &&
                network_mux_stats->video_access_units_enqueued > 0U &&
                network_mux_stats->video_access_units_enqueued ==
                    stats.video_frames_encoded &&
                network_mux_stats->video_access_units_written ==
                    network_mux_stats->video_access_units_enqueued &&
                network_mux_stats->audio_packets_enqueued > 0U &&
                audio_sink_stats.has_value() &&
                network_mux_stats->audio_packets_enqueued ==
                    audio_sink_stats->packets_written &&
                network_mux_stats->audio_packets_written ==
                    network_mux_stats->audio_packets_enqueued &&
                network_mux_stats->audio_bytes_written ==
                    audio_sink_stats->bytes_written &&
                network_mux_stats->first_video_pts_us <
                    network_mux_stats->last_video_pts_us &&
                network_mux_stats->first_audio_pts_ns <
                    network_mux_stats->last_audio_pts_ns &&
                QueueDrained(network_mux_stats->packet_queue);
        }
#endif
        report << "network_mux_ok=" << (network_mux_ok ? 1 : 0) << '\n';

        bool telemetry_ok = true;
        if (telemetry_enabled) {
            telemetry_ok =
                telemetry_start_ok && !telemetry_runtime_fault &&
                telemetry_final_update_ok && telemetry_stop_ok &&
                telemetry_stats.has_value() &&
                telemetry_stats->started &&
                telemetry_stats->stopped_cleanly &&
                !telemetry_stats->running &&
                !telemetry_stats->fatal_error &&
                telemetry_stats->control_updates > 0U &&
                telemetry_stats->control_updates <=
                    state_stats.processed_packets &&
                telemetry_stats->runtime_updates > 0U &&
                telemetry_stats->datagrams_attempted > 0U &&
                telemetry_stats->datagrams_attempted ==
                    telemetry_stats->datagrams_sent &&
                telemetry_stats->final_sequence ==
                    telemetry_stats->datagrams_attempted &&
                telemetry_stats->bytes_sent > 0U &&
                telemetry_stats->send_failures == 0U &&
                telemetry_stats->serialization_failures == 0U &&
                telemetry_stats->oversized_datagrams == 0U;
        }
        report << "telemetry_ok=" << (telemetry_ok ? 1 : 0) << '\n';

        bool control_ok = false;
        if (options.control_backend == ControlBackend::MOCK) {
            control_ok =
                mock_control_stats.submissions == state_stats.processed_packets;
        }
#if defined(VISIONARM_HAS_UART_CONTROL)
        else if (uart_control_stats.has_value() &&
                 uart_link_stats.has_value()) {
            const auto& adapter = *uart_control_stats;
            const auto& link = *uart_link_stats;
            const bool link_state_ok =
                link.state == visionarm::uart::LinkState::READY ||
                link.state == visionarm::uart::LinkState::DEGRADED;
            control_ok =
                adapter.metrics.submissions == state_stats.processed_packets &&
                adapter.metrics.accepted == adapter.metrics.submissions &&
                adapter.metrics.rejected == 0U &&
                adapter.metrics.accepted ==
                    adapter.metrics.valid_inputs +
                        adapter.metrics.invalid_inputs &&
                adapter.metrics.valid_inputs ==
                    state_stats.valid_controls &&
                adapter.metrics.nonfinite_invalidations == 0U &&
                adapter.metrics.invalid_timestamp_invalidations == 0U &&
                adapter.metrics.invalid_state_invalidations == 0U &&
                adapter.metrics.identity_truncations == 0U &&
                link_state_ok &&
                link.peer_boot_id_valid &&
                link.metrics.tx_bytes > 0U &&
                link.metrics.rx_bytes > 0U &&
                link.metrics.control_accepted == adapter.metrics.accepted &&
                link.metrics.control_accepted ==
                    link.metrics.control_overwritten +
                        link.metrics.control_sent &&
                link.metrics.hello_ack_received > 0U &&
                link.metrics.status_received > 0U &&
                link.metrics.control_sent > 0U &&
                link.metrics.control_sent ==
                    link.metrics.valid_control_sent +
                        link.metrics.invalid_control_sent &&
                link.metrics.poll_errors == 0U &&
                link.metrics.read_errors == 0U &&
                link.metrics.write_errors == 0U &&
                link.metrics.message_decode_errors == 0U &&
                link.metrics.unexpected_responses == 0U &&
                link.metrics.parser.crc_errors == 0U &&
                link.metrics.parser.length_errors == 0U &&
                link.metrics.parser.version_errors == 0U &&
                link.metrics.parser.escape_errors == 0U &&
                link.metrics.parser.oversize_errors == 0U;
        }
#endif

        const bool passed =
            completed_requested_duration &&
            !terminated_by_signal &&
            stats.captured_frames > 0U &&
            stats.video_frames_encoded > 0U &&
            stats.inference_successes > 0U &&
            stats.postprocess_successes > 0U &&
            stats.preprocess_failures == 0U &&
            stats.inference_failures == 0U &&
            stats.postprocess_failures == 0U &&
            stats.result_publish_failures == 0U &&
            stats.video_encode_failures == 0U &&
            !stats.video_branch_failed &&
            stats.video_frames_dropped == 0U &&
            stats.video_packets_dropped == 0U &&
            stats.video_sink_failures == 0U &&
            stats.requeue_failures == 0U &&
            stats.dmabuf_sync_failures == 0U &&
            stats.camera_buffer_count_at_start > 0U &&
            stats.camera_outstanding_before_stop == 0U &&
            stats.broker_outstanding_frames_before_camera_stop == 0U &&
            stats.broker_outstanding_leases_before_camera_stop == 0U &&
            !stats.fatal_error &&
            stats.graceful_shutdown_completed &&
            stats.split_final_completed_frame_drained &&
            broker_stats.outstanding_frames == 0U &&
            broker_stats.outstanding_leases == 0U &&
            encoder_stats.encode_failures == 0U &&
            encoder_stats.source_buffer_reimports == 0U &&
            sink_stats.write_failures == 0U &&
            sink_stats.bytes_written > 0U &&
            rga_stats.process_successes > 0U &&
            state_stats.control_sink_failures == 0U &&
            state_stats.perception_sink_failures == 0U &&
            state_stats.invalid_timestamp_packets == 0U &&
            control_ok &&
            !auxiliary_media_runtime_fault &&
            audio_ok &&
            audio_encode_ok &&
            local_av_mux_ok &&
            network_mux_ok &&
            telemetry_ok &&
            queues_ok && throughput_ok && timing_ok && rss_ok;

        report << "control_ok=" << (control_ok ? 1 : 0) << '\n';

        encoder.Shutdown();
        engine.Shutdown();
        visionarm::logging::Log(
            passed ? visionarm::logging::LogLevel::INFO
                   : visionarm::logging::LogLevel::ERROR,
            "runtime", "shutdown pipeline_result=",
            (passed ? "PASS" : "FAIL"),
            " duration_sec=", observed_duration_seconds,
            " captured_frames=", stats.captured_frames,
            " inference_successes=", stats.inference_successes,
            " video_frames_encoded=", stats.video_frames_encoded);
        const bool logger_flush_ok = visionarm::logging::FlushGlobalLogger();
        const visionarm::logging::LoggerSnapshot logger_stats =
            visionarm::logging::GlobalLoggerSnapshot();
        const bool product_passed =
            passed && logger_flush_ok &&
            logger_stats.current_size == 0U &&
            logger_stats.emitted == logger_stats.accepted &&
            logger_stats.dropped ==
                logger_stats.dropped_contention +
                    logger_stats.dropped_overflow &&
            logger_stats.high_watermark <=
                logger_stats.queue_capacity +
                    logger_stats.critical_queue_capacity &&
            logger_stats.dropped_critical == 0U &&
            logger_stats.sink_failures == 0U;
        report << "log.queue_capacity=" << logger_stats.queue_capacity << '\n';
        report << "log.critical_queue_capacity="
               << logger_stats.critical_queue_capacity << '\n';
        report << "log.current_size=" << logger_stats.current_size << '\n';
        report << "log.high_watermark=" << logger_stats.high_watermark << '\n';
        report << "log.accepted=" << logger_stats.accepted << '\n';
        report << "log.emitted=" << logger_stats.emitted << '\n';
        report << "log.dropped=" << logger_stats.dropped << '\n';
        report << "log.dropped_contention="
               << logger_stats.dropped_contention << '\n';
        report << "log.dropped_overflow="
               << logger_stats.dropped_overflow << '\n';
        report << "log.dropped_critical="
               << logger_stats.dropped_critical << '\n';
        report << "log.sink_failures=" << logger_stats.sink_failures << '\n';
        report << "log.flush_ok=" << (logger_flush_ok ? 1 : 0) << '\n';
        report << "result=" << (product_passed ? "PASS" : "FAIL") << '\n';
        report << "vision_pipeline_r7_r8_probe="
               << (product_passed ? "PASS" : "FAIL") << '\n';

        std::string report_error;
        const bool report_ok = options.report.empty()
            ? visionarm::report::WriteRuntimeReport(
                  diagnostic_report.str(), options.report_level,
                  std::cout, &report_error)
            : visionarm::report::WriteRuntimeReportFile(
                  options.report, diagnostic_report.str(),
                  options.report_level, &report_error);
        if (!report_ok) {
            visionarm::logging::Log(
                visionarm::logging::LogLevel::FATAL, "report",
                "report write failed: ", report_error);
            (void)visionarm::logging::FlushGlobalLogger();
        }
        visionarm::logging::ShutdownGlobalLogger();
        return product_passed && report_ok ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        visionarm::logging::Log(
            visionarm::logging::LogLevel::FATAL, "runtime",
            "fatal startup/runtime exception: ", error.what());
        (void)visionarm::logging::FlushGlobalLogger();
        const visionarm::logging::LoggerSnapshot logger_stats =
            visionarm::logging::GlobalLoggerSnapshot();

        visionarm::report::ReportLevel report_level =
            visionarm::report::ReportLevel::SUMMARY;
        std::string report_path = FindOptionValue(argc, argv, "--report");
        if (options_parsed) {
            report_path = options.report;
            report_level = options.report_level;
        } else {
            const std::string level_text =
                FindOptionValue(argc, argv, "--report-level");
            (void)visionarm::report::ParseReportLevel(
                level_text, &report_level);
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
              << (options_parsed ? options.duration_seconds : 0) << '\n'
              << "observed_duration_seconds=0\n"
              << "completed_requested_duration=0\n"
              << "terminated_by_signal=0\n"
              << "fatal_error=1\n"
              << "fatal_message="
              << visionarm::report::EscapeReportValue(error.what()) << '\n'
              << "graceful_shutdown_completed=0\n"
              << "log.accepted=" << logger_stats.accepted << '\n'
              << "log.emitted=" << logger_stats.emitted << '\n'
              << "log.dropped=" << logger_stats.dropped << '\n'
              << "log.dropped_critical=" << logger_stats.dropped_critical
              << '\n'
              << "log.sink_failures=" << logger_stats.sink_failures << '\n'
              << "result=FAIL\n";
        std::string report_error;
        const bool fault_report_ok = report_path.empty()
            ? visionarm::report::WriteRuntimeReport(
                  fault.str(), report_level, std::cout, &report_error)
            : visionarm::report::WriteRuntimeReportFile(
                  report_path, fault.str(), report_level, &report_error);
        if (!fault_report_ok) {
            visionarm::logging::Log(
                visionarm::logging::LogLevel::FATAL, "report",
                "fault report write failed: ", report_error);
            (void)visionarm::logging::FlushGlobalLogger();
        }
        visionarm::logging::ShutdownGlobalLogger();
        return EXIT_FAILURE;
    }
}
