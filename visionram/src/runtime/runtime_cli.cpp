#include "runtime/runtime_app.h"

#include "camera/v4l2_sensor_controller.h"
#if defined(VISIONARM_HAS_AUDIO_ENCODE)
#include "media/ffmpeg_log_control.h"
#endif

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace visionarm::runtime {

const char* ControlBackendName(ControlBackend backend) noexcept {
    switch (backend) {
        case ControlBackend::MOCK: return "mock";
        case ControlBackend::UART: return "uart";
    }
    return "unknown";
}

[[nodiscard]] std::int64_t EffectiveNetworkRateBps(
    const RuntimeOptions& options) noexcept {
    if (options.network_rate_bps > 0) {
        return options.network_rate_bps;
    }
    const std::int64_t encoded_rate =
        static_cast<std::int64_t>(options.bitrate) + options.audio_bitrate;
    return (encoded_rate * 6LL + 4LL) / 5LL;
}

[[nodiscard]] std::int64_t EffectiveNetworkBurstBits(
    const RuntimeOptions& options) noexcept {
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
    // High-frame-rate product mode. The sensor uses the
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

void ValidateCameraMode(const RuntimeOptions& options) {
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

RuntimeOptions ParseRuntimeOptions(int argc, char** argv) {
    RuntimeOptions options;
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
                "audio device must be an hw: PCM");
        }
        if (options.audio_rate != 48'000) {
            throw std::invalid_argument(
                "audio path requires 48000 Hz");
        }
        if (options.audio_channels != 2) {
            throw std::invalid_argument(
                "audio path requires 2 capture channels");
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
            "--av-output requires the audio path to be enabled");
    }
    if (!options.network_url.empty() && !options.audio_enabled) {
        throw std::invalid_argument(
            "--network-url requires the audio path to be enabled");
    }
    if (!options.network_url.empty() &&
        options.network_url.rfind("udp://", 0U) != 0U) {
        throw std::invalid_argument(
            "network URL must start with udp://");
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
    const RuntimeOptions& options,
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

void ReloadLoggerFromSupervisor(const RuntimeOptions& options) noexcept {
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
    int argc, char** argv, const std::string& option) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (argv[index] != nullptr && option == argv[index] &&
            argv[index + 1] != nullptr) {
            return argv[index + 1];
        }
    }
    return {};
}


}  // namespace visionarm::runtime
