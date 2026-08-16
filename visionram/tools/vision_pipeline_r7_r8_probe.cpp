#include "camera/capture_buffer_broker.h"
#include "camera/v4l2_camera.h"
#include "control/control_sink.h"
#if defined(VISIONARM_HAS_UART_CONTROL)
#include "control/uart_control_sink.h"
#include "uart/uart_link.h"
#endif
#include "inference/rknn_engine.h"
#include "metrics/v7_control_trace_recorder.h"
#include "pipeline/inference_pipeline.h"
#include "pipeline/latest_result_store.h"
#include "pipeline/target_state_machine.h"
#include "postprocess/yolov8_top1_postprocessor.h"
#include "preprocess/rga_letterbox_preprocessor.h"
#include "video/h265_file_sink.h"
#include "video/mpp_h265_encoder.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_stop{false};

constexpr int kModelWidth = 960;
constexpr int kModelHeight = 544;
constexpr int kClassCount = 1;
constexpr int kTargetClassId = 0;

void SignalHandler(int) {
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
    std::string model;
    std::string output;
    std::string report;
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
    std::string v7_control_csv;
    std::string v7_status_csv;
    visionarm::InferenceThreadTopology topology =
        visionarm::InferenceThreadTopology::FUSED_NPU_POSTPROCESS;
};

[[noreturn]] void Usage(const char* program) {
    std::cerr
        << "Usage: " << program << " \\\n"
        << "  --device /dev/videoX --model model.rknn --output stream.h265 \\\n"
        << "  --width W --height H --fps FPS --bitrate BPS --gop N \\\n"
        << "  [--duration-sec N] [--buffers N] [--video-queue N] \\\n"
        << "  [--topology fused|split] [--input-slots N] [--output-slots N] \\\n"
        << "  [--acquire-hits N] [--lost-misses N] \\\n"
        << "  [--max-result-age-ms N] [--latency-samples N] \\\n"
        << "  [--max-rss-growth-kb N] [--vertical-stride N] \\\n"
        << "  [--input-dma-heap PATH] [--confidence F] [--report PATH] \\\n"
        << "  [--control-backend mock|uart] [--uart-device /dev/ttyS3] \\\n"
        << "  [--uart-baud 115200] [--uart-ready-timeout-ms 5000] \\\n"
        << "  [--v7-control-csv control.csv] [--v7-status-csv status.csv]\n";
    std::exit(EXIT_FAILURE);
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

Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string key = argv[index];
        auto next = [&]() -> const char* {
            if (++index >= argc) Usage(argv[0]);
            return argv[index];
        };

        if (key == "--device") options.device = next();
        else if (key == "--model") options.model = next();
        else if (key == "--output") options.output = next();
        else if (key == "--report") options.report = next();
        else if (key == "--width") options.width = ParsePositiveInt(next(), "width");
        else if (key == "--height") options.height = ParsePositiveInt(next(), "height");
        else if (key == "--fps") options.fps = ParsePositiveInt(next(), "fps");
        else if (key == "--buffers") options.buffers = ParsePositiveInt(next(), "buffers");
        else if (key == "--duration-sec") options.duration_seconds = ParsePositiveInt(next(), "duration");
        else if (key == "--timeout-ms") options.timeout_ms = ParsePositiveInt(next(), "timeout");
        else if (key == "--bitrate") options.bitrate = ParsePositiveInt(next(), "bitrate");
        else if (key == "--gop") options.gop = ParsePositiveInt(next(), "gop");
        else if (key == "--vertical-stride") options.vertical_stride = ParsePositiveInt(next(), "vertical stride");
        else if (key == "--video-queue") options.video_queue = ParsePositiveInt(next(), "video queue");
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
        } else if (key == "--v7-control-csv") {
            options.v7_control_csv = next();
        } else if (key == "--v7-status-csv") {
            options.v7_status_csv = next();
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
    if (options.control_backend == ControlBackend::UART &&
        options.uart_device.empty()) {
        throw std::invalid_argument("UART device must not be empty");
    }
    if (!options.v7_status_csv.empty() &&
        options.control_backend != ControlBackend::UART) {
        throw std::invalid_argument(
            "--v7-status-csv requires --control-backend uart");
    }
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

void WriteV7StatusCsvHeader(std::ofstream& output) {
    output
        << "host_steady_ns,link_state,mcu_state,remote_stop_latched,"
        << "control_valid,last_rx_wire_sequence,last_control_wire_sequence,"
        << "control_mailbox_overwrite_count,mcu_tick_ms,pan_command_q15,"
        << "tilt_command_q15\n";
}

bool WriteV7StatusCsvRow(
    std::ofstream& output,
    const visionarm::uart::UartModuleSnapshot& snapshot) {
    if (!snapshot.last_status.has_value()) {
        return false;
    }

    const auto host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto& status = *snapshot.last_status;
    output << host_ns << ','
           << static_cast<unsigned int>(snapshot.state) << ','
           << static_cast<unsigned int>(status.mcu_state) << ','
           << static_cast<unsigned int>(status.remote_stop_latched) << ','
           << static_cast<unsigned int>(status.control_valid) << ','
           << status.last_rx_wire_sequence << ','
           << status.last_control_wire_sequence << ','
           << status.control_mailbox_overwrite_count << ','
           << status.mcu_tick_ms << ','
           << status.pan_stub_q15 << ','
           << status.tilt_stub_q15 << '\n';
    output.flush();
    return static_cast<bool>(output);
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

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = ParseOptions(argc, argv);
        std::signal(SIGINT, SignalHandler);
        std::signal(SIGTERM, SignalHandler);

        visionarm::V4L2CameraConfig camera_config;
        camera_config.device = options.device;
        camera_config.width = static_cast<uint32_t>(options.width);
        camera_config.height = static_cast<uint32_t>(options.height);
        camera_config.pixel_format = V4L2_PIX_FMT_NV12;
        camera_config.fps = static_cast<uint32_t>(options.fps);
        camera_config.buffer_count = static_cast<uint32_t>(options.buffers);
        camera_config.timeout_ms = options.timeout_ms;
        camera_config.export_dmabuf = true;
        camera_config.require_dmabuf_export = true;

        visionarm::V4L2Camera camera(camera_config);
        camera.Open();
        const visionarm::CameraFormat camera_format = camera.format();
        if (camera_format.pixel_format != V4L2_PIX_FMT_NV12 ||
            camera_format.plane_count != 1U ||
            camera_format.bytes_per_line.empty()) {
            throw std::runtime_error(
                "R7/R8 requires frozen single-plane linear NV12");
        }

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

        visionarm::MppH265EncoderConfig encoder_config;
        encoder_config.width = static_cast<int>(camera_format.width);
        encoder_config.height = static_cast<int>(camera_format.height);
        encoder_config.horizontal_stride =
            static_cast<int>(camera_format.bytes_per_line[0]);
        encoder_config.vertical_stride =
            DeriveVerticalStride(camera_format, options.vertical_stride);
        encoder_config.fps_numerator = options.fps;
        encoder_config.bitrate_bps = options.bitrate;
        encoder_config.gop_length = options.gop;
        encoder_config.max_source_buffers = camera.buffer_count();

        visionarm::MppH265Encoder encoder;
        encoder.Initialize(encoder_config);
        visionarm::H265FileSink file_sink(options.output);
        if (!file_sink.opened()) {
            throw std::runtime_error("failed to open output H.265 file");
        }

        visionarm::LatestResultStore latest_perception;
        visionarm::MockControlSink mock_control_sink;
        visionarm::IControlSink* selected_control_sink = &mock_control_sink;

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
            std::cout << "UART READY device=" << options.uart_device
                      << " baud=" << options.uart_baud << '\n';
        }
#endif

        std::unique_ptr<visionarm::V7ControlTraceRecorder> v7_control_trace;
        if (!options.v7_control_csv.empty()) {
            v7_control_trace =
                std::make_unique<visionarm::V7ControlTraceRecorder>(
                    *selected_control_sink, options.v7_control_csv);
            selected_control_sink = v7_control_trace.get();
            std::cout << "V7 control trace=" << options.v7_control_csv << '\n';
        }

#if defined(VISIONARM_HAS_UART_CONTROL)
        std::ofstream v7_status_trace;
        std::optional<uint32_t> v7_last_status_tick;
        if (!options.v7_status_csv.empty()) {
            v7_status_trace.open(options.v7_status_csv, std::ios::trunc);
            if (!v7_status_trace) {
                throw std::runtime_error("failed to open V7 status trace CSV");
            }
            WriteV7StatusCsvHeader(v7_status_trace);
            v7_status_trace.flush();
            if (!v7_status_trace) {
                throw std::runtime_error(
                    "failed to initialize V7 status trace CSV");
            }
            std::cout << "V7 MCU status trace=" << options.v7_status_csv << '\n';
        }
#endif

        visionarm::TargetStateMachineConfig state_config;
        state_config.acquire_hits =
            static_cast<uint32_t>(options.acquire_hits);
        state_config.lost_misses =
            static_cast<uint32_t>(options.lost_misses);
        state_config.max_result_age_ns =
            static_cast<int64_t>(options.max_result_age_ms) * 1'000'000LL;
        visionarm::TargetStateMachine state_machine(
            state_config, selected_control_sink, &latest_perception);

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
            &file_sink);

        if (!pipeline.Start()) {
            throw std::runtime_error("pipeline Start failed");
        }

        RssSamples rss;
        rss.Add(ReadVmRssKb());
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(options.duration_seconds);
        auto next_rss_sample = std::chrono::steady_clock::now() +
            std::chrono::seconds(1);
#if defined(VISIONARM_HAS_UART_CONTROL)
        auto next_v7_status_sample = std::chrono::steady_clock::now();
#endif
        while (!g_stop.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < deadline &&
               pipeline.running()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_rss_sample) {
                rss.Add(ReadVmRssKb());
                next_rss_sample += std::chrono::seconds(1);
            }
#if defined(VISIONARM_HAS_UART_CONTROL)
            if (uart_link != nullptr && v7_status_trace.is_open() &&
                now >= next_v7_status_sample) {
                const auto snapshot = uart_link->GetSnapshot();
                if (snapshot.last_status.has_value()) {
                    const uint32_t tick = snapshot.last_status->mcu_tick_ms;
                    if (!v7_last_status_tick.has_value() ||
                        tick != *v7_last_status_tick) {
                        if (!WriteV7StatusCsvRow(v7_status_trace, snapshot)) {
                            throw std::runtime_error(
                                "failed to write V7 status trace CSV");
                        }
                        v7_last_status_tick = tick;
                    }
                }
                next_v7_status_sample = now + std::chrono::milliseconds(50);
            }
#endif
        }
        pipeline.Stop();
        if (v7_control_trace != nullptr) {
            v7_control_trace->Stop();
        }
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
        std::optional<visionarm::V7ControlTraceStats> v7_trace_stats;
        if (v7_control_trace != nullptr) {
            v7_trace_stats = v7_control_trace->Snapshot();
        }

#if defined(VISIONARM_HAS_UART_CONTROL)
        std::optional<visionarm::UartControlSinkSnapshot> uart_control_stats;
        std::optional<visionarm::uart::UartModuleSnapshot> uart_link_stats;
        if (options.control_backend == ControlBackend::UART) {
            uart_control_stats = uart_control_sink->Snapshot();
            uart_link_stats = uart_link->GetSnapshot();
            uart_link->Stop();
        }
#endif

        std::ofstream report_file;
        std::ostream* output_stream = &std::cout;
        if (!options.report.empty()) {
            report_file.open(options.report, std::ios::trunc);
            if (!report_file) {
                throw std::runtime_error("failed to open report file");
            }
            output_stream = &report_file;
        }
        std::ostream& report = *output_stream;
        report << std::fixed << std::setprecision(3);
        report << "topology="
               << visionarm::InferenceThreadTopologyName(stats.topology)
               << '\n';
        report << "model_input_width=" << kModelWidth << '\n';
        report << "model_input_height=" << kModelHeight << '\n';
        report << "input_slots=" << options.input_slots << '\n';
        report << "output_slots=" << options.output_slots << '\n';
        report << "latest_frame_queue_capacity=1\n";
        report << "acquire_hits=" << options.acquire_hits << '\n';
        report << "lost_misses=" << options.lost_misses << '\n';
        report << "max_result_age_ms=" << options.max_result_age_ms << '\n';
        report << "control_backend="
               << ControlBackendName(options.control_backend) << '\n';
        report << "v7_control_csv=" << options.v7_control_csv << '\n';
        report << "v7_status_csv=" << options.v7_status_csv << '\n';
        if (options.control_backend == ControlBackend::UART) {
            report << "uart_device=" << options.uart_device << '\n';
            report << "uart_baud=" << options.uart_baud << '\n';
            report << "uart_ready_timeout_ms="
                   << options.uart_ready_timeout_ms << '\n';
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
                   << visionarm::uart::LinkStateName(link.state) << '\n';
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
        if (v7_trace_stats.has_value()) {
            const auto& trace = *v7_trace_stats;
            report << "v7.trace.submissions=" << trace.submissions << '\n';
            report << "v7.trace.downstream_accepted="
                   << trace.downstream_accepted << '\n';
            report << "v7.trace.downstream_rejected="
                   << trace.downstream_rejected << '\n';
            report << "v7.trace.enqueued=" << trace.enqueued << '\n';
            report << "v7.trace.dropped=" << trace.dropped << '\n';
            report << "v7.trace.written=" << trace.written << '\n';
            report << "v7.trace.write_errors=" << trace.write_errors << '\n';
        }

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
            stats.timing.capture_to_result.total_samples > 0U &&
            stats.timing.result_age.total_samples > 0U &&
            !stats.timing.capture_to_result.truncated &&
            !stats.timing.result_age.truncated;
        const bool queues_ok =
            QueueBounded(stats.captured_frame_queue) &&
            QueueBounded(stats.prepared_frame_queue) &&
            QueueBounded(stats.completed_frame_queue) &&
            QueueBounded(stats.video_frame_queue) &&
            QueueBounded(stats.encoded_packet_queue);

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
                link_state_ok &&
                link.peer_boot_id_valid &&
                link.metrics.hello_ack_received > 0U &&
                link.metrics.status_received > 0U &&
                link.metrics.control_sent > 0U &&
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

        const bool v7_trace_ok =
            !v7_trace_stats.has_value() ||
            (v7_trace_stats->submissions > 0U &&
             v7_trace_stats->downstream_rejected == 0U &&
             v7_trace_stats->dropped == 0U &&
             v7_trace_stats->write_errors == 0U &&
             v7_trace_stats->written == v7_trace_stats->enqueued);

        const bool passed =
            stats.captured_frames > 0U &&
            stats.video_frames_encoded > 0U &&
            stats.inference_successes > 0U &&
            stats.postprocess_successes > 0U &&
            stats.preprocess_failures == 0U &&
            stats.inference_failures == 0U &&
            stats.postprocess_failures == 0U &&
            stats.result_publish_failures == 0U &&
            stats.video_encode_failures == 0U &&
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
            queues_ok && timing_ok && rss_ok && v7_trace_ok;

        report << "vision_pipeline_r7_r8_probe="
               << (passed ? "PASS" : "FAIL") << '\n';

        encoder.Shutdown();
        engine.Shutdown();
        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
