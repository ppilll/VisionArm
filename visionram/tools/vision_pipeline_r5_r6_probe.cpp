#include "camera/capture_buffer_broker.h"
#include "camera/v4l2_camera.h"
#include "inference/rknn_engine.h"
#include "pipeline/inference_pipeline.h"
#include "postprocess/yolov8_top1_postprocessor.h"
#include "preprocess/rga_letterbox_preprocessor.h"
#include "video/h265_file_sink.h"
#include "video/mpp_h265_encoder.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_stop{false};

constexpr int kModelWidth = 960;
constexpr int kModelHeight = 544;
constexpr int kClassCount = 1;
constexpr int kTargetClassId = 0;

void SignalHandler(int) {
    g_stop.store(true, std::memory_order_release);
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
    int duration_seconds = 60;
    int timeout_ms = 2000;
    int bitrate = 0;
    int gop = 0;
    int vertical_stride = 0;
    int input_slots = 1;
    int output_slots = 1;
    int video_queue = 2;
    float confidence = 0.25F;
    visionarm::InferenceThreadTopology topology =
        visionarm::InferenceThreadTopology::FUSED_NPU_POSTPROCESS;
};

[[noreturn]] void Usage(const char* program) {
    std::cerr
        << "Usage: " << program << " \\\n"
        << "  --device /dev/videoX --model model.rknn --output stream.h265 \\\n"
        << "  --width W --height H --fps FPS --bitrate BPS --gop N \\\n"
        << "  [--topology fused|split] [--duration-sec N] \\\n"
        << "  [--buffers N] [--video-queue N] [--input-slots N] \\\n"
        << "  [--output-slots N] [--vertical-stride N] \\\n"
        << "  [--input-dma-heap PATH] [--confidence F] [--report PATH]\n";
    std::exit(EXIT_FAILURE);
}

int ParseInt(const char* text, const char* name) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value <= 0 ||
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
            if (++index >= argc) {
                Usage(argv[0]);
            }
            return argv[index];
        };

        if (key == "--device") options.device = next();
        else if (key == "--model") options.model = next();
        else if (key == "--output") options.output = next();
        else if (key == "--report") options.report = next();
        else if (key == "--width") options.width = ParseInt(next(), "width");
        else if (key == "--height") options.height = ParseInt(next(), "height");
        else if (key == "--fps") options.fps = ParseInt(next(), "fps");
        else if (key == "--buffers") options.buffers = ParseInt(next(), "buffers");
        else if (key == "--duration-sec") options.duration_seconds = ParseInt(next(), "duration");
        else if (key == "--timeout-ms") options.timeout_ms = ParseInt(next(), "timeout");
        else if (key == "--bitrate") options.bitrate = ParseInt(next(), "bitrate");
        else if (key == "--gop") options.gop = ParseInt(next(), "gop");
        else if (key == "--vertical-stride") options.vertical_stride = ParseInt(next(), "vertical stride");
        else if (key == "--video-queue") options.video_queue = ParseInt(next(), "video queue");
        else if (key == "--input-slots") options.input_slots = ParseInt(next(), "input slots");
        else if (key == "--output-slots") options.output_slots = ParseInt(next(), "output slots");
        else if (key == "--input-dma-heap") options.input_dma_heap = next();
        else if (key == "--confidence") options.confidence = std::stof(next());
        else if (key == "--topology") {
            const std::string value = next();
            if (value == "fused") {
                options.topology = visionarm::InferenceThreadTopology::FUSED_NPU_POSTPROCESS;
            } else if (value == "split") {
                options.topology = visionarm::InferenceThreadTopology::SPLIT_NPU_POSTPROCESS;
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

    const int maximum_video_queue =
        options.buffers > 4 ? options.buffers - 4 : 1;
    if (options.video_queue > maximum_video_queue) {
        throw std::invalid_argument(
            "video queue is too large for the V4L2 pool; reserve at least "
            "two driver buffers plus one encoder and one capture buffer");
    }
    return options;
}

class BenchmarkSink final : public visionarm::IPerceptionSink {
public:
    void Publish(visionarm::PerceptionPacket packet) override {
        std::lock_guard<std::mutex> lock(mutex_);
        preprocess_ns_.push_back(
            packet.preprocess_end_ns - packet.preprocess_start_ns);
        inference_ns_.push_back(
            packet.inference_end_ns - packet.inference_start_ns);
        postprocess_ns_.push_back(
            packet.postprocess_end_ns - packet.inference_end_ns);
        capture_to_result_ns_.push_back(
            packet.postprocess_end_ns - packet.identity.capture_timestamp_ns);
    }

    struct Summary {
        std::size_t count = 0U;
        std::vector<int64_t> preprocess;
        std::vector<int64_t> inference;
        std::vector<int64_t> postprocess;
        std::vector<int64_t> capture_to_result;
    };

    [[nodiscard]] Summary Copy() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return Summary{
            capture_to_result_ns_.size(),
            preprocess_ns_, inference_ns_, postprocess_ns_,
            capture_to_result_ns_};
    }

private:
    mutable std::mutex mutex_;
    std::vector<int64_t> preprocess_ns_;
    std::vector<int64_t> inference_ns_;
    std::vector<int64_t> postprocess_ns_;
    std::vector<int64_t> capture_to_result_ns_;
};

struct Distribution {
    double mean_ms = 0.0;
    double p50_ms = 0.0;
    double p95_ms = 0.0;
    double p99_ms = 0.0;
    double max_ms = 0.0;
};

Distribution Summarize(std::vector<int64_t> samples) {
    Distribution result;
    if (samples.empty()) return result;
    std::sort(samples.begin(), samples.end());
    const long double sum = std::accumulate(
        samples.begin(), samples.end(), static_cast<long double>(0));
    auto percentile = [&samples](double p) {
        const double position = p * static_cast<double>(samples.size() - 1U);
        return static_cast<double>(
            samples[static_cast<std::size_t>(std::llround(position))]) /
            1.0e6;
    };
    result.mean_ms = static_cast<double>(sum / samples.size()) / 1.0e6;
    result.p50_ms = percentile(0.50);
    result.p95_ms = percentile(0.95);
    result.p99_ms = percentile(0.99);
    result.max_ms = static_cast<double>(samples.back()) / 1.0e6;
    return result;
}

void WriteDistribution(
    std::ostream& stream,
    const char* name,
    const Distribution& value) {
    stream << name << ".mean_ms=" << value.mean_ms << '\n'
           << name << ".p50_ms=" << value.p50_ms << '\n'
           << name << ".p95_ms=" << value.p95_ms << '\n'
           << name << ".p99_ms=" << value.p99_ms << '\n'
           << name << ".max_ms=" << value.max_ms << '\n';
}

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
    if (attr.n_dims != 4U ||
        attr.dims[0] != 1U ||
        attr.dims[1] != channels ||
        attr.dims[2] != height ||
        attr.dims[3] != width ||
        attr.fmt != RKNN_TENSOR_NCHW ||
        attr.type != RKNN_TENSOR_INT8 ||
        attr.qnt_type != RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC) {
        throw std::runtime_error(
            "RKNN output tensor " + std::to_string(tensor_index) +
            " does not match the expected INT8 NCHW YOLOv8 contract");
    }
}

void ValidateYoloV8OutputContract(
    const visionarm::RknnModelInfo& model_info) {

    if (model_info.output_attributes.size() != 9U) {
        throw std::runtime_error(
            "the optimized YOLOv8 RKNN model must expose exactly 9 outputs");
    }

    constexpr std::array<uint32_t, 3> strides{8U, 16U, 32U};
    constexpr std::array<uint32_t, 3> box_indices{0U, 3U, 6U};
    constexpr std::array<uint32_t, 3> class_indices{1U, 4U, 7U};
    constexpr std::array<uint32_t, 3> sum_indices{2U, 5U, 8U};

    for (std::size_t branch = 0U; branch < strides.size(); ++branch) {
        const uint32_t stride = strides[branch];
        const uint32_t height =
            static_cast<uint32_t>(kModelHeight) / stride;
        const uint32_t width =
            static_cast<uint32_t>(kModelWidth) / stride;

        ValidateOutputAttribute(
            model_info, box_indices[branch], 64U, height, width);
        ValidateOutputAttribute(
            model_info,
            class_indices[branch],
            static_cast<uint32_t>(kClassCount),
            height,
            width);
        ValidateOutputAttribute(
            model_info, sum_indices[branch], 1U, height, width);
    }
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
        const visionarm::CameraFormat& camera_format = camera.format();
        if (camera_format.pixel_format != V4L2_PIX_FMT_NV12 ||
            camera_format.plane_count != 1U ||
            camera_format.bytes_per_line.empty()) {
            throw std::runtime_error(
                "R5 requires frozen single-plane linear NV12");
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
                "vision_pipeline_r5_r6_probe requires an RKNN model with "
                "logical image shape width=960, height=544");
        }
        ValidateYoloV8OutputContract(engine.model_info());

        visionarm::RgaLetterboxConfig rga_config;
        rga_config.model_width = static_cast<int>(engine.input_shape().width);
        rga_config.model_height = static_cast<int>(engine.input_shape().height);
        rga_config.padding_value = 114U;
        // Production geometry is always aspect-preserving centered letterbox.
        // For 16:9 input and a 960x544 model this gives 960x540 plus 2 rows
        // of padding at the top and bottom.
        rga_config.resize_policy.stretch_matching_source_aspect_ratio = false;
        rga_config.max_source_buffers = camera.buffer_count();
        rga_config.max_destination_slots = engine.input_slot_count();

        visionarm::LetterboxGeometry expected_geometry;
        visionarm::PreprocessTransform expected_transform;
        if (!visionarm::ComputeModelResizeGeometry(
                static_cast<int>(camera_format.width),
                static_cast<int>(camera_format.height),
                rga_config.model_width,
                rga_config.model_height,
                rga_config.resize_policy,
                &expected_geometry,
                &expected_transform) ||
            !expected_transform.letterbox) {
            throw std::runtime_error(
                "failed to derive aspect-preserving centered letterbox geometry");
        }

        visionarm::RgaLetterboxPreprocessor preprocessor(rga_config);

        visionarm::YoloV8Top1PostprocessConfig postprocess_config;
        postprocess_config.decoder.model_width = rga_config.model_width;
        postprocess_config.decoder.model_height = rga_config.model_height;
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

        const int vertical_stride =
            DeriveVerticalStride(camera_format, options.vertical_stride);
        visionarm::MppH265EncoderConfig encoder_config;
        encoder_config.width = static_cast<int>(camera_format.width);
        encoder_config.height = static_cast<int>(camera_format.height);
        encoder_config.horizontal_stride =
            static_cast<int>(camera_format.bytes_per_line[0]);
        encoder_config.vertical_stride = vertical_stride;
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

        BenchmarkSink result_sink;
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

        visionarm::InferencePipeline pipeline(
            pipeline_config,
            &camera,
            &broker,
            &preprocessor,
            &engine,
            &postprocessor,
            &result_sink,
            &encoder,
            &file_sink);

        if (!pipeline.Start()) {
            throw std::runtime_error("pipeline Start failed");
        }

        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(options.duration_seconds);
        while (!g_stop.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < deadline &&
               pipeline.running()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        pipeline.Stop();

        const visionarm::PipelineStatsSnapshot stats = pipeline.stats();
        const visionarm::RgaPreprocessorSnapshot rga_stats =
            preprocessor.snapshot();
        const visionarm::CaptureBufferBrokerSnapshot broker_stats =
            broker.GetSnapshot();
        const visionarm::VideoEncoderSnapshot encoder_stats =
            encoder.snapshot();
        const visionarm::H265FileSinkSnapshot sink_stats =
            file_sink.snapshot();
        const BenchmarkSink::Summary timing = result_sink.Copy();

        std::ofstream report_file;
        std::ostream* output = &std::cout;
        if (!options.report.empty()) {
            report_file.open(options.report, std::ios::trunc);
            if (!report_file) {
                throw std::runtime_error("failed to open report file");
            }
            output = &report_file;
        }
        std::ostream& report = *output;
        report << std::fixed << std::setprecision(3);
        report << "topology="
               << visionarm::InferenceThreadTopologyName(stats.topology)
               << '\n';
        report << "model_input_width=" << rga_config.model_width << '\n';
        report << "model_input_height=" << rga_config.model_height << '\n';
        report << "preprocess_mode=aspect_preserving_centered_letterbox\n";
        report << "preprocess_resized_width="
               << expected_geometry.resized_width << '\n';
        report << "preprocess_resized_height="
               << expected_geometry.resized_height << '\n';
        report << "preprocess_pad_left=" << expected_geometry.pad_left << '\n';
        report << "preprocess_pad_top=" << expected_geometry.pad_top << '\n';
        report << "preprocess_pad_right=" << expected_geometry.pad_right << '\n';
        report << "preprocess_pad_bottom=" << expected_geometry.pad_bottom << '\n';
        report << "preprocess_scale=" << expected_geometry.scale << '\n';
        report << "decoder_class_count=" << kClassCount << '\n';
        report << "decoder_target_class_id=" << kTargetClassId << '\n';
        report << "model_output_count="
               << engine.model_info().output_attributes.size() << '\n';
        report << "captured_frames=" << stats.captured_frames << '\n';
        report << "video_queue_capacity=" << options.video_queue << '\n';
        report << "video_frames_encoded=" << stats.video_frames_encoded << '\n';
        report << "video_encode_failures=" << stats.video_encode_failures << '\n';
        report << "video_packets_enqueued=" << stats.video_packets_enqueued << '\n';
        report << "video_packets_dropped=" << stats.video_packets_dropped << '\n';
        report << "video_sink_failures=" << stats.video_sink_failures << '\n';
        report << "inference_successes=" << stats.inference_successes << '\n';
        report << "postprocess_successes=" << stats.postprocess_successes << '\n';
        report << "rga_process_calls=" << rga_stats.process_calls << '\n';
        report << "rga_process_successes="
               << rga_stats.process_successes << '\n';
        report << "rga_direct_resize_successes="
               << rga_stats.direct_resize_successes << '\n';
        report << "rga_letterbox_successes="
               << rga_stats.letterbox_successes << '\n';
        report << "rga_fill_operations="
               << rga_stats.fill_operations << '\n';
        report << "replaced_waiting_frames=" << stats.replaced_waiting_frames << '\n';
        report << "skipped_no_input_slot=" << stats.skipped_no_input_slot << '\n';
        report << "broker_outstanding_frames="
               << broker_stats.outstanding_frames << '\n';
        report << "broker_outstanding_leases="
               << broker_stats.outstanding_leases << '\n';
        report << "camera_outstanding_buffers="
               << camera.outstanding_buffers() << '\n';
        report << "mpp_submitted_frames="
               << encoder_stats.submitted_frames << '\n';
        report << "mpp_encoded_frames="
               << encoder_stats.encoded_frames << '\n';
        report << "mpp_encode_failures="
               << encoder_stats.encode_failures << '\n';
        report << "mpp_emitted_packets="
               << encoder_stats.emitted_packets << '\n';
        report << "mpp_emitted_bytes="
               << encoder_stats.emitted_bytes << '\n';
        report << "mpp_imported_source_buffers="
               << encoder_stats.imported_source_buffers << '\n';
        report << "mpp_source_reimports="
               << encoder_stats.source_buffer_reimports << '\n';
        report << "mpp_codec_config_packets="
               << encoder_stats.codec_config_packets << '\n';
        report << "h265_packets_written=" << sink_stats.packets_written << '\n';
        report << "h265_bytes_written=" << sink_stats.bytes_written << '\n';
        report << "h265_write_failures=" << sink_stats.write_failures << '\n';
        report << "perception_results=" << timing.count << '\n';
        report << "requeue_failures=" << stats.requeue_failures << '\n';
        WriteDistribution(report, "preprocess", Summarize(timing.preprocess));
        WriteDistribution(report, "inference", Summarize(timing.inference));
        WriteDistribution(report, "postprocess", Summarize(timing.postprocess));
        WriteDistribution(
            report, "capture_to_result",
            Summarize(timing.capture_to_result));

        const bool passed =
            stats.captured_frames > 0U &&
            stats.video_frames_encoded > 0U &&
            stats.inference_successes > 0U &&
            stats.postprocess_successes > 0U &&
            stats.video_encode_failures == 0U &&
            stats.video_packets_dropped == 0U &&
            stats.video_sink_failures == 0U &&
            stats.requeue_failures == 0U &&
            encoder_stats.encode_failures == 0U &&
            encoder_stats.source_buffer_reimports == 0U &&
            sink_stats.write_failures == 0U &&
            broker_stats.outstanding_frames == 0U &&
            broker_stats.outstanding_leases == 0U &&
            camera.outstanding_buffers() == 0U &&
            timing.count > 0U &&
            sink_stats.bytes_written > 0U;
        report << "vision_pipeline_r5_r6_probe="
               << (passed ? "PASS" : "FAIL") << '\n';

        encoder.Shutdown();
        engine.Shutdown();
        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
