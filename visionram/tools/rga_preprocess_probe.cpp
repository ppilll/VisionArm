#include "camera/dmabuf_cpu_sync.h"
#include "camera/v4l2_camera.h"
#include "common/monotonic_clock.h"
#include "inference/rknn_engine.h"
#include "preprocess/nv12_letterbox_preprocessor.h"
#include "preprocess/rga_letterbox_preprocessor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Args {
    std::string device;
    std::string model;
    std::string report;
    std::string input_dma_heap =
        "/dev/dma_heap/system-uncached-dma32";
    uint32_t width = 1280U;
    uint32_t height = 720U;
    uint32_t fps = 30U;
    uint32_t buffers = 6U;
    uint32_t frames = 100U;
    int timeout_ms = 2000;
    bool compare_cpu = true;
    double max_mean_absolute_error = 4.0;
    int max_absolute_error = 32;
};

[[noreturn]] void Usage(const char* program) {
    std::cerr
        << "Usage: " << program << " --device /dev/videoX --model model.rknn "
        << "[--width 1280] [--height 720] [--fps 30] [--buffers 6] "
        << "[--frames 100] [--timeout-ms 2000] [--report path] "
        << "[--input-dma-heap /dev/dma_heap/system-uncached-dma32] "
        << "[--no-cpu-compare] [--max-mae 4.0] [--max-abs 32]\n";
    throw std::invalid_argument("invalid command line");
}

uint32_t ParseU32(const std::string& text) {
    const unsigned long value = std::stoul(text);
    if (value > 0xffffffffUL) {
        throw std::out_of_range("integer too large");
    }
    return static_cast<uint32_t>(value);
}

Args ParseArgs(int argc, char** argv) {
    Args args;
    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];
        auto value = [&]() -> std::string {
            if (index + 1 >= argc) {
                Usage(argv[0]);
            }
            return argv[++index];
        };
        if (option == "--device") args.device = value();
        else if (option == "--model") args.model = value();
        else if (option == "--report") args.report = value();
        else if (option == "--input-dma-heap") {
            args.input_dma_heap = value();
        }
        else if (option == "--width") args.width = ParseU32(value());
        else if (option == "--height") args.height = ParseU32(value());
        else if (option == "--fps") args.fps = ParseU32(value());
        else if (option == "--buffers") args.buffers = ParseU32(value());
        else if (option == "--frames") args.frames = ParseU32(value());
        else if (option == "--timeout-ms") args.timeout_ms = std::stoi(value());
        else if (option == "--max-mae") args.max_mean_absolute_error = std::stod(value());
        else if (option == "--max-abs") args.max_absolute_error = std::stoi(value());
        else if (option == "--no-cpu-compare") args.compare_cpu = false;
        else Usage(argv[0]);
    }
    if (args.device.empty() || args.model.empty() || args.frames == 0U ||
        args.buffers == 0U) {
        Usage(argv[0]);
    }
    return args;
}

struct DiffStats {
    uint64_t samples = 0U;
    long double absolute_sum = 0.0L;
    int maximum = 0;

    void Add(const uint8_t* first, const uint8_t* second, std::size_t bytes) {
        for (std::size_t index = 0; index < bytes; ++index) {
            const int difference = std::abs(
                static_cast<int>(first[index]) -
                static_cast<int>(second[index]));
            absolute_sum += difference;
            maximum = std::max(maximum, difference);
        }
        samples += bytes;
    }

    double Mean() const {
        return samples == 0U
            ? 0.0
            : static_cast<double>(absolute_sum /
                  static_cast<long double>(samples));
    }
};

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = ParseArgs(argc, argv);

        visionarm::RknnEngine engine;
        visionarm::RknnEngineConfig engine_config;
        engine_config.model_path = args.model;
        engine_config.input_slot_count = 2U;
        engine_config.output_slot_count = 1U;
        engine_config.io_mode =
            visionarm::RknnIoMode::INPUTS_SET_PREALLOC_OUTPUT;
        engine_config.input_dma_heap_path = args.input_dma_heap;
        engine.Initialize(engine_config);

        visionarm::V4L2CameraConfig camera_config;
        camera_config.device = args.device;
        camera_config.width = args.width;
        camera_config.height = args.height;
        camera_config.pixel_format = V4L2_PIX_FMT_NV12;
        camera_config.fps = args.fps;
        camera_config.buffer_count = args.buffers;
        camera_config.timeout_ms = args.timeout_ms;
        camera_config.export_dmabuf = true;
        camera_config.require_dmabuf_export = true;

        visionarm::V4L2Camera camera(camera_config);
        camera.Open();

        visionarm::RgaLetterboxConfig rga_config;
        rga_config.model_width = static_cast<int>(engine.input_shape().width);
        rga_config.model_height = static_cast<int>(engine.input_shape().height);
        rga_config.max_source_buffers = camera.buffer_count();
        rga_config.max_destination_slots = engine.input_slot_count();
        visionarm::RgaLetterboxPreprocessor rga(rga_config);

        visionarm::Nv12LetterboxPreprocessor cpu({
            rga_config.model_width,
            rga_config.model_height,
            114U,
        });

        const std::size_t tight_bytes =
            static_cast<std::size_t>(rga_config.model_width) *
            static_cast<std::size_t>(rga_config.model_height) * 3U;
        std::vector<uint8_t> cpu_output(tight_bytes);
        visionarm::ModelInputBufferView cpu_destination;
        cpu_destination.cpu_address = cpu_output.data();
        cpu_destination.capacity_bytes = cpu_output.size();
        cpu_destination.width = rga_config.model_width;
        cpu_destination.height = rga_config.model_height;
        cpu_destination.channels = 3;
        cpu_destination.memory_layout =
            visionarm::ModelInputMemoryLayout::RGB_UINT8_NHWC;
        cpu_destination.row_stride_bytes =
            static_cast<uint32_t>(rga_config.model_width * 3);

        camera.Start();
        uint64_t processed = 0U;
        uint64_t failures = 0U;
        int64_t rga_total_ns = 0;
        DiffStats diff;

        while (processed < args.frames) {
            visionarm::CaptureFrameView frame;
            const visionarm::CaptureResult capture = camera.Capture(&frame);
            if (capture == visionarm::CaptureResult::TIMEOUT ||
                capture == visionarm::CaptureResult::WAKE ||
                capture == visionarm::CaptureResult::DROPPED) {
                continue;
            }
            if (capture != visionarm::CaptureResult::FRAME) {
                throw std::runtime_error("camera stopped during RGA probe");
            }

            const std::size_t slot =
                static_cast<std::size_t>(processed) %
                engine.input_slot_count();
            const visionarm::ModelInputBufferView* destination =
                engine.input_buffer(slot);
            bool ok = destination != nullptr;
            visionarm::PreprocessTransform rga_transform;
            const int64_t begin_ns = visionarm::MonotonicNowNs();
            if (ok) {
                ok = rga.Process(frame, *destination, &rga_transform);
            }
            rga_total_ns += visionarm::MonotonicNowNs() - begin_ns;

            if (ok && args.compare_cpu) {
                visionarm::PreprocessTransform cpu_transform;
                visionarm::DmabufCpuAccessGuard source_guard;
                ok = source_guard.Begin(
                    frame, visionarm::DmabufCpuAccessMode::READ) &&
                    cpu.Process(frame, cpu_destination, &cpu_transform) &&
                    source_guard.End();

                visionarm::DmabufCpuAccessGuard destination_guard;
                if (ok) {
                    ok = destination_guard.BeginFd(
                        destination->dma_fd,
                        visionarm::DmabufCpuAccessMode::READ);
                }
                if (ok) {
                    if (rga_transform.pad_left != cpu_transform.pad_left ||
                        rga_transform.pad_top != cpu_transform.pad_top ||
                        rga_transform.pad_right != cpu_transform.pad_right ||
                        rga_transform.pad_bottom != cpu_transform.pad_bottom) {
                        ok = false;
                    } else {
                        diff.Add(
                            static_cast<const uint8_t*>(
                                destination->cpu_address),
                            cpu_output.data(),
                            tight_bytes);
                    }
                }
                if (destination_guard.active() && !destination_guard.End()) {
                    ok = false;
                }
            }

            if (!camera.Requeue(visionarm::MakeRequeueRequest(frame))) {
                throw std::runtime_error("key-aware camera Requeue failed");
            }
            ++processed;
            if (!ok) {
                ++failures;
            }
        }

        camera.Stop();
        const double average_ms = processed == 0U
            ? 0.0
            : static_cast<double>(rga_total_ns) /
                  static_cast<double>(processed) / 1.0e6;
        const visionarm::RgaPreprocessorSnapshot snapshot = rga.snapshot();

        std::ostream* output = &std::cout;
        std::ofstream report;
        if (!args.report.empty()) {
            report.open(args.report);
            if (!report) {
                throw std::runtime_error("failed to open report path");
            }
            output = &report;
        }

        const visionarm::ModelInputBufferView* first_input =
            engine.input_buffer(0U);
        *output << "input_dma_heap=" << engine.input_dma_heap_path() << '\n'
                << "input_dma_fd="
                << (first_input != nullptr ? first_input->dma_fd : -1) << '\n'
                << "input_capacity_bytes="
                << (first_input != nullptr
                        ? first_input->capacity_bytes
                        : 0U) << '\n'
                << "processed_frames=" << processed << '\n'
                << "failed_frames=" << failures << '\n'
                << "average_rga_ms=" << average_ms << '\n'
                << "mean_absolute_error=" << diff.Mean() << '\n'
                << "max_absolute_error=" << diff.maximum << '\n'
                << "source_imports=" << snapshot.source_imports << '\n'
                << "destination_imports=" << snapshot.destination_imports << '\n';

        const bool parity_ok = !args.compare_cpu ||
            (diff.Mean() <= args.max_mean_absolute_error &&
             diff.maximum <= args.max_absolute_error);
        const bool passed = failures == 0U && parity_ok &&
            camera.outstanding_buffers() == 0U;
        *output << "rga_preprocess_probe="
                << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "rga_preprocess_probe failed: " << error.what() << '\n';
        return 1;
    }
}
