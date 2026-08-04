#include "camera/dmabuf_cpu_sync.h"
#include "inference/rknn_engine.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Args {
    std::string model;
    std::string report;
    std::string input_dma_heap;
    std::size_t warmup = 20U;
    std::size_t iterations = 200U;
    std::size_t input_slots = 2U;
    std::size_t output_slots = 2U;
};

[[noreturn]] void Usage(const char* program) {
    std::cerr << "Usage: " << program
              << " --model model.rknn [--warmup 20] [--iterations 200] "
              << "[--input-slots 2] [--output-slots 2] [--report path] "
              << "[--input-dma-heap /dev/dma_heap/system-uncached-dma32]\n";
    throw std::invalid_argument("invalid command line");
}

Args ParseArgs(int argc, char** argv) {
    Args args;
    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];
        auto value = [&]() -> std::string {
            if (index + 1 >= argc) Usage(argv[0]);
            return argv[++index];
        };
        if (option == "--model") args.model = value();
        else if (option == "--report") args.report = value();
        else if (option == "--input-dma-heap") {
            args.input_dma_heap = value();
        }
        else if (option == "--warmup") args.warmup = std::stoul(value());
        else if (option == "--iterations") args.iterations = std::stoul(value());
        else if (option == "--input-slots") args.input_slots = std::stoul(value());
        else if (option == "--output-slots") args.output_slots = std::stoul(value());
        else Usage(argv[0]);
    }
    if (args.model.empty() || args.iterations == 0U ||
        args.input_slots == 0U || args.output_slots == 0U) {
        Usage(argv[0]);
    }
    return args;
}

struct Distribution {
    double mean_us = 0.0;
    double p50_us = 0.0;
    double p95_us = 0.0;
    double p99_us = 0.0;
    double max_us = 0.0;
};

Distribution Summarize(std::vector<int64_t> values) {
    if (values.empty()) return {};
    std::sort(values.begin(), values.end());
    const long double sum = std::accumulate(
        values.begin(), values.end(), static_cast<long double>(0));
    auto percentile = [&](double fraction) {
        const double index = fraction * static_cast<double>(values.size() - 1U);
        return static_cast<double>(
            values[static_cast<std::size_t>(std::llround(index))]) / 1000.0;
    };
    return Distribution{
        static_cast<double>(sum / static_cast<long double>(values.size())) / 1000.0,
        percentile(0.50),
        percentile(0.95),
        percentile(0.99),
        static_cast<double>(values.back()) / 1000.0,
    };
}

uint64_t HashBytes(const void* data, std::size_t bytes, uint64_t seed) {
    const auto* cursor = static_cast<const uint8_t*>(data);
    uint64_t hash = seed;
    for (std::size_t index = 0; index < bytes; ++index) {
        hash ^= cursor[index];
        hash *= 1099511628211ULL;
    }
    return hash;
}

uint64_t HashOutputs(
    const std::vector<visionarm::TensorView>& outputs) {
    uint64_t hash = 1469598103934665603ULL;
    for (const visionarm::TensorView& output : outputs) {
        hash = HashBytes(output.data, output.bytes, hash);
    }
    return hash;
}

void FillInput(const visionarm::ModelInputBufferView& input) {
    if (input.cpu_address == nullptr ||
        input.memory_layout !=
            visionarm::ModelInputMemoryLayout::RGB_UINT8_NHWC) {
        throw std::runtime_error("benchmark requires RGB UINT8 NHWC input");
    }

    visionarm::DmabufCpuAccessGuard guard;
    if (!guard.BeginFd(input.dma_fd, visionarm::DmabufCpuAccessMode::WRITE)) {
        throw std::runtime_error("DMA-BUF input CPU sync START failed");
    }

    const uint32_t row_stride = input.row_stride_bytes != 0U
        ? input.row_stride_bytes
        : static_cast<uint32_t>(input.width * input.channels);
    auto* bytes = static_cast<uint8_t*>(input.cpu_address);
    for (int y = 0; y < input.height; ++y) {
        uint8_t* row = bytes + static_cast<std::size_t>(y) * row_stride;
        for (int x = 0; x < input.width; ++x) {
            row[x * 3 + 0] = static_cast<uint8_t>((x + y) & 0xff);
            row[x * 3 + 1] = static_cast<uint8_t>((2 * x + y) & 0xff);
            row[x * 3 + 2] = static_cast<uint8_t>((x + 2 * y) & 0xff);
        }
    }
    if (!guard.End()) {
        throw std::runtime_error("DMA-BUF input CPU sync END failed");
    }
}

struct ModeResult {
    visionarm::RknnIoMode mode{};
    bool supported = false;
    bool passed = false;
    std::string error;
    uint64_t output_hash = 0U;
    Distribution submit;
    Distribution output_bind;
    Distribution run;
    Distribution output_get;
    Distribution output_release;
    Distribution total;
    std::string api_version;
    std::string driver_version;
    std::size_t input_capacity_bytes = 0U;
    uint32_t input_row_stride_bytes = 0U;
    bool native_input_query_succeeded = false;
    bool native_direct_input_supported = false;
};

ModeResult RunMode(const Args& args, visionarm::RknnIoMode mode) {
    ModeResult result;
    result.mode = mode;
    try {
        visionarm::RknnEngine engine;
        visionarm::RknnEngineConfig config;
        config.model_path = args.model;
        config.input_slot_count = args.input_slots;
        config.output_slot_count = args.output_slots;
        config.io_mode = mode;
        config.input_dma_heap_path = args.input_dma_heap;
        engine.Initialize(config);
        result.supported = true;
        result.api_version = engine.model_info().api_version;
        result.driver_version = engine.model_info().driver_version;
        result.native_input_query_succeeded =
            engine.model_info().native_input_query_succeeded;
        result.native_direct_input_supported =
            engine.native_direct_input_supported();
        const auto* first_input = engine.input_buffer(0U);
        if (first_input == nullptr) {
            throw std::runtime_error("missing first input slot");
        }
        result.input_capacity_bytes = first_input->capacity_bytes;
        result.input_row_stride_bytes = first_input->row_stride_bytes;

        for (std::size_t slot = 0; slot < engine.input_slot_count(); ++slot) {
            const auto* input = engine.input_buffer(slot);
            if (input == nullptr) throw std::runtime_error("missing input slot");
            FillInput(*input);
        }

        const std::size_t total_iterations = args.warmup + args.iterations;
        std::vector<int64_t> submit;
        std::vector<int64_t> output_bind;
        std::vector<int64_t> run;
        std::vector<int64_t> output_get;
        std::vector<int64_t> output_release;
        std::vector<int64_t> total;
        submit.reserve(args.iterations);
        output_bind.reserve(args.iterations);
        run.reserve(args.iterations);
        output_get.reserve(args.iterations);
        output_release.reserve(args.iterations);
        total.reserve(args.iterations);

        uint64_t stable_hash = 0U;
        for (std::size_t iteration = 0;
             iteration < total_iterations;
             ++iteration) {
            const std::size_t input_slot =
                iteration % engine.input_slot_count();
            const std::size_t output_slot =
                iteration % engine.output_slot_count();
            visionarm::RknnRunTiming timing;
            if (!engine.Run(input_slot, output_slot, &timing)) {
                throw std::runtime_error("RKNN Run failed");
            }

            const std::vector<int>* output_fds =
                engine.output_dma_fds(output_slot);
            const std::vector<visionarm::TensorView>* outputs =
                engine.output_views(output_slot);
            if (output_fds == nullptr || outputs == nullptr) {
                throw std::runtime_error("missing output slot metadata");
            }

            visionarm::DmabufCpuAccessGuard output_guard;
            if (mode != visionarm::RknnIoMode::INPUTS_SET_PREALLOC_OUTPUT &&
                !output_guard.BeginFds(
                    output_fds->data(), output_fds->size(),
                    visionarm::DmabufCpuAccessMode::READ)) {
                throw std::runtime_error("output DMA-BUF CPU sync failed");
            }
            const uint64_t hash = HashOutputs(*outputs);
            if (output_guard.active() && !output_guard.End()) {
                throw std::runtime_error("output DMA-BUF CPU sync END failed");
            }
            if (stable_hash == 0U) stable_hash = hash;
            if (hash != stable_hash) {
                throw std::runtime_error("non-deterministic output checksum");
            }

            if (iteration >= args.warmup) {
                submit.push_back(timing.input_submit_ns);
                output_bind.push_back(timing.output_bind_ns);
                run.push_back(timing.run_ns);
                output_get.push_back(timing.output_get_ns);
                output_release.push_back(timing.output_release_ns);
                total.push_back(timing.total_ns);
            }
        }

        result.output_hash = stable_hash;
        result.submit = Summarize(std::move(submit));
        result.output_bind = Summarize(std::move(output_bind));
        result.run = Summarize(std::move(run));
        result.output_get = Summarize(std::move(output_get));
        result.output_release = Summarize(std::move(output_release));
        result.total = Summarize(std::move(total));
        result.passed = true;
    } catch (const std::exception& error) {
        result.error = error.what();
    }
    return result;
}

void PrintDistribution(
    std::ostream& output,
    const char* name,
    const Distribution& distribution) {
    output << name << "_mean_us=" << distribution.mean_us << '\n'
           << name << "_p50_us=" << distribution.p50_us << '\n'
           << name << "_p95_us=" << distribution.p95_us << '\n'
           << name << "_p99_us=" << distribution.p99_us << '\n'
           << name << "_max_us=" << distribution.max_us << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = ParseArgs(argc, argv);
        const std::vector<visionarm::RknnIoMode> modes{
            visionarm::RknnIoMode::INPUTS_SET_PREALLOC_OUTPUT,
            visionarm::RknnIoMode::BOUND_HOST_IO,
            visionarm::RknnIoMode::BOUND_NATIVE_INPUT_LOGICAL_OUTPUT,
        };

        std::vector<ModeResult> results;
        for (const auto mode : modes) {
            results.push_back(RunMode(args, mode));
        }

        uint64_t reference_hash = 0U;
        bool overall_pass = true;
        for (ModeResult& result : results) {
            if (!result.supported || !result.passed) {
                if (result.mode != visionarm::RknnIoMode::BOUND_NATIVE_INPUT_LOGICAL_OUTPUT) {
                    overall_pass = false;
                }
                continue;
            }
            if (reference_hash == 0U) reference_hash = result.output_hash;
            if (result.output_hash != reference_hash) {
                result.passed = false;
                result.error = "output checksum differs from baseline";
                overall_pass = false;
            }
        }

        std::ostream* output = &std::cout;
        std::ofstream report;
        if (!args.report.empty()) {
            report.open(args.report);
            if (!report) throw std::runtime_error("failed to open report path");
            output = &report;
        }
        *output << std::fixed << std::setprecision(3);
        *output << "model=" << args.model << '\n'
                << "warmup=" << args.warmup << '\n'
                << "iterations=" << args.iterations << '\n'
                << "input_slots=" << args.input_slots << '\n'
                << "output_slots=" << args.output_slots << '\n'
                << "input_dma_heap="
                << (args.input_dma_heap.empty()
                        ? std::string("rknn_internal")
                        : args.input_dma_heap)
                << "\n\n";
        for (const ModeResult& result : results) {
            *output << "[mode " << visionarm::RknnIoModeName(result.mode)
                    << "]\n"
                    << "supported=" << (result.supported ? 1 : 0) << '\n'
                    << "passed=" << (result.passed ? 1 : 0) << '\n'
                    << "api_version=" << result.api_version << '\n'
                    << "driver_version=" << result.driver_version << '\n'
                    << "input_capacity_bytes="
                    << result.input_capacity_bytes << '\n'
                    << "input_row_stride_bytes="
                    << result.input_row_stride_bytes << '\n'
                    << "native_input_query_succeeded="
                    << (result.native_input_query_succeeded ? 1 : 0) << '\n'
                    << "native_direct_input_supported="
                    << (result.native_direct_input_supported ? 1 : 0) << '\n'
                    << "output_hash=" << result.output_hash << '\n';
            if (!result.error.empty()) {
                *output << "error=" << result.error << '\n';
            }
            PrintDistribution(*output, "input_submit", result.submit);
            PrintDistribution(*output, "output_bind", result.output_bind);
            PrintDistribution(*output, "run", result.run);
            PrintDistribution(*output, "output_get", result.output_get);
            PrintDistribution(*output, "output_release", result.output_release);
            PrintDistribution(*output, "total", result.total);
            *output << '\n';
        }
        *output << "rknn_io_benchmark="
                << (overall_pass ? "PASS" : "FAIL") << '\n';
        return overall_pass ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "rknn_io_benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
