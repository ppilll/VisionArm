#include "v4l2_capture.h"

#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

volatile sig_atomic_t g_stop_requested = 0;

void signalHandler(int) {
    g_stop_requested = 1;
}

void printUsage(const char* program) {
    std::cout
        << "Usage:\n"
        << "  " << program << " \\\n"
        << "    --device <path> \\\n"
        << "    --width <value> \\\n"
        << "    --height <value> \\\n"
        << "    --format <FOURCC> \\\n"
        << "    --fps <value> \\\n"
        << "    [--buffers <count>] \\\n"
        << "    [--frames <count>] \\\n"
        << "    [--timeout-ms <value>] \\\n"
        << "    [--output-dir <path>] \\\n"
        << "    [--save-first <count>] \\\n"
        << "    [--csv <path>] \\\n"
        << "    [--nonblock]\n\n"
        << "Hardware-dependent arguments are mandatory and must "
        << "come from V4L2 enumeration results.\n";
}

uint64_t parseUnsigned(const std::string& option,
                       const std::string& value,
                       uint64_t minimum,
                       uint64_t maximum) {
    std::size_t parsed = 0;
    unsigned long long result = 0;

    try {
        result = std::stoull(value, &parsed, 10);
    } catch (const std::exception&) {
        throw std::invalid_argument(
            option + ": invalid integer: " + value);
    }

    if (parsed != value.size() ||
        result < minimum ||
        result > maximum) {
        throw std::invalid_argument(
            option + ": value out of range: " + value);
    }

    return static_cast<uint64_t>(result);
}

CaptureConfig parseArguments(int argc, char** argv) {
    CaptureConfig config;

    bool device_set = false;
    bool width_set = false;
    bool height_set = false;
    bool format_set = false;
    bool fps_set = false;

    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];

        auto requireValue =
            [&](const std::string& name) -> std::string {
                if (index + 1 >= argc) {
                    throw std::invalid_argument(
                        name + " requires a value");
                }
                return argv[++index];
            };

        if (option == "--device") {
            config.device = requireValue(option);
            device_set = true;
        } else if (option == "--width") {
            config.width = static_cast<uint32_t>(
                parseUnsigned(
                    option,
                    requireValue(option),
                    1,
                    std::numeric_limits<uint32_t>::max()));
            width_set = true;
        } else if (option == "--height") {
            config.height = static_cast<uint32_t>(
                parseUnsigned(
                    option,
                    requireValue(option),
                    1,
                    std::numeric_limits<uint32_t>::max()));
            height_set = true;
        } else if (option == "--format") {
            config.fourcc = requireValue(option);
            if (config.fourcc.size() != 4) {
                throw std::invalid_argument(
                    "--format must contain exactly four characters");
            }
            format_set = true;
        } else if (option == "--fps") {
            config.fps = static_cast<uint32_t>(
                parseUnsigned(option,
                              requireValue(option),
                              1,
                              10000));
            fps_set = true;
        } else if (option == "--buffers") {
            config.buffer_count = static_cast<uint32_t>(
                parseUnsigned(option,
                              requireValue(option),
                              2,
                              VIDEO_MAX_FRAME));
        } else if (option == "--frames") {
            config.frame_count =
                parseUnsigned(option,
                              requireValue(option),
                              1,
                              std::numeric_limits<uint64_t>::max());
        } else if (option == "--timeout-ms") {
            config.timeout_ms = static_cast<int>(
                parseUnsigned(option,
                              requireValue(option),
                              1,
                              600000));
        } else if (option == "--output-dir") {
            config.output_dir = requireValue(option);
        } else if (option == "--save-first") {
            config.save_first = static_cast<uint32_t>(
                parseUnsigned(option,
                              requireValue(option),
                              0,
                              1000));
        } else if (option == "--csv") {
            config.csv_path = requireValue(option);
        } else if (option == "--nonblock") {
            config.nonblock = true;
        } else if (option == "--help" ||
                   option == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            throw std::invalid_argument(
                "Unknown option: " + option);
        }
    }

    if (!device_set ||
        !width_set ||
        !height_set ||
        !format_set ||
        !fps_set) {
        throw std::invalid_argument(
            "--device, --width, --height, --format and "
            "--fps are mandatory");
    }

    return config;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        CaptureConfig config =
            parseArguments(argc, argv);

        V4L2Capture capture(std::move(config));
        capture.setStopFlag(&g_stop_requested);
        capture.run();

        if (g_stop_requested != 0) {
            std::cerr
                << "Capture stopped by signal\n";
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        printUsage(argv[0]);
        return 1;
    }
}