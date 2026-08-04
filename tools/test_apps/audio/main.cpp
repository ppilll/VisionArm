#include "alsa_capture.h"

#include <csignal>
#include <cstdlib>
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

uint64_t parseUnsigned(
    const std::string& option,
    const std::string& value,
    uint64_t minimum,
    uint64_t maximum) {
    std::size_t parsed = 0;
    unsigned long long result = 0;

    try {
        result =
            std::stoull(
                value,
                &parsed,
                10);
    } catch (...) {
        throw std::invalid_argument(
            option +
            ": invalid integer: " +
            value);
    }

    if (parsed != value.size() ||
        result < minimum ||
        result > maximum) {
        throw std::invalid_argument(
            option +
            ": value out of range: " +
            value);
    }

    return static_cast<uint64_t>(result);
}

double parsePositiveDouble(
    const std::string& option,
    const std::string& value,
    bool allow_zero = false) {
    std::size_t parsed = 0;
    double result = 0.0;

    try {
        result =
            std::stod(
                value,
                &parsed);
    } catch (...) {
        throw std::invalid_argument(
            option +
            ": invalid number: " +
            value);
    }

    if (parsed != value.size() ||
        result < 0.0 ||
        (!allow_zero && result == 0.0)) {
        throw std::invalid_argument(
            option +
            ": invalid value: " +
            value);
    }

    return result;
}

void printUsage(const char* program) {
    std::cout
        << "Usage:\n"
        << "  " << program << " \\\n"
        << "    --device <hw:CARD,DEV> \\\n"
        << "    --rate <Hz> \\\n"
        << "    --channels <count> \\\n"
        << "    --format <S16_LE/...> \\\n"
        << "    --period-frames <count> \\\n"
        << "    --buffer-frames <count> \\\n"
        << "    --duration <seconds> \\\n"
        << "    [--output <path.wav/path.raw>] \\\n"
        << "    [--csv <path>] \\\n"
        << "    [--no-save] \\\n"
        << "    [--timestamp-type "
           "<monotonic/monotonic-raw>] \\\n"
        << "    [--stats-interval <seconds>]\n\n"
        << "All hardware parameters must come from "
           "actual ALSA hw: enumeration/results.\n";
}

AudioCaptureConfig parseArguments(int argc,char** argv) {
    AudioCaptureConfig config;

    bool device_set = false;
    bool rate_set = false;
    bool channels_set = false;
    bool format_set = false;
    bool period_set = false;
    bool buffer_set = false;
    bool duration_set = false;

    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];

        auto requireValue =
            [&](const std::string& name) {
                if (index + 1 >= argc) {
                    throw std::invalid_argument(
                        name +
                        " requires a value");
                }

                return std::string(
                    argv[++index]);
            };

        if (option == "--device") {
            config.device = requireValue(option);
            device_set = true;
            
        } else if (option == "--rate") {
            config.rate =static_cast<uint32_t>(
                parseUnsigned(
                    option,
                    requireValue(option),
                    1,
                    std::numeric_limits<
                        uint32_t>::max()));
            rate_set = true;

        } else if (option == "--channels") {
            config.channels =static_cast<uint32_t>(
                parseUnsigned(
                    option,
                    requireValue(option),
                    1,
                    256));
            channels_set = true;

        } else if (option == "--format") {
            config.format =requireValue(option);
            format_set = true;

        } else if (
            option == "--period-frames") {
            config.period_frames =static_cast<snd_pcm_uframes_t>(
                parseUnsigned(
                    option,
                    requireValue(option),
                    1,
                    std::numeric_limits<
                        uint32_t>::max()));
            period_set = true;

        } else if (
            option == "--buffer-frames") {
            config.buffer_frames =static_cast<snd_pcm_uframes_t>(
                parseUnsigned(
                    option,
                    requireValue(option),
                    2,
                    std::numeric_limits<
                        uint32_t>::max()));
            buffer_set = true;

        } else if (
            option == "--duration") {
            config.duration_seconds =
                parsePositiveDouble(
                    option,
                    requireValue(option));
            duration_set = true;

        } else if (
            option == "--output") {
            config.output_path =
                requireValue(option);

        } else if (
            option == "--csv") {
            config.csv_path =
                requireValue(option);

        } else if (
            option == "--no-save") {
            config.no_save = true;

        } else if (
            option == "--timestamp-type") {
            config.timestamp_type =
                requireValue(option);

        } else if (
            option == "--stats-interval") {
            config.stats_interval_seconds =
                parsePositiveDouble(
                    option,
                    requireValue(option),
                    true);

        } else if (
            option == "--help" ||
            option == "-h") {
            printUsage(argv[0]);
            std::exit(0);

        } else {
            throw std::invalid_argument(
                "Unknown option: " +
                option);
        }
    }

    if (!device_set ||
        !rate_set ||
        !channels_set ||
        !format_set ||
        !period_set ||
        !buffer_set ||
        !duration_set) {
        throw std::invalid_argument(
            "--device, --rate, --channels, "
            "--format, --period-frames, "
            "--buffer-frames and --duration "
            "are mandatory");
    }

    return config;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT,signalHandler);
        std::signal(SIGTERM,signalHandler);

        AudioCaptureConfig config =
            parseArguments(argc,argv);

        AlsaCapture capture(std::move(config));
        capture.setStopFlag(&g_stop_requested);
        capture.run();

        if (g_stop_requested != 0) {
            std::cerr
                << "Capture stopped by signal\n";
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr<< "ERROR: "<< error.what()<< '\n';
        printUsage(argv[0]);
        return 1;
    }
}
