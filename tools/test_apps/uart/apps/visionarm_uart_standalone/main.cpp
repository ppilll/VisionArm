#include "uart/uart_link.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <poll.h>
#include <time.h>
#include <unistd.h>

namespace {

using visionarm::uart::ControlUpdateInput;
using visionarm::uart::LinkState;
using visionarm::uart::UartLink;
using visionarm::uart::UartLinkConfig;
using visionarm::uart::UartModuleSnapshot;

std::atomic<bool> g_stop{false};

void HandleSignal(int) noexcept {
    g_stop.store(true);
}

uint64_t MonotonicNs() noexcept {
    timespec value{};
    if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0U;
    }
    return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(value.tv_nsec);
}

bool ParseU32(const char* text, uint32_t* value) noexcept {
    if (text == nullptr || value == nullptr || *text == '\0') {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const unsigned long parsed = std::strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed > std::numeric_limits<uint32_t>::max()) {
        return false;
    }
    *value = static_cast<uint32_t>(parsed);
    return true;
}

struct Options {
    std::string device = "/dev/ttyS3";
    std::string mode = "smoke";
    std::string report_path;
    uint32_t baud = 115200U;
    uint32_t control_rate_hz = 30U;
    uint32_t heartbeat_ms = 200U;
    uint32_t duration_sec = 60U;
    uint32_t ready_timeout_sec = 8U;
};

bool IsSupportedMode(const std::string& mode) {
    return mode == "smoke" || mode == "control" ||
           mode == "invalid-control" || mode == "ping" ||
           mode == "stop-clear" || mode == "mcu-reset" ||
           mode == "link-loss" || mode == "soak" ||
           mode == "interactive";
}

void PrintUsage(const char* program) {
    std::cout
        << "VisionArm R5 Linux UART standalone\n\n"
        << "Usage:\n  " << program << " [options]\n\n"
        << "Options:\n"
        << "  --device PATH            default /dev/ttyS3\n"
        << "  --baud N                 default 115200\n"
        << "  --control-rate-hz N      default 30\n"
        << "  --heartbeat-ms N         default 200\n"
        << "  --duration-sec N         default 60\n"
        << "  --ready-timeout-sec N    default 8\n"
        << "  --mode MODE              smoke|control|invalid-control|ping|\n"
        << "                           stop-clear|mcu-reset|link-loss|soak|interactive\n"
        << "  --report FILE            write final JSON report\n"
        << "  --help\n\n"
        << "mcu-reset and link-loss are physical qualification modes.\n"
        << "They do not emulate faults; reset the MCU or disconnect/reconnect\n"
        << "the real RS-485 link while the program is running.\n";
}

bool ParseArguments(int argc, char** argv, Options* options) {
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--help") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
        if (index + 1 >= argc) {
            std::cerr << "missing value for " << arg << '\n';
            return false;
        }
        const char* value = argv[++index];
        if (arg == "--device") {
            options->device = value;
        } else if (arg == "--mode") {
            options->mode = value;
        } else if (arg == "--report") {
            options->report_path = value;
        } else if (arg == "--baud") {
            if (!ParseU32(value, &options->baud)) return false;
        } else if (arg == "--control-rate-hz") {
            if (!ParseU32(value, &options->control_rate_hz)) return false;
        } else if (arg == "--heartbeat-ms") {
            if (!ParseU32(value, &options->heartbeat_ms)) return false;
        } else if (arg == "--duration-sec") {
            if (!ParseU32(value, &options->duration_sec)) return false;
        } else if (arg == "--ready-timeout-sec") {
            if (!ParseU32(value, &options->ready_timeout_sec)) return false;
        } else {
            std::cerr << "unknown option: " << arg << '\n';
            return false;
        }
    }

    if (options->device.empty() || options->baud == 0U ||
        options->control_rate_hz == 0U || options->heartbeat_ms == 0U ||
        options->duration_sec == 0U || options->ready_timeout_sec == 0U ||
        !IsSupportedMode(options->mode)) {
        return false;
    }
    return true;
}

int16_t TriangleQ15(uint64_t frame_id, uint64_t phase) noexcept {
    constexpr int32_t amplitude = 28000;
    constexpr uint64_t period = 120U;
    const uint64_t x = (frame_id + phase) % period;
    int32_t value = 0;
    if (x < period / 2U) {
        value = -amplitude + static_cast<int32_t>(
            (2ULL * static_cast<uint64_t>(amplitude) * x) / (period / 2U));
    } else {
        const uint64_t descending = x - period / 2U;
        value = amplitude - static_cast<int32_t>(
            (2ULL * static_cast<uint64_t>(amplitude) * descending) /
            (period / 2U));
    }
    return static_cast<int16_t>(std::clamp<int32_t>(value, -32768, 32767));
}

ControlUpdateInput MakeSyntheticControl(uint64_t frame_id, bool valid) noexcept {
    ControlUpdateInput input;
    input.source_capture_session_id = 1U;
    input.source_frame_id = static_cast<uint32_t>(frame_id);
    input.source_v4l2_sequence = static_cast<uint32_t>(frame_id);
    input.target_state = valid ? 2U : 3U;  // DETECTED / LOST protocol-domain values.
    input.valid = valid;
    input.capture_monotonic_ns = MonotonicNs();

    if (valid) {
        input.error_x_q15 = TriangleQ15(frame_id, 0U);
        input.error_y_q15 = TriangleQ15(frame_id, 30U);
        input.dx_px = static_cast<int16_t>(input.error_x_q15 / 560);
        input.dy_px = static_cast<int16_t>(input.error_y_q15 / 700);
        input.confidence_u16 = 55705U;
    }
    return input;
}

void PrintSnapshotLine(const UartModuleSnapshot& snapshot) {
    const auto& m = snapshot.metrics;
    std::cout
        << "state=" << visionarm::uart::LinkStateName(snapshot.state)
        << " peer_boot=";
    if (snapshot.peer_boot_id_valid) {
        std::cout << snapshot.peer_boot_id;
    } else {
        std::cout << "unknown";
    }
    std::cout
        << " status=" << m.status_received
        << " pong=" << m.pong_received
        << " control=" << m.control_sent
        << " overwrite=" << m.control_overwritten
        << " retry=" << m.reliable_retries
        << " timeout=" << m.response_window_timeouts
        << " crc=" << m.parser.crc_errors
        << " reopen=" << m.reopen_count;
    if (snapshot.last_status.has_value()) {
        std::cout
            << " mcu_control_valid="
            << static_cast<unsigned>(snapshot.last_status->control_valid)
            << " mcu_stop="
            << static_cast<unsigned>(snapshot.last_status->remote_stop_latched);
    }
    if (!snapshot.last_error.empty()) {
        std::cout << " error=\"" << snapshot.last_error << '"';
    }
    std::cout << '\n';
}

std::string EscapeJson(const std::string& value) {
    std::ostringstream output;
    for (char c : value) {
        switch (c) {
            case '\\': output << "\\\\"; break;
            case '"': output << "\\\""; break;
            case '\n': output << "\\n"; break;
            case '\r': output << "\\r"; break;
            case '\t': output << "\\t"; break;
            default: output << c; break;
        }
    }
    return output.str();
}

bool EvaluatePass(const Options& options,
                  bool became_ready,
                  const UartModuleSnapshot& snapshot) noexcept {
    const auto& m = snapshot.metrics;
    if (!became_ready || m.hello_ack_received == 0U ||
        m.status_received == 0U) {
        return false;
    }

    if (options.mode == "smoke") {
        return m.control_sent > 0U && m.pong_received > 0U &&
               m.reliable_failures == 0U;
    }
    if (options.mode == "control") {
        return m.control_sent > 0U;
    }
    if (options.mode == "invalid-control") {
        return m.valid_control_sent > 0U && m.invalid_control_sent > 0U;
    }
    if (options.mode == "ping") {
        return m.pong_received > 0U;
    }
    if (options.mode == "stop-clear") {
        return m.reliable_acks >= 2U && m.reliable_failures == 0U;
    }
    if (options.mode == "mcu-reset") {
        return m.peer_boot_changes > 0U && m.link_recoveries > 0U;
    }
    if (options.mode == "link-loss") {
        return m.link_losses > 0U && m.link_recoveries > 0U;
    }
    if (options.mode == "soak") {
        return m.control_sent > 0U && m.reliable_failures == 0U;
    }
    return true;
}

bool WriteReport(const std::string& path,
                 const Options& options,
                 bool became_ready,
                 bool passed,
                 bool clean_stop,
                 const UartModuleSnapshot& snapshot,
                 std::string* error) {
    if (path.empty()) {
        return true;
    }

    std::error_code ec;
    const std::filesystem::path report_path(path);
    if (report_path.has_parent_path()) {
        std::filesystem::create_directories(report_path.parent_path(), ec);
        if (ec) {
            if (error != nullptr) *error = ec.message();
            return false;
        }
    }

    std::ofstream out(report_path);
    if (!out) {
        if (error != nullptr) *error = "cannot open report file";
        return false;
    }

    const auto& m = snapshot.metrics;
    out << "{\n"
        << "  \"mode\": \"" << EscapeJson(options.mode) << "\",\n"
        << "  \"device\": \"" << EscapeJson(options.device) << "\",\n"
        << "  \"baud\": " << options.baud << ",\n"
        << "  \"control_rate_hz\": " << options.control_rate_hz << ",\n"
        << "  \"heartbeat_ms\": " << options.heartbeat_ms << ",\n"
        << "  \"duration_sec\": " << options.duration_sec << ",\n"
        << "  \"became_ready\": " << (became_ready ? "true" : "false") << ",\n"
        << "  \"pass\": " << (passed ? "true" : "false") << ",\n"
        << "  \"clean_stop\": " << (clean_stop ? "true" : "false") << ",\n"
        << "  \"final_link_state\": \""
        << visionarm::uart::LinkStateName(snapshot.state) << "\",\n"
        << "  \"local_boot_id\": " << snapshot.local_boot_id << ",\n"
        << "  \"peer_boot_id_valid\": "
        << (snapshot.peer_boot_id_valid ? "true" : "false") << ",\n"
        << "  \"peer_boot_id\": " << snapshot.peer_boot_id << ",\n"
        << "  \"last_error\": \"" << EscapeJson(snapshot.last_error) << "\",\n"
        << "  \"metrics\": {\n"
        << "    \"open_attempts\": " << m.open_attempts << ",\n"
        << "    \"open_successes\": " << m.open_successes << ",\n"
        << "    \"reopen_count\": " << m.reopen_count << ",\n"
        << "    \"rx_bytes\": " << m.rx_bytes << ",\n"
        << "    \"tx_bytes\": " << m.tx_bytes << ",\n"
        << "    \"rx_valid_frames\": " << m.rx_valid_frames << ",\n"
        << "    \"tx_frames\": " << m.tx_frames << ",\n"
        << "    \"wire_duplicates\": " << m.wire_duplicates << ",\n"
        << "    \"wire_gaps\": " << m.wire_gaps << ",\n"
        << "    \"wire_old_sequences\": " << m.wire_old_sequences << ",\n"
        << "    \"control_accepted\": " << m.control_accepted << ",\n"
        << "    \"control_overwritten\": " << m.control_overwritten << ",\n"
        << "    \"control_sent\": " << m.control_sent << ",\n"
        << "    \"valid_control_sent\": " << m.valid_control_sent << ",\n"
        << "    \"invalid_control_sent\": " << m.invalid_control_sent << ",\n"
        << "    \"hello_sent\": " << m.hello_sent << ",\n"
        << "    \"hello_ack_received\": " << m.hello_ack_received << ",\n"
        << "    \"status_received\": " << m.status_received << ",\n"
        << "    \"ping_sent\": " << m.ping_sent << ",\n"
        << "    \"pong_received\": " << m.pong_received << ",\n"
        << "    \"reliable_requests\": " << m.reliable_requests << ",\n"
        << "    \"reliable_retries\": " << m.reliable_retries << ",\n"
        << "    \"reliable_acks\": " << m.reliable_acks << ",\n"
        << "    \"reliable_nacks\": " << m.reliable_nacks << ",\n"
        << "    \"reliable_timeouts\": " << m.reliable_timeouts << ",\n"
        << "    \"reliable_retry_exhausted\": " << m.reliable_retry_exhausted << ",\n"
        << "    \"response_window_timeouts\": " << m.response_window_timeouts << ",\n"
        << "    \"peer_boot_changes\": " << m.peer_boot_changes << ",\n"
        << "    \"link_losses\": " << m.link_losses << ",\n"
        << "    \"link_recoveries\": " << m.link_recoveries << ",\n"
        << "    \"parser_crc_errors\": " << m.parser.crc_errors << ",\n"
        << "    \"parser_length_errors\": " << m.parser.length_errors << ",\n"
        << "    \"parser_version_errors\": " << m.parser.version_errors << ",\n"
        << "    \"parser_escape_errors\": " << m.parser.escape_errors << ",\n"
        << "    \"parser_oversize_errors\": " << m.parser.oversize_errors << ",\n"
        << "    \"parser_timeout_errors\": " << m.parser.timeout_errors << "\n"
        << "  }\n"
        << "}\n";
    return true;
}

bool PollInteractiveCommand(std::string* command) {
    pollfd fd{};
    fd.fd = STDIN_FILENO;
    fd.events = POLLIN;
    const int result = ::poll(&fd, 1U, 0);
    if (result <= 0 || (fd.revents & POLLIN) == 0) {
        return false;
    }
    return static_cast<bool>(std::getline(std::cin, *command));
}

void PrintInteractiveHelp() {
    std::cout << "commands: status | ping | stop | clear | quit | help\n";
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseArguments(argc, argv, &options)) {
        PrintUsage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    UartLinkConfig config;
    config.device_path = options.device;
    config.baud_rate = options.baud;
    config.heartbeat_period_ms = options.heartbeat_ms;
    config.max_control_rate_hz = options.control_rate_hz;
    config.control_max_age_ms = 200U;

    UartLink link(config);
    std::string error;
    if (!link.Start(&error)) {
        std::cerr << "UART Start failed: " << error << '\n';
        return 3;
    }

    bool became_ready = false;
    const auto ready_deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(options.ready_timeout_sec);
    while (!g_stop.load() && std::chrono::steady_clock::now() < ready_deadline) {
        const auto snapshot = link.GetSnapshot();
        if (snapshot.state == LinkState::READY) {
            became_ready = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (!became_ready) {
        std::cerr << "UART link did not reach READY within "
                  << options.ready_timeout_sec << " s\n";
        const auto failed_snapshot = link.GetSnapshot();
        PrintSnapshotLine(failed_snapshot);
        link.Stop();
        const bool clean_stop = link.GetLinkState() == LinkState::CLOSED;
        std::string report_error;
        (void)WriteReport(options.report_path, options, false, false,
                          clean_stop, failed_snapshot, &report_error);
        return 4;
    }

    std::cout << "READY device=" << options.device
              << " baud=" << options.baud
              << " mode=" << options.mode << '\n';

    if (options.mode == "mcu-reset") {
        std::cout << "ACTION: physically reset the STM32 during this run.\n";
    } else if (options.mode == "link-loss") {
        std::cout << "ACTION: physically disconnect and reconnect RS-485 during this run.\n";
    } else if (options.mode == "interactive") {
        PrintInteractiveHelp();
    }

    const auto run_start = std::chrono::steady_clock::now();
    const auto run_end = run_start + std::chrono::seconds(options.duration_sec);
    const auto control_period = std::chrono::microseconds(
        1000000ULL / static_cast<uint64_t>(options.control_rate_hz));
    auto next_control = run_start;
    auto next_print = run_start;
    auto next_ping = run_start + std::chrono::seconds(1);
    bool stop_queued = false;
    bool clear_queued = false;
    uint64_t frame_id = 0U;

    while (!g_stop.load() && std::chrono::steady_clock::now() < run_end) {
        const auto now = std::chrono::steady_clock::now();

        const bool submit_control = options.mode != "ping";
        if (submit_control && now >= next_control) {
            bool valid = true;
            if (options.mode == "invalid-control") {
                const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - run_start).count();
                valid = (elapsed_ms % 3000) < 2000;
            }
            (void)link.SubmitLatestControl(MakeSyntheticControl(frame_id, valid));
            ++frame_id;
            next_control += control_period;
            if (next_control < now - control_period) {
                next_control = now + control_period;
            }
        }

        if ((options.mode == "smoke" || options.mode == "ping" ||
             options.mode == "soak" || options.mode == "mcu-reset" ||
             options.mode == "link-loss") && now >= next_ping) {
            uint32_t ping_id = 0U;
            (void)link.RequestPing(&ping_id);
            const uint32_t period_sec = options.mode == "soak" ? 5U : 2U;
            next_ping = now + std::chrono::seconds(period_sec);
        }

        if (options.mode == "stop-clear") {
            const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - run_start).count();
            const auto snapshot = link.GetSnapshot();
            if (!stop_queued && elapsed >= 1 && !snapshot.reliable_busy) {
                uint32_t id = 0U;
                stop_queued = link.RequestRemoteStop(1U, &id);
                if (stop_queued) std::cout << "REMOTE_STOP queued tx=" << id << '\n';
            }
            if (stop_queued && !clear_queued && elapsed >= 3 &&
                !snapshot.reliable_busy) {
                uint32_t id = 0U;
                clear_queued = link.RequestClearRemoteStop(&id);
                if (clear_queued) std::cout << "CLEAR_REMOTE_STOP queued tx=" << id << '\n';
            }
        }

        if (options.mode == "interactive") {
            std::string command;
            if (PollInteractiveCommand(&command)) {
                if (command == "quit") {
                    break;
                } else if (command == "status") {
                    PrintSnapshotLine(link.GetSnapshot());
                } else if (command == "ping") {
                    uint32_t id = 0U;
                    std::cout << (link.RequestPing(&id) ? "PING queued" : "PING busy")
                              << " id=" << id << '\n';
                } else if (command == "stop") {
                    uint32_t id = 0U;
                    std::cout << (link.RequestRemoteStop(1U, &id) ?
                                  "REMOTE_STOP queued" : "REMOTE_STOP busy")
                              << " id=" << id << '\n';
                } else if (command == "clear") {
                    uint32_t id = 0U;
                    std::cout << (link.RequestClearRemoteStop(&id) ?
                                  "CLEAR queued" : "CLEAR busy")
                              << " id=" << id << '\n';
                } else if (command == "help") {
                    PrintInteractiveHelp();
                } else if (!command.empty()) {
                    std::cout << "unknown command: " << command << '\n';
                }
            }
        }

        if (now >= next_print) {
            PrintSnapshotLine(link.GetSnapshot());
            next_print = now + std::chrono::seconds(1);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const UartModuleSnapshot final_snapshot = link.GetSnapshot();
    const bool passed = EvaluatePass(options, became_ready, final_snapshot);
    PrintSnapshotLine(final_snapshot);

    link.Stop();
    const bool clean_stop = link.GetLinkState() == LinkState::CLOSED;

    std::string report_error;
    if (!WriteReport(options.report_path, options, became_ready, passed,
                     clean_stop, final_snapshot, &report_error)) {
        std::cerr << "failed to write report: " << report_error << '\n';
        return 5;
    }

    std::cout << "RESULT=" << (passed ? "PASS" : "FAIL")
              << " clean_stop=" << (clean_stop ? "yes" : "no") << '\n';
    return passed && clean_stop ? 0 : 6;
}
