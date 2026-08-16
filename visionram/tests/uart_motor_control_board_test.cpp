#include "uart/uart_link.h"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <time.h>

namespace {

using visionarm::uart::ControlUpdateInput;
using visionarm::uart::LinkState;
using visionarm::uart::UartLink;
using visionarm::uart::UartLinkConfig;
using visionarm::uart::UartModuleSnapshot;

std::atomic<bool> g_stop{false};

void SignalHandler(int) noexcept {
    g_stop.store(true, std::memory_order_relaxed);
}

struct Options {
    std::string device = "/dev/ttyS3";
    uint32_t baud = 115200U;
    uint32_t control_rate_hz = 20U;
    uint64_t heartbeat_ms = 200U;
    uint64_t ready_timeout_ms = 5000U;
    uint64_t link_loss_silence_ms = 1500U;

    int16_t dx_px = 30;
    int16_t dy_px = -20;
    int16_t error_x_q15 = 2048;
    int16_t error_y_q15 = -2048;
    uint16_t confidence_u16 = 60000U;

    std::string report_path;
};

struct Results {
    bool hello_heartbeat = false;
    bool valid_control = false;
    bool invalid_control = false;
    bool invalid_recovery = false;
    bool freshness = false;
    bool remote_stop = false;
    bool clear_stop_hold = false;
    bool clear_stop_recovery = false;
    bool link_loss_silence = false;
    bool link_loss_reconnect_safe = false;
    bool clean_stop = false;
};

void PrintUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n\n"
        << "Real RK3588 -> RS-485 -> MCU motor-control qualification test.\n"
        << "This executable uses the frozen production UartLink; it is not a simulator.\n\n"
        << "Options:\n"
        << "  --device PATH                 UART device (default /dev/ttyS3)\n"
        << "  --baud N                      UART baud (default 115200)\n"
        << "  --control-rate-hz N           Valid-control stream rate, 1..30 (default 20)\n"
        << "  --heartbeat-ms N              Heartbeat period (default 200)\n"
        << "  --ready-timeout-ms N          READY timeout (default 5000)\n"
        << "  --link-loss-silence-ms N      Host silence for MCU watchdog (default 1500)\n"
        << "  --dx-px N                     Valid control dx (default 30)\n"
        << "  --dy-px N                     Valid control dy (default -20)\n"
        << "  --error-x-q15 N               Valid control X error (default 2048)\n"
        << "  --error-y-q15 N               Valid control Y error (default -2048)\n"
        << "  --confidence-u16 N            Valid confidence (default 60000)\n"
        << "  --report FILE                 Write final text summary\n"
        << "  --help                        Show this help\n\n"
        << "Safety: run with the gimbal mechanically secured and conservative motor/current limits.\n";
}

bool ParseUnsigned(const std::string& text, uint64_t maximum, uint64_t* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' || parsed > maximum) {
        return false;
    }
    *value = static_cast<uint64_t>(parsed);
    return true;
}

bool ParseSigned(const std::string& text, int64_t minimum, int64_t maximum, int64_t* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const long long parsed = std::strtoll(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        parsed < minimum || parsed > maximum) {
        return false;
    }
    *value = static_cast<int64_t>(parsed);
    return true;
}

bool ParseOptions(int argc, char** argv, Options* options) {
    if (options == nullptr) {
        return false;
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
        if (i + 1 >= argc) {
            std::cerr << "missing value for " << arg << '\n';
            return false;
        }
        const std::string value = argv[++i];
        uint64_t u64 = 0U;
        int64_t i64 = 0;

        if (arg == "--device") {
            options->device = value;
        } else if (arg == "--baud") {
            if (!ParseUnsigned(value, std::numeric_limits<uint32_t>::max(), &u64)) {
                return false;
            }
            options->baud = static_cast<uint32_t>(u64);
        } else if (arg == "--control-rate-hz") {
            if (!ParseUnsigned(value, 30U, &u64) || u64 == 0U) {
                return false;
            }
            options->control_rate_hz = static_cast<uint32_t>(u64);
        } else if (arg == "--heartbeat-ms") {
            if (!ParseUnsigned(value, 10000U, &u64) || u64 == 0U) {
                return false;
            }
            options->heartbeat_ms = u64;
        } else if (arg == "--ready-timeout-ms") {
            if (!ParseUnsigned(value, 60000U, &u64) || u64 == 0U) {
                return false;
            }
            options->ready_timeout_ms = u64;
        } else if (arg == "--link-loss-silence-ms") {
            if (!ParseUnsigned(value, 60000U, &u64) || u64 < 1200U) {
                std::cerr << "--link-loss-silence-ms must be >= 1200 ms\n";
                return false;
            }
            options->link_loss_silence_ms = u64;
        } else if (arg == "--dx-px") {
            if (!ParseSigned(value, std::numeric_limits<int16_t>::min(),
                             std::numeric_limits<int16_t>::max(), &i64)) {
                return false;
            }
            options->dx_px = static_cast<int16_t>(i64);
        } else if (arg == "--dy-px") {
            if (!ParseSigned(value, std::numeric_limits<int16_t>::min(),
                             std::numeric_limits<int16_t>::max(), &i64)) {
                return false;
            }
            options->dy_px = static_cast<int16_t>(i64);
        } else if (arg == "--error-x-q15") {
            if (!ParseSigned(value, std::numeric_limits<int16_t>::min(),
                             std::numeric_limits<int16_t>::max(), &i64)) {
                return false;
            }
            options->error_x_q15 = static_cast<int16_t>(i64);
        } else if (arg == "--error-y-q15") {
            if (!ParseSigned(value, std::numeric_limits<int16_t>::min(),
                             std::numeric_limits<int16_t>::max(), &i64)) {
                return false;
            }
            options->error_y_q15 = static_cast<int16_t>(i64);
        } else if (arg == "--confidence-u16") {
            if (!ParseUnsigned(value, std::numeric_limits<uint16_t>::max(), &u64)) {
                return false;
            }
            options->confidence_u16 = static_cast<uint16_t>(u64);
        } else if (arg == "--report") {
            options->report_path = value;
        } else {
            std::cerr << "unknown option: " << arg << '\n';
            return false;
        }
    }

    if (options->device.empty() || options->baud == 0U) {
        return false;
    }
    return true;
}

uint64_t MonotonicNs() noexcept {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

std::string NowLabel() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%H:%M:%S");
    return out.str();
}

void Phase(const std::string& text) {
    std::cout << "\n[" << NowLabel() << "] === " << text << " ===\n";
}

void PrintSnapshot(const std::string& label, const UartModuleSnapshot& snapshot) {
    const auto& m = snapshot.metrics;
    std::cout << "[" << NowLabel() << "] " << label
              << " linux_state=" << visionarm::uart::LinkStateName(snapshot.state)
              << " hello=" << m.hello_sent << '/' << m.hello_ack_received
              << " heartbeat=" << m.heartbeat_sent
              << " status=" << m.status_received
              << " control=" << m.control_sent
              << " valid_tx=" << m.valid_control_sent
              << " invalid_tx=" << m.invalid_control_sent
              << " age_invalid=" << m.control_invalidated_age
              << " ack=" << m.reliable_acks
              << " nack=" << m.reliable_nacks
              << " retries=" << m.reliable_retries
              << " link_loss=" << m.link_losses
              << " recoveries=" << m.link_recoveries;

    if (snapshot.last_status.has_value()) {
        const auto& status = *snapshot.last_status;
        std::cout << " mcu_link=" << static_cast<unsigned>(status.link_state)
                  << " stop=" << static_cast<unsigned>(status.remote_stop_latched)
                  << " control_valid=" << static_cast<unsigned>(status.control_valid)
                  << " pan_q15=" << status.pan_stub_q15
                  << " tilt_q15=" << status.tilt_stub_q15
                  << " last_rx_seq=" << status.last_rx_wire_sequence
                  << " last_control_seq=" << status.last_control_wire_sequence;
    } else {
        std::cout << " status_payload=none";
    }

    if (!snapshot.last_error.empty()) {
        std::cout << " error=\"" << snapshot.last_error << '"';
    }
    std::cout << '\n';
}

bool WaitFor(const std::string& description,
             UartLink& link,
             uint64_t timeout_ms,
             const std::function<bool(const UartModuleSnapshot&)>& predicate) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    auto next_print = std::chrono::steady_clock::now();
    while (!g_stop.load(std::memory_order_relaxed) &&
           std::chrono::steady_clock::now() < deadline) {
        const auto snapshot = link.GetSnapshot();
        if (predicate(snapshot)) {
            PrintSnapshot("PASS " + description, snapshot);
            return true;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_print) {
            PrintSnapshot("WAIT " + description, snapshot);
            next_print = now + std::chrono::milliseconds(500);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PrintSnapshot("FAIL " + description, link.GetSnapshot());
    return false;
}

ControlUpdateInput MakeValidControl(const Options& options, uint32_t frame_id) {
    ControlUpdateInput input;
    input.source_capture_session_id = 1U;
    input.source_frame_id = frame_id;
    input.source_v4l2_sequence = frame_id;
    input.target_state = 2U;  // DETECTED in the frozen V5/R5 policy.
    input.valid = true;
    input.dx_px = options.dx_px;
    input.dy_px = options.dy_px;
    input.error_x_q15 = options.error_x_q15;
    input.error_y_q15 = options.error_y_q15;
    input.confidence_u16 = options.confidence_u16;
    input.capture_monotonic_ns = MonotonicNs();
    return input;
}

ControlUpdateInput MakeInvalidControl(uint32_t frame_id) {
    ControlUpdateInput input;
    input.source_capture_session_id = 1U;
    input.source_frame_id = frame_id;
    input.source_v4l2_sequence = frame_id;
    input.target_state = 3U;  // LOST, therefore not DETECTED.
    input.valid = false;
    input.capture_monotonic_ns = MonotonicNs();
    return input;
}

bool RunValidStream(UartLink& link,
                    const Options& options,
                    uint64_t duration_ms,
                    uint32_t* frame_id) {
    if (frame_id == nullptr) {
        return false;
    }
    const auto period = std::chrono::microseconds(
        1000000ULL / static_cast<uint64_t>(options.control_rate_hz));
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(duration_ms);
    auto next = std::chrono::steady_clock::now();
    bool all_accepted = true;

    while (!g_stop.load(std::memory_order_relaxed) &&
           std::chrono::steady_clock::now() < end) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next) {
            const bool accepted =
                link.SubmitLatestControl(MakeValidControl(options, *frame_id));
            all_accepted = all_accepted && accepted;
            ++(*frame_id);
            next += period;
            if (next < now - period) {
                next = now + period;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return all_accepted && !g_stop.load(std::memory_order_relaxed);
}

bool SubmitFreshValid(UartLink& link, const Options& options, uint32_t* frame_id) {
    if (frame_id == nullptr) {
        return false;
    }
    const bool accepted =
        link.SubmitLatestControl(MakeValidControl(options, *frame_id));
    ++(*frame_id);
    return accepted;
}

bool StartLink(const UartLinkConfig& config,
               uint64_t ready_timeout_ms,
               std::unique_ptr<UartLink>* link) {
    if (link == nullptr) {
        return false;
    }
    auto candidate = std::make_unique<UartLink>(config);
    std::string error;
    if (!candidate->Start(&error)) {
        std::cerr << "UartLink::Start failed: " << error << '\n';
        return false;
    }

    if (!WaitFor("HELLO -> HELLO_ACK / READY",
                 *candidate,
                 ready_timeout_ms,
                 [](const UartModuleSnapshot& snapshot) {
                     return snapshot.state == LinkState::READY &&
                            snapshot.metrics.hello_ack_received > 0U;
                 })) {
        candidate->Stop();
        return false;
    }

    if (!WaitFor("HEARTBEAT -> STATUS",
                 *candidate,
                 2000U,
                 [](const UartModuleSnapshot& snapshot) {
                     return snapshot.metrics.heartbeat_sent > 0U &&
                            snapshot.metrics.status_received > 0U &&
                            snapshot.last_status.has_value();
                 })) {
        candidate->Stop();
        return false;
    }

    *link = std::move(candidate);
    return true;
}

bool WaitControlValid(UartLink& link, bool expected, uint64_t timeout_ms) {
    return WaitFor(expected ? "MCU control_valid=true" : "MCU control_valid=false",
                   link,
                   timeout_ms,
                   [expected](const UartModuleSnapshot& snapshot) {
                       return snapshot.last_status.has_value() &&
                              (snapshot.last_status->control_valid != 0U) == expected;
                   });
}

bool WaitRemoteStop(UartLink& link, bool expected, uint64_t timeout_ms) {
    return WaitFor(expected ? "MCU remote_stop_latched=true"
                            : "MCU remote_stop_latched=false",
                   link,
                   timeout_ms,
                   [expected](const UartModuleSnapshot& snapshot) {
                       return snapshot.last_status.has_value() &&
                              (snapshot.last_status->remote_stop_latched != 0U) == expected;
                   });
}

bool WaitReliableAckAfter(UartLink& link, uint64_t baseline, uint64_t timeout_ms) {
    return WaitFor("reliable ACK",
                   link,
                   timeout_ms,
                   [baseline](const UartModuleSnapshot& snapshot) {
                       return snapshot.metrics.reliable_acks > baseline &&
                              !snapshot.reliable_busy;
                   });
}

bool WriteReport(const Options& options, const Results& r) {
    if (options.report_path.empty()) {
        return true;
    }
    std::ofstream out(options.report_path);
    if (!out) {
        std::cerr << "cannot open report: " << options.report_path << '\n';
        return false;
    }
    out << "uart_motor_control_board_test\n"
        << "device=" << options.device << '\n'
        << "baud=" << options.baud << '\n'
        << "control_rate_hz=" << options.control_rate_hz << '\n'
        << "heartbeat_ms=" << options.heartbeat_ms << '\n'
        << "link_loss_silence_ms=" << options.link_loss_silence_ms << '\n'
        << "hello_heartbeat=" << r.hello_heartbeat << '\n'
        << "valid_control=" << r.valid_control << '\n'
        << "invalid_control=" << r.invalid_control << '\n'
        << "invalid_recovery=" << r.invalid_recovery << '\n'
        << "freshness=" << r.freshness << '\n'
        << "remote_stop=" << r.remote_stop << '\n'
        << "clear_stop_hold=" << r.clear_stop_hold << '\n'
        << "clear_stop_recovery=" << r.clear_stop_recovery << '\n'
        << "link_loss_silence=" << r.link_loss_silence << '\n'
        << "link_loss_reconnect_safe=" << r.link_loss_reconnect_safe << '\n'
        << "clean_stop=" << r.clean_stop << '\n';
    return true;
}

bool AllRequiredPass(const Results& r) noexcept {
    return r.hello_heartbeat && r.valid_control && r.invalid_control &&
           r.invalid_recovery && r.freshness && r.remote_stop &&
           r.clear_stop_hold && r.clear_stop_recovery &&
           r.link_loss_silence && r.link_loss_reconnect_safe && r.clean_stop;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, &options)) {
        PrintUsage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "RK3588 real UART motor-control qualification\n"
              << "device=" << options.device
              << " baud=" << options.baud
              << " heartbeat_ms=" << options.heartbeat_ms
              << " control_rate_hz=" << options.control_rate_hz << '\n'
              << "valid command: dx=" << options.dx_px
              << " dy=" << options.dy_px
              << " error_x_q15=" << options.error_x_q15
              << " error_y_q15=" << options.error_y_q15
              << " confidence=" << options.confidence_u16 << '\n';

    UartLinkConfig config;
    config.device_path = options.device;
    config.baud_rate = options.baud;
    config.heartbeat_period_ms = options.heartbeat_ms;
    config.control_max_age_ms = 200U;
    config.link_watchdog_timeout_ms = 1000U;
    config.max_control_rate_hz = options.control_rate_hz;

    Results results;
    std::unique_ptr<UartLink> link;
    uint32_t frame_id = 1U;

    Phase("1. HELLO / HELLO_ACK + continuous HEARTBEAT");
    if (!StartLink(config, options.ready_timeout_ms, &link)) {
        std::cerr << "Phase 1 failed; aborting.\n";
        (void)WriteReport(options, results);
        return 3;
    }
    results.hello_heartbeat = true;

    Phase("2. Legal CONTROL_UPDATE stream (<200 ms interval)");
    if (!RunValidStream(*link, options, 2000U, &frame_id)) {
        std::cerr << "valid control submissions were rejected\n";
    }
    results.valid_control = WaitControlValid(*link, true, 1500U);

    Phase("3. One business-invalid CONTROL_UPDATE, then HEARTBEAT only");
    const bool invalid_accepted =
        link->SubmitLatestControl(MakeInvalidControl(frame_id++));
    if (!invalid_accepted) {
        std::cerr << "invalid control was not accepted into latest mailbox\n";
    }
    results.invalid_control = invalid_accepted && WaitControlValid(*link, false, 1500U);

    std::cout << "No CONTROL_UPDATE is sent for 400 ms; HEARTBEAT remains automatic.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    std::cout << "Send one new legal CONTROL_UPDATE generation.\n";
    const bool recovery_submit = SubmitFreshValid(*link, options, &frame_id);
    results.invalid_recovery = recovery_submit && WaitControlValid(*link, true, 1500U);

    Phase("4. Control freshness watchdog: HEARTBEAT continues, CONTROL_UPDATE stops");
    (void)RunValidStream(*link, options, 1200U, &frame_id);
    if (!WaitControlValid(*link, true, 1000U)) {
        std::cerr << "could not establish valid control before freshness test\n";
    }
    std::cout << "CONTROL_UPDATE stopped now; waiting >200 ms while HEARTBEAT continues.\n";
    results.freshness = WaitControlValid(*link, false, 1500U);

    Phase("5. REMOTE_STOP / CLEAR_REMOTE_STOP / fresh generation gating");
    (void)RunValidStream(*link, options, 1200U, &frame_id);
    if (!WaitControlValid(*link, true, 1000U)) {
        std::cerr << "could not establish valid control before REMOTE_STOP\n";
    }

    std::cout << "CONTROL_UPDATE stream paused before REMOTE_STOP.\n";
    const uint64_t stop_ack_baseline = link->GetMetrics().reliable_acks;
    uint32_t stop_transaction_id = 0U;
    const bool stop_queued = link->RequestRemoteStop(1U, &stop_transaction_id);
    std::cout << "REMOTE_STOP_REQUEST queued=" << (stop_queued ? "yes" : "no")
              << " transaction_id=" << stop_transaction_id << '\n';
    const bool stop_ack = stop_queued &&
                          WaitReliableAckAfter(*link, stop_ack_baseline, 2500U);
    const bool stop_latched = WaitRemoteStop(*link, true, 2000U);
    const bool stop_invalid = WaitControlValid(*link, false, 1000U);
    results.remote_stop = stop_ack && stop_latched && stop_invalid;

    const uint64_t clear_ack_baseline = link->GetMetrics().reliable_acks;
    uint32_t clear_transaction_id = 0U;
    const bool clear_queued = link->RequestClearRemoteStop(&clear_transaction_id);
    std::cout << "CLEAR_REMOTE_STOP queued=" << (clear_queued ? "yes" : "no")
              << " transaction_id=" << clear_transaction_id << '\n';
    const bool clear_ack = clear_queued &&
                           WaitReliableAckAfter(*link, clear_ack_baseline, 2500U);
    const bool clear_unlatched = WaitRemoteStop(*link, false, 2000U);

    std::cout << "After CLEAR, only HEARTBEAT is allowed for 500 ms.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const bool still_invalid_after_clear = WaitControlValid(*link, false, 700U);
    results.clear_stop_hold = clear_ack && clear_unlatched && still_invalid_after_clear;

    std::cout << "Send one fresh legal generation after CLEAR.\n";
    const bool fresh_after_clear = SubmitFreshValid(*link, options, &frame_id);
    results.clear_stop_recovery =
        fresh_after_clear && WaitControlValid(*link, true, 1500U);

    Phase("6. Link watchdog: valid stream -> stop ALL host traffic");
    (void)RunValidStream(*link, options, 1200U, &frame_id);
    if (!WaitControlValid(*link, true, 1000U)) {
        std::cerr << "could not establish valid control before link-loss phase\n";
    }

    PrintSnapshot("before host silence", link->GetSnapshot());
    std::cout
        << "Stopping UartLink now. This stops HEARTBEAT, CONTROL_UPDATE, PING and all legal host traffic.\n"
        << "MCU must expire its ~1000 ms link watchdog during the silent window.\n"
        << "OBSERVE MCU/logic analyzer now: link -> LOST, control_valid=false, motor/mailbox safe.\n";

    link->Stop();
    const bool first_closed = link->GetLinkState() == LinkState::CLOSED;
    link.reset();
    const auto silence_begin = std::chrono::steady_clock::now();
    while (!g_stop.load(std::memory_order_relaxed) &&
           std::chrono::steady_clock::now() - silence_begin <
               std::chrono::milliseconds(options.link_loss_silence_ms)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    results.link_loss_silence = first_closed &&
        !g_stop.load(std::memory_order_relaxed);

    std::cout << "Host silence complete (" << options.link_loss_silence_ms
              << " ms). Reopening UART; no CONTROL_UPDATE will be sent yet.\n";

    if (!g_stop.load(std::memory_order_relaxed) &&
        StartLink(config, options.ready_timeout_ms, &link)) {
        // After the MCU watchdog event, a reconnect/HELLO/HEARTBEAT may recover
        // the link state, but control must remain invalid until a new generation.
        results.link_loss_reconnect_safe = WaitControlValid(*link, false, 1500U);
        PrintSnapshot("after reconnect, before new control", link->GetSnapshot());
    } else {
        std::cerr << "UART did not recover after host silence\n";
    }

    if (link) {
        link->Stop();
        results.clean_stop = link->GetLinkState() == LinkState::CLOSED;
    }

    const bool report_ok = WriteReport(options, results);
    const bool passed = AllRequiredPass(results) && report_ok &&
                        !g_stop.load(std::memory_order_relaxed);

    std::cout << "\n=== FINAL RESULT ===\n"
              << "hello_heartbeat=" << results.hello_heartbeat << '\n'
              << "valid_control=" << results.valid_control << '\n'
              << "invalid_control=" << results.invalid_control << '\n'
              << "invalid_recovery=" << results.invalid_recovery << '\n'
              << "freshness=" << results.freshness << '\n'
              << "remote_stop=" << results.remote_stop << '\n'
              << "clear_stop_hold=" << results.clear_stop_hold << '\n'
              << "clear_stop_recovery=" << results.clear_stop_recovery << '\n'
              << "link_loss_silence=" << results.link_loss_silence << '\n'
              << "link_loss_reconnect_safe=" << results.link_loss_reconnect_safe << '\n'
              << "clean_stop=" << results.clean_stop << '\n'
              << "RESULT=" << (passed ? "PASS" : "FAIL") << '\n';

    if (passed) {
        std::cout
            << "NOTE: protocol STATUS does not expose a separate mailbox-valid bit.\n"
            << "Use MCU V6 logs/logic analyzer/motor-driver observation to close the\n"
            << "mailbox-invalid and physical motor-disable evidence for phases 3-6.\n";
    }

    return passed ? 0 : 6;
}