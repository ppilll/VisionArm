#include "uart/protocol.h"
#include "uart/serial_port.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <poll.h>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {

using visionarm::uart::Frame;
using visionarm::uart::FrameParser;
using visionarm::uart::Header;
using visionarm::uart::Heartbeat;
using visionarm::uart::Hello;
using visionarm::uart::HelloAck;
using visionarm::uart::Message;
using visionarm::uart::MessageBody;
using visionarm::uart::MessageType;
using visionarm::uart::SequenceRelation;
using visionarm::uart::SequenceTracker;
using visionarm::uart::SerialPort;
using visionarm::uart::SerialPortConfig;
using visionarm::uart::Status;
using visionarm::uart::ControlUpdate;

constexpr uint32_t kControlRateHz = 50U;
constexpr uint64_t kControlPeriodMs = 20U;
constexpr uint64_t kHeartbeatPeriodMs = 100U;
constexpr uint64_t kResponseTimeoutMs = 80U;
constexpr uint64_t kParserAssemblyTimeoutMs = 100U;
constexpr uint8_t kHostRole = 1U;
constexpr uint8_t kExpectedMcuRole = 2U;
constexpr uint16_t kHelloMaxPayload = 128U;
constexpr uint16_t kHelloMaxControlRateHz = 50U;
constexpr uint16_t kSoftwareVersionMajor = 6U;
constexpr uint16_t kSoftwareVersionMinor = 0U;
constexpr uint16_t kSoftwareVersionPatch = 11U;  // Step K utility.
constexpr uint32_t kSourceSessionId = 1U;
constexpr uint16_t kConfidenceFull = 65535U;
constexpr int16_t kStatusNormalizedMinimum = -32767;
constexpr int16_t kStatusNormalizedMaximum = 32767;

std::atomic<bool> g_stop{false};

void SignalHandler(int) noexcept {
    g_stop.store(true, std::memory_order_relaxed);
}

uint64_t MonotonicNs() noexcept {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t MonotonicMs() noexcept {
    return MonotonicNs() / 1000000ULL;
}

uint32_t GenerateNonzeroSessionId() noexcept {
    try {
        std::random_device random;
        const uint32_t a = static_cast<uint32_t>(random());
        const uint32_t b = static_cast<uint32_t>(random());
        const uint32_t value = (a << 16U) ^ b ^
                               static_cast<uint32_t>(MonotonicNs()) ^
                               static_cast<uint32_t>(::getpid());
        return value == 0U ? 1U : value;
    } catch (...) {
        uint32_t value = static_cast<uint32_t>(MonotonicNs()) ^
                         static_cast<uint32_t>(::getpid());
        if (value == 0U) {
            value = 1U;
        }
        return value;
    }
}

std::string NowLabel() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time_value = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&time_value, &local);
    std::ostringstream out;
    out << std::put_time(&local, "%H:%M:%S");
    return out.str();
}

struct Options {
    std::string device = "/dev/ttyS3";
    uint32_t baud = 115200U;
    uint64_t ready_timeout_ms = 5000U;
    int16_t center_tolerance_q15 = 2048;
    int16_t hold_tolerance_q15 = 2048;
    int16_t movement_min_q15 = 256;
    std::string report_path;
    std::string csv_path;
};

void PrintUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n\n"
        << "RK3588 Step-K synthetic gimbal-control contract test.\n"
        << "Uses the existing V5 protocol + SerialPort only; no simulator and no new wire type.\n\n"
        << "Fixed contract timings:\n"
        << "  HEARTBEAT       100 ms\n"
        << "  CONTROL_UPDATE   20 ms / 50 Hz\n"
        << "  capture_age_at_tx_ms = 0\n\n"
        << "Options:\n"
        << "  --device PATH                  UART device (default /dev/ttyS3)\n"
        << "  --baud N                       UART baud (default 115200)\n"
        << "  --ready-timeout-ms N           HELLO_ACK timeout budget (default 5000)\n"
        << "  --center-tolerance-q15 N       STATUS center tolerance (default 2048)\n"
        << "  --hold-tolerance-q15 N         Non-moving-axis tolerance (default 2048)\n"
        << "  --movement-min-q15 N           Minimum STATUS movement for sign checks (default 256)\n"
        << "  --report FILE                  Write text summary\n"
        << "  --csv FILE                     Write every received STATUS to CSV\n"
        << "  --help                         Show this help\n\n"
        << "WARNING: this program intentionally moves the physical gimbal.\n";
}

bool ParseUnsigned(const std::string& text, uint64_t maximum, uint64_t* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' || parsed > maximum) {
        return false;
    }
    *value = static_cast<uint64_t>(parsed);
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
        uint64_t parsed = 0U;
        if (arg == "--device") {
            options->device = value;
        } else if (arg == "--baud") {
            if (!ParseUnsigned(value, std::numeric_limits<uint32_t>::max(), &parsed) ||
                parsed == 0U) {
                return false;
            }
            options->baud = static_cast<uint32_t>(parsed);
        } else if (arg == "--ready-timeout-ms") {
            if (!ParseUnsigned(value, 60000U, &parsed) || parsed == 0U) {
                return false;
            }
            options->ready_timeout_ms = parsed;
        } else if (arg == "--center-tolerance-q15") {
            if (!ParseUnsigned(value, 32767U, &parsed)) {
                return false;
            }
            options->center_tolerance_q15 = static_cast<int16_t>(parsed);
        } else if (arg == "--hold-tolerance-q15") {
            if (!ParseUnsigned(value, 32767U, &parsed)) {
                return false;
            }
            options->hold_tolerance_q15 = static_cast<int16_t>(parsed);
        } else if (arg == "--movement-min-q15") {
            if (!ParseUnsigned(value, 32767U, &parsed)) {
                return false;
            }
            options->movement_min_q15 = static_cast<int16_t>(parsed);
        } else if (arg == "--report") {
            options->report_path = value;
        } else if (arg == "--csv") {
            options->csv_path = value;
        } else {
            std::cerr << "unknown option: " << arg << '\n';
            return false;
        }
    }
    return !options->device.empty();
}

void EnsureParentDirectory(const std::string& path) {
    if (path.empty()) {
        return;
    }
    const std::filesystem::path file_path(path);
    const auto parent = file_path.parent_path();
    if (!parent.empty()) {
        std::error_code error;
        std::filesystem::create_directories(parent, error);
    }
}

struct McuErrorCounters {
    uint32_t crc = 0U;
    uint32_t length = 0U;
    uint32_t version = 0U;
    uint32_t unknown_type = 0U;
    uint32_t sequence_gap = 0U;
    uint32_t overflow = 0U;
};

McuErrorCounters ExtractMcuErrors(const Status& status) noexcept {
    return {status.rx_crc_error_count,
            status.rx_length_error_count,
            status.rx_version_error_count,
            status.rx_unknown_type_count,
            status.rx_sequence_gap_count,
            status.rx_overflow_count};
}

bool SameMcuErrors(const McuErrorCounters& a, const McuErrorCounters& b) noexcept {
    return a.crc == b.crc && a.length == b.length &&
           a.version == b.version && a.unknown_type == b.unknown_type &&
           a.sequence_gap == b.sequence_gap && a.overflow == b.overflow;
}

struct TestResults {
    bool hello = false;
    bool heartbeat_contract = false;
    bool session_contract = false;
    bool k0_center = false;
    bool k0_dead_zone = false;
    bool k1_pan_positive = false;
    bool k1_pan_return = false;
    bool k2_pan_negative = false;
    bool k2_pan_return = false;
    bool k3_tilt_positive = false;
    bool k3_tilt_return = false;
    bool k4_tilt_negative = false;
    bool k4_tilt_return = false;
    bool k5_status_direction = false;
    bool k6_uart_integrity = false;
    bool k7_stale = false;
    bool k7_fresh_reenable = false;
    bool final_error_counters = false;
    bool clean_close = false;
};

bool AllAutomatedPass(const TestResults& r) noexcept {
    return r.hello && r.heartbeat_contract && r.session_contract &&
           r.k0_center && r.k0_dead_zone &&
           r.k1_pan_positive && r.k1_pan_return &&
           r.k2_pan_negative && r.k2_pan_return &&
           r.k3_tilt_positive && r.k3_tilt_return &&
           r.k4_tilt_negative && r.k4_tilt_return &&
           r.k5_status_direction && r.k6_uart_integrity &&
           r.k7_stale && r.k7_fresh_reenable &&
           r.final_error_counters && r.clean_close;
}

class StepKHost {
public:
    explicit StepKHost(const Options& options)
        : options_(options), parser_(kParserAssemblyTimeoutMs),
          sender_boot_id_(GenerateNonzeroSessionId()) {}

    bool Open(std::string* error) {
        SerialPortConfig config;
        config.device_path = options_.device;
        config.baud_rate = options_.baud;
        config.exclusive = true;
        config.flush_on_open = true;
        if (!port_.Open(config, error)) {
            return false;
        }
        start_ms_ = MonotonicMs();
        next_heartbeat_due_ms_ = 0U;
        last_heartbeat_tx_ms_ = 0U;
        if (!options_.csv_path.empty()) {
            EnsureParentDirectory(options_.csv_path);
            csv_.open(options_.csv_path);
            if (!csv_) {
                if (error != nullptr) {
                    *error = "cannot open CSV file: " + options_.csv_path;
                }
                port_.Close();
                return false;
            }
            csv_ << "host_ms,phase,heartbeat_index,mcu_state,mcu_link_state,remote_stop_latched,"
                    "control_valid,last_rx_wire_sequence,last_control_wire_sequence,rx_valid_frame_count,"
                    "rx_crc_error_count,rx_length_error_count,rx_version_error_count,rx_unknown_type_count,"
                    "rx_sequence_gap_count,rx_overflow_count,control_mailbox_overwrite_count,mcu_tick_ms,"
                    "pan_stub_q15,tilt_stub_q15,pan_approx_us,tilt_approx_us\n";
        }
        return true;
    }

    void Close() noexcept {
        port_.Close();
        if (csv_.is_open()) {
            csv_.flush();
            csv_.close();
        }
    }

    [[nodiscard]] uint32_t SenderBootId() const noexcept { return sender_boot_id_; }
    [[nodiscard]] uint32_t NextWireSequence() const noexcept { return next_wire_sequence_; }
    [[nodiscard]] uint64_t HeartbeatsSent() const noexcept { return heartbeats_sent_; }
    [[nodiscard]] uint64_t StatusReceived() const noexcept { return statuses_received_; }
    [[nodiscard]] uint64_t ControlsSent() const noexcept { return controls_sent_; }
    [[nodiscard]] uint64_t MaxActiveControlGapMs() const noexcept { return max_active_control_gap_ms_; }
    [[nodiscard]] uint64_t MaxHeartbeatGapMs() const noexcept { return max_heartbeat_gap_ms_; }
    [[nodiscard]] uint64_t McuBootId() const noexcept { return mcu_boot_id_; }
    [[nodiscard]] uint64_t PeerBootChanges() const noexcept { return peer_boot_changes_; }
    [[nodiscard]] const std::optional<Status>& LastStatus() const noexcept { return last_status_; }
    [[nodiscard]] const visionarm::uart::ParserStats& ParserStats() const noexcept {
        return parser_.Stats();
    }

    void SetPhase(std::string phase) {
        phase_ = std::move(phase);
        std::cout << "\n[" << NowLabel() << "] === " << phase_ << " ===\n";
    }

    bool Handshake(uint64_t timeout_ms) {
        Hello hello;
        hello.device_role = kHostRole;
        hello.minimum_protocol_version = visionarm::uart::kProtocolVersion;
        hello.maximum_protocol_version = visionarm::uart::kProtocolVersion;
        hello.max_payload = kHelloMaxPayload;
        hello.max_control_rate_hz = kHelloMaxControlRateHz;
        hello.capability_bits = 0U;
        hello.software_version_major = kSoftwareVersionMajor;
        hello.software_version_minor = kSoftwareVersionMinor;
        hello.software_version_patch = kSoftwareVersionPatch;

        const uint32_t hello_sequence = next_wire_sequence_;
        if (!SendBody(hello)) {
            return false;
        }

        const uint64_t deadline = MonotonicMs() + timeout_ms;
        while (!g_stop.load(std::memory_order_relaxed) && MonotonicMs() < deadline) {
            Message message;
            if (!ReadOneMessage(100U, &message)) {
                continue;
            }
            if (message.header.message_type != MessageType::HELLO_ACK) {
                std::cerr << "unexpected response while waiting HELLO_ACK: type="
                          << static_cast<unsigned>(message.header.message_type) << '\n';
                return false;
            }
            const auto& ack = std::get<HelloAck>(message.body);
            if (ack.hello_wire_sequence != hello_sequence || ack.accepted == 0U ||
                ack.selected_protocol_version != visionarm::uart::kProtocolVersion ||
                ack.device_role != kExpectedMcuRole || ack.max_payload < kHelloMaxPayload ||
                ack.max_control_rate_hz < kControlRateHz) {
                std::cerr << "HELLO_ACK rejected/contract mismatch: accepted="
                          << static_cast<unsigned>(ack.accepted)
                          << " version=" << static_cast<unsigned>(ack.selected_protocol_version)
                          << " role=" << static_cast<unsigned>(ack.device_role)
                          << " max_payload=" << ack.max_payload
                          << " max_control_rate_hz=" << ack.max_control_rate_hz << '\n';
                return false;
            }
            std::cout << "HELLO_ACK accepted: sender_boot_id=" << sender_boot_id_
                      << " mcu_boot_id=" << message.header.sender_boot_id
                      << " mcu_max_control_rate_hz=" << ack.max_control_rate_hz
                      << " firmware=" << ack.firmware_version_major << '.'
                      << ack.firmware_version_minor << '.' << ack.firmware_version_patch << '\n';
            next_heartbeat_due_ms_ = MonotonicMs();
            return true;
        }
        std::cerr << "HELLO_ACK timeout\n";
        return false;
    }

    bool SendInitialHeartbeat() {
        return SendHeartbeatAndReceiveStatus();
    }

    bool RunControlBlock(uint64_t duration_ms, int16_t error_x_q15, int16_t error_y_q15) {
        const uint64_t start = MonotonicMs();
        const uint64_t end = start + duration_ms;
        uint64_t next_control_due = start;
        uint64_t previous_control_tx_ms = 0U;
        while (!g_stop.load(std::memory_order_relaxed) && MonotonicMs() < end) {
            const uint64_t now = MonotonicMs();
            if (now >= next_control_due) {
                if (!SendControl(error_x_q15, error_y_q15)) {
                    return false;
                }
                if (previous_control_tx_ms != 0U) {
                    const uint64_t gap = last_control_tx_ms_ - previous_control_tx_ms;
                    max_active_control_gap_ms_ = std::max(max_active_control_gap_ms_, gap);
                    if (gap > 30U) {
                        std::cerr << "CONTROL_UPDATE active gap exceeded 30 ms in "
                                  << phase_ << ": " << gap << " ms\n";
                        return false;
                    }
                }
                previous_control_tx_ms = last_control_tx_ms_;
                next_control_due += kControlPeriodMs;
                if (next_control_due + kControlPeriodMs < now) {
                    std::cerr << "CONTROL_UPDATE scheduler overrun in " << phase_ << '\n';
                    return false;
                }
                continue;
            }
            if (now >= next_heartbeat_due_ms_) {
                if (!SendHeartbeatAndReceiveStatus()) {
                    return false;
                }
                continue;
            }
            const uint64_t next_event = std::min({next_control_due, next_heartbeat_due_ms_, end});
            SleepUntilMs(next_event);
        }
        return !g_stop.load(std::memory_order_relaxed);
    }

    bool RunHeartbeatOnly(uint64_t duration_ms) {
        const uint64_t end = MonotonicMs() + duration_ms;
        while (!g_stop.load(std::memory_order_relaxed) && MonotonicMs() < end) {
            const uint64_t now = MonotonicMs();
            if (now >= next_heartbeat_due_ms_) {
                if (!SendHeartbeatAndReceiveStatus()) {
                    return false;
                }
                continue;
            }
            SleepUntilMs(std::min(next_heartbeat_due_ms_, end));
        }
        return !g_stop.load(std::memory_order_relaxed);
    }

    bool SendHeartbeatAndReceiveStatus() {
        const uint64_t before = MonotonicMs();
        if (last_heartbeat_tx_ms_ != 0U) {
            const uint64_t gap = before - last_heartbeat_tx_ms_;
            max_heartbeat_gap_ms_ = std::max(max_heartbeat_gap_ms_, gap);
        }
        if (!SendBody(Heartbeat{})) {
            return false;
        }
        last_heartbeat_tx_ms_ = MonotonicMs();
        ++heartbeats_sent_;
        next_heartbeat_due_ms_ += kHeartbeatPeriodMs;
        if (next_heartbeat_due_ms_ <= last_heartbeat_tx_ms_) {
            const uint64_t periods_late =
                (last_heartbeat_tx_ms_ - next_heartbeat_due_ms_) / kHeartbeatPeriodMs + 1U;
            next_heartbeat_due_ms_ += periods_late * kHeartbeatPeriodMs;
        }

        Message message;
        if (!ReadOneMessage(kResponseTimeoutMs, &message)) {
            std::cerr << "STATUS timeout after HEARTBEAT #" << heartbeats_sent_ << '\n';
            return false;
        }
        if (message.header.message_type != MessageType::STATUS) {
            std::cerr << "expected STATUS, got type="
                      << static_cast<unsigned>(message.header.message_type) << '\n';
            return false;
        }
        last_status_ = std::get<Status>(message.body);
        ++statuses_received_;
        RecordStatus(*last_status_);
        PrintStatus(*last_status_);
        return StatusCommandRangeValid(*last_status_);
    }

private:
    template <typename T>
    bool SendBody(const T& body) {
        Message message;
        message.header.protocol_version = visionarm::uart::kProtocolVersion;
        message.header.message_type = visionarm::uart::MessageTypeOf(MessageBody{body});
        message.header.wire_sequence = next_wire_sequence_++;
        message.header.sender_boot_id = sender_boot_id_;
        const uint64_t now = MonotonicMs();
        message.header.sender_uptime_ms = static_cast<uint32_t>(now - start_ms_);
        message.body = body;

        std::vector<uint8_t> encoded;
        std::string error;
        if (!visionarm::uart::EncodeMessage(message, &encoded, &error)) {
            std::cerr << "EncodeMessage failed: " << error << '\n';
            return false;
        }
        return WriteAll(encoded);
    }

    bool SendControl(int16_t error_x_q15, int16_t error_y_q15) {
        ControlUpdate control;
        control.source_capture_session_id = kSourceSessionId;
        control.source_frame_id = next_frame_id_;
        control.source_v4l2_sequence = next_frame_id_;
        ++next_frame_id_;
        control.target_state = 2U;       // DETECTED.
        control.control_flags = 0x01U;   // VALID.
        control.dx_px = 0;
        control.dy_px = 0;
        control.error_x_q15 = error_x_q15;
        control.error_y_q15 = error_y_q15;
        control.confidence_u16 = kConfidenceFull;
        control.capture_age_at_tx_ms = 0U;

        if (!SendBody(control)) {
            return false;
        }
        last_control_tx_ms_ = MonotonicMs();
        ++controls_sent_;
        return true;
    }

    bool WriteAll(const std::vector<uint8_t>& bytes) {
        std::size_t offset = 0U;
        const uint64_t deadline = MonotonicMs() + 100U;
        while (offset < bytes.size() && !g_stop.load(std::memory_order_relaxed)) {
            std::string error;
            const ssize_t written = port_.Write(bytes.data() + offset,
                                                bytes.size() - offset,
                                                &error);
            if (written > 0) {
                offset += static_cast<std::size_t>(written);
                continue;
            }
            if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                std::cerr << "serial write failed: " << error << '\n';
                return false;
            }
            const int remaining = static_cast<int>(deadline > MonotonicMs()
                ? deadline - MonotonicMs() : 0U);
            if (remaining <= 0) {
                std::cerr << "serial write timeout\n";
                return false;
            }
            pollfd pfd{};
            pfd.fd = port_.Fd();
            pfd.events = POLLOUT;
            const int result = ::poll(&pfd, 1, remaining);
            if (result < 0 && errno == EINTR) {
                continue;
            }
            if (result <= 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                std::cerr << "serial POLLOUT failure revents=" << pfd.revents << '\n';
                return false;
            }
        }
        if (offset != bytes.size()) {
            return false;
        }
        if (::tcdrain(port_.Fd()) != 0) {
            std::cerr << "tcdrain failed: " << std::strerror(errno) << '\n';
            return false;
        }
        return true;
    }

    bool ReadOneMessage(uint64_t timeout_ms, Message* message) {
        if (message == nullptr) {
            return false;
        }
        const uint64_t deadline = MonotonicMs() + timeout_ms;
        std::array<uint8_t, 512U> buffer{};
        while (!g_stop.load(std::memory_order_relaxed) && MonotonicMs() < deadline) {
            const uint64_t now = MonotonicMs();
            const int remaining = static_cast<int>(deadline - now);
            pollfd pfd{};
            pfd.fd = port_.Fd();
            pfd.events = POLLIN;
            const int poll_result = ::poll(&pfd, 1, remaining);
            if (poll_result < 0) {
                if (errno == EINTR) {
                    continue;
                }
                std::cerr << "serial POLLIN poll failed: " << std::strerror(errno) << '\n';
                return false;
            }
            if (poll_result == 0) {
                parser_.Tick(MonotonicMs());
                continue;
            }
            if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                std::cerr << "serial POLLIN failure revents=" << pfd.revents << '\n';
                return false;
            }
            if ((pfd.revents & POLLIN) == 0) {
                continue;
            }
            std::string error;
            const ssize_t bytes = port_.Read(buffer.data(), buffer.size(), &error);
            if (bytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    continue;
                }
                std::cerr << "serial read failed: " << error << '\n';
                return false;
            }
            if (bytes == 0) {
                continue;
            }
            std::vector<Frame> frames;
            parser_.Feed(buffer.data(), static_cast<std::size_t>(bytes),
                         MonotonicMs(), &frames);
            for (const auto& frame : frames) {
                Message decoded;
                if (!visionarm::uart::DecodeMessage(frame, &decoded, &error)) {
                    std::cerr << "DecodeMessage failed: " << error << '\n';
                    return false;
                }
                if (!ValidatePeerSequence(decoded.header)) {
                    return false;
                }
                *message = std::move(decoded);
                return true;
            }
        }
        return false;
    }

    bool ValidatePeerSequence(const Header& header) {
        if (header.sender_boot_id == 0U) {
            std::cerr << "MCU sender_boot_id is zero\n";
            return false;
        }
        if (mcu_boot_id_ == 0U) {
            mcu_boot_id_ = header.sender_boot_id;
        } else if (mcu_boot_id_ != header.sender_boot_id) {
            ++peer_boot_changes_;
            std::cerr << "MCU reboot/session change detected: old=" << mcu_boot_id_
                      << " new=" << header.sender_boot_id << '\n';
            mcu_boot_id_ = header.sender_boot_id;
            return false;
        }
        const SequenceRelation relation = peer_sequence_.Observe(
            header.sender_boot_id, header.wire_sequence);
        if (relation == SequenceRelation::DUPLICATE ||
            relation == SequenceRelation::OLDER ||
            relation == SequenceRelation::NEWER_GAP) {
            std::cerr << "MCU response wire sequence violation relation="
                      << static_cast<int>(relation)
                      << " sequence=" << header.wire_sequence << '\n';
            return false;
        }
        return true;
    }

    void SleepUntilMs(uint64_t deadline_ms) const {
        while (!g_stop.load(std::memory_order_relaxed)) {
            const uint64_t now = MonotonicMs();
            if (now >= deadline_ms) {
                return;
            }
            const uint64_t remaining = deadline_ms - now;
            const uint64_t sleep_ms = std::min<uint64_t>(remaining, 2U);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    static double ApproxPulseUs(int16_t q15, double minimum, double center,
                                double maximum) noexcept {
        const double normalized = std::clamp(static_cast<double>(q15) / 32767.0,
                                             -1.0, 1.0);
        if (normalized < 0.0) {
            return center + normalized * (center - minimum);
        }
        return center + normalized * (maximum - center);
    }

    bool StatusCommandRangeValid(const Status& status) const {
        if (status.pan_stub_q15 < kStatusNormalizedMinimum ||
            status.pan_stub_q15 > kStatusNormalizedMaximum ||
            status.tilt_stub_q15 < kStatusNormalizedMinimum ||
            status.tilt_stub_q15 > kStatusNormalizedMaximum) {
            std::cerr << "STATUS applied-command q15 outside Step-K normalized range\n";
            return false;
        }
        return true;
    }

    void RecordStatus(const Status& status) {
        if (!csv_.is_open()) {
            return;
        }
        const double pan_us = ApproxPulseUs(status.pan_stub_q15, 1000.0, 1500.0, 2000.0);
        const double tilt_us = ApproxPulseUs(status.tilt_stub_q15, 1200.0, 1500.0, 1600.0);
        csv_ << MonotonicMs() - start_ms_ << ',' << phase_ << ',' << heartbeats_sent_ << ','
             << static_cast<unsigned>(status.mcu_state) << ','
             << static_cast<unsigned>(status.link_state) << ','
             << static_cast<unsigned>(status.remote_stop_latched) << ','
             << static_cast<unsigned>(status.control_valid) << ','
             << status.last_rx_wire_sequence << ',' << status.last_control_wire_sequence << ','
             << status.rx_valid_frame_count << ',' << status.rx_crc_error_count << ','
             << status.rx_length_error_count << ',' << status.rx_version_error_count << ','
             << status.rx_unknown_type_count << ',' << status.rx_sequence_gap_count << ','
             << status.rx_overflow_count << ',' << status.control_mailbox_overwrite_count << ','
             << status.mcu_tick_ms << ',' << status.pan_stub_q15 << ',' << status.tilt_stub_q15
             << ',' << std::fixed << std::setprecision(2) << pan_us << ',' << tilt_us << '\n';
        csv_.flush();
    }

    void PrintStatus(const Status& status) const {
        const double pan_us = ApproxPulseUs(status.pan_stub_q15, 1000.0, 1500.0, 2000.0);
        const double tilt_us = ApproxPulseUs(status.tilt_stub_q15, 1200.0, 1500.0, 1600.0);
        std::cout << '[' << NowLabel() << "] STATUS"
                  << " hb=" << heartbeats_sent_
                  << " link=" << static_cast<unsigned>(status.link_state)
                  << " valid=" << static_cast<unsigned>(status.control_valid)
                  << " pan_q15=" << status.pan_stub_q15
                  << " tilt_q15=" << status.tilt_stub_q15
                  << " pan~" << std::fixed << std::setprecision(1) << pan_us << "us"
                  << " tilt~" << tilt_us << "us"
                  << " last_rx=" << status.last_rx_wire_sequence
                  << " last_control=" << status.last_control_wire_sequence
                  << " crc=" << status.rx_crc_error_count
                  << " len=" << status.rx_length_error_count
                  << " ver=" << status.rx_version_error_count
                  << " type=" << status.rx_unknown_type_count
                  << " gap=" << status.rx_sequence_gap_count
                  << " ovf=" << status.rx_overflow_count << '\n';
    }

    Options options_;
    SerialPort port_;
    FrameParser parser_;
    SequenceTracker peer_sequence_;
    const uint32_t sender_boot_id_;
    uint32_t next_wire_sequence_ = 0U;
    uint32_t next_frame_id_ = 1U;
    uint32_t mcu_boot_id_ = 0U;
    uint64_t peer_boot_changes_ = 0U;
    uint64_t start_ms_ = 0U;
    uint64_t next_heartbeat_due_ms_ = 0U;
    uint64_t last_heartbeat_tx_ms_ = 0U;
    uint64_t last_control_tx_ms_ = 0U;
    uint64_t heartbeats_sent_ = 0U;
    uint64_t statuses_received_ = 0U;
    uint64_t controls_sent_ = 0U;
    uint64_t max_active_control_gap_ms_ = 0U;
    uint64_t max_heartbeat_gap_ms_ = 0U;
    std::optional<Status> last_status_;
    std::string phase_ = "startup";
    std::ofstream csv_;
};

struct StatusPoint {
    int16_t pan = 0;
    int16_t tilt = 0;
    uint8_t link_state = 0U;
    uint8_t control_valid = 0U;
};

std::optional<StatusPoint> Point(const StepKHost& host) {
    if (!host.LastStatus().has_value()) {
        return std::nullopt;
    }
    const auto& status = *host.LastStatus();
    return StatusPoint{status.pan_stub_q15, status.tilt_stub_q15,
                       status.link_state, status.control_valid};
}

bool Near(int16_t a, int16_t b, int16_t tolerance) noexcept {
    const int difference = std::abs(static_cast<int>(a) - static_cast<int>(b));
    return difference <= static_cast<int>(tolerance);
}

bool WriteReport(const Options& options,
                 const TestResults& results,
                 const StepKHost& host,
                 const McuErrorCounters& baseline,
                 const McuErrorCounters& final_errors,
                 bool manual_required) {
    if (options.report_path.empty()) {
        return true;
    }
    EnsureParentDirectory(options.report_path);
    std::ofstream out(options.report_path);
    if (!out) {
        std::cerr << "cannot open report: " << options.report_path << '\n';
        return false;
    }
    out << "uart_step_k_synthetic_test\n"
        << "device=" << options.device << '\n'
        << "baud=" << options.baud << '\n'
        << "sender_boot_id=" << host.SenderBootId() << '\n'
        << "mcu_boot_id=" << host.McuBootId() << '\n'
        << "next_wire_sequence=" << host.NextWireSequence() << '\n'
        << "heartbeats_sent=" << host.HeartbeatsSent() << '\n'
        << "status_received=" << host.StatusReceived() << '\n'
        << "controls_sent=" << host.ControlsSent() << '\n'
        << "max_active_control_gap_ms=" << host.MaxActiveControlGapMs() << '\n'
        << "max_heartbeat_gap_ms=" << host.MaxHeartbeatGapMs() << '\n'
        << "peer_boot_changes=" << host.PeerBootChanges() << '\n'
        << "baseline_crc=" << baseline.crc << '\n'
        << "final_crc=" << final_errors.crc << '\n'
        << "baseline_length=" << baseline.length << '\n'
        << "final_length=" << final_errors.length << '\n'
        << "baseline_version=" << baseline.version << '\n'
        << "final_version=" << final_errors.version << '\n'
        << "baseline_unknown_type=" << baseline.unknown_type << '\n'
        << "final_unknown_type=" << final_errors.unknown_type << '\n'
        << "baseline_sequence_gap=" << baseline.sequence_gap << '\n'
        << "final_sequence_gap=" << final_errors.sequence_gap << '\n'
        << "baseline_overflow=" << baseline.overflow << '\n'
        << "final_overflow=" << final_errors.overflow << '\n'
        << "hello=" << results.hello << '\n'
        << "heartbeat_contract=" << results.heartbeat_contract << '\n'
        << "session_contract=" << results.session_contract << '\n'
        << "k0_center=" << results.k0_center << '\n'
        << "k0_dead_zone=" << results.k0_dead_zone << '\n'
        << "k1_pan_positive=" << results.k1_pan_positive << '\n'
        << "k1_pan_return=" << results.k1_pan_return << '\n'
        << "k2_pan_negative=" << results.k2_pan_negative << '\n'
        << "k2_pan_return=" << results.k2_pan_return << '\n'
        << "k3_tilt_positive=" << results.k3_tilt_positive << '\n'
        << "k3_tilt_return=" << results.k3_tilt_return << '\n'
        << "k4_tilt_negative=" << results.k4_tilt_negative << '\n'
        << "k4_tilt_return=" << results.k4_tilt_return << '\n'
        << "k5_status_direction=" << results.k5_status_direction << '\n'
        << "k6_uart_integrity=" << results.k6_uart_integrity << '\n'
        << "k7_stale=" << results.k7_stale << '\n'
        << "k7_fresh_reenable=" << results.k7_fresh_reenable << '\n'
        << "final_error_counters=" << results.final_error_counters << '\n'
        << "clean_close=" << results.clean_close << '\n'
        << "manual_logic_analyzer_required=" << manual_required << '\n'
        << "AUTOMATED_RESULT=" << (AllAutomatedPass(results) ? "PASS" : "FAIL") << '\n';
    return true;
}

void PrintManualEvidenceChecklist() {
    std::cout
        << "\n=== MANUAL / LOGIC-ANALYZER EVIDENCE REQUIRED ===\n"
        << "K0: PA6~=1500 us, PA7~=1500 us; +1024/-1024 dead-zone causes no persistent movement.\n"
        << "K1: +X moves camera RIGHT; Pan PWM decreases ~1 us per 20 ms cycle.\n"
        << "K2: -X moves camera LEFT; Pan PWM increases ~1 us per 20 ms cycle.\n"
        << "K3: +Y moves camera DOWN; Tilt PWM increases ~1 us per 20 ms cycle.\n"
        << "K4: -Y moves camera UP; Tilt PWM decreases ~1 us per 20 ms cycle.\n"
        << "K5: no one-cycle target delta >2 us; Pan stays 1000..2000 us; Tilt 1200..1600 us.\n"
        << "K6: no >2 us/cycle jump, no dense-edge PWM regression, bounded oscillation only.\n"
        << "K7: after freshness expiry PA6/PA7 PWM disabled and pins LOW; no old command resumes.\n";
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

    std::cout
        << "RK3588 Synthetic Control Contract for V6 Step K\n"
        << "device=" << options.device << " baud=" << options.baud << '\n'
        << "HEARTBEAT=100 ms CONTROL_UPDATE=20 ms/50 Hz capture_age_at_tx_ms=0\n"
        << "SAFETY: mechanically secure the gimbal and verify calibrated PWM limits first.\n";

    StepKHost host(options);
    std::string error;
    if (!host.Open(&error)) {
        std::cerr << "SerialPort::Open failed: " << error << '\n';
        return 3;
    }

    TestResults results;
    McuErrorCounters baseline_errors{};
    McuErrorCounters final_errors{};
    const uint32_t initial_boot_id = host.SenderBootId();

    host.SetPhase("Session: HELLO -> accepted HELLO_ACK");
    results.hello = host.Handshake(options.ready_timeout_ms);
    if (!results.hello) {
        host.Close();
        return 4;
    }
    if (host.SenderBootId() == 0U) {
        std::cerr << "sender_boot_id must be nonzero\n";
        host.Close();
        return 4;
    }

    host.SetPhase("Initial HEARTBEAT -> STATUS baseline");
    if (!host.SendInitialHeartbeat() || !host.LastStatus().has_value()) {
        host.Close();
        return 5;
    }
    baseline_errors = ExtractMcuErrors(*host.LastStatus());
    const uint8_t ready_link_state = host.LastStatus()->link_state;

    host.SetPhase("K0a - Center: (0, 0) for 1000 ms");
    if (!host.RunControlBlock(1000U, 0, 0)) {
        host.Close();
        return 6;
    }
    const auto k0_center = Point(host);
    results.k0_center = k0_center.has_value() && k0_center->control_valid != 0U &&
                        Near(k0_center->pan, 0, options.center_tolerance_q15) &&
                        Near(k0_center->tilt, 0, options.center_tolerance_q15);
    std::cout << (results.k0_center ? "PASS" : "FAIL")
              << " K0 center STATUS check\n";

    host.SetPhase("K0b - Dead-zone: (+1024, -1024) for 500 ms");
    if (!host.RunControlBlock(500U, 1024, -1024)) {
        host.Close();
        return 6;
    }
    const auto k0_dead = Point(host);
    results.k0_dead_zone = k0_center.has_value() && k0_dead.has_value() &&
                           k0_dead->control_valid != 0U &&
                           Near(k0_dead->pan, k0_center->pan, options.center_tolerance_q15) &&
                           Near(k0_dead->tilt, k0_center->tilt, options.center_tolerance_q15);
    std::cout << (results.k0_dead_zone ? "PASS" : "FAIL")
              << " K0 dead-zone STATUS stability\n";

    host.SetPhase("K1a - Pan +8192 for 600 ms: expected camera RIGHT / Pan PWM decrease");
    const auto k1_base = Point(host);
    if (!host.RunControlBlock(600U, 8192, 0)) {
        host.Close();
        return 6;
    }
    const auto k1_pos = Point(host);
    results.k1_pan_positive = k1_base.has_value() && k1_pos.has_value() &&
        static_cast<int>(k1_pos->pan) <= static_cast<int>(k1_base->pan) - options.movement_min_q15 &&
        Near(k1_pos->tilt, k1_base->tilt, options.hold_tolerance_q15);
    std::cout << (results.k1_pan_positive ? "PASS" : "FAIL")
              << " K1 positive-X STATUS direction\n";

    host.SetPhase("K1b - Pan -8192 for 600 ms: return toward center");
    if (!host.RunControlBlock(600U, -8192, 0)) {
        host.Close();
        return 6;
    }
    const auto k1_return = Point(host);
    results.k1_pan_return = k1_base.has_value() && k1_pos.has_value() && k1_return.has_value() &&
        k1_return->pan > k1_pos->pan;
    std::cout << (results.k1_pan_return ? "PASS" : "FAIL") << " K1 return direction\n";

    host.SetPhase("K2a - Pan -8192 for 600 ms: expected camera LEFT / Pan PWM increase");
    const auto k2_base = Point(host);
    if (!host.RunControlBlock(600U, -8192, 0)) {
        host.Close();
        return 6;
    }
    const auto k2_neg = Point(host);
    results.k2_pan_negative = k2_base.has_value() && k2_neg.has_value() &&
        static_cast<int>(k2_neg->pan) >= static_cast<int>(k2_base->pan) + options.movement_min_q15 &&
        Near(k2_neg->tilt, k2_base->tilt, options.hold_tolerance_q15);
    std::cout << (results.k2_pan_negative ? "PASS" : "FAIL")
              << " K2 negative-X STATUS direction\n";

    host.SetPhase("K2b - Pan +8192 for 600 ms: return toward center");
    if (!host.RunControlBlock(600U, 8192, 0)) {
        host.Close();
        return 6;
    }
    const auto k2_return = Point(host);
    results.k2_pan_return = k2_base.has_value() && k2_neg.has_value() && k2_return.has_value() &&
        k2_return->pan < k2_neg->pan;
    std::cout << (results.k2_pan_return ? "PASS" : "FAIL") << " K2 return direction\n";

    host.SetPhase("K3a - Tilt +8192 for 600 ms: expected camera DOWN / Tilt PWM increase");
    const auto k3_base = Point(host);
    if (!host.RunControlBlock(600U, 0, 8192)) {
        host.Close();
        return 6;
    }
    const auto k3_pos = Point(host);
    results.k3_tilt_positive = k3_base.has_value() && k3_pos.has_value() &&
        static_cast<int>(k3_pos->tilt) >= static_cast<int>(k3_base->tilt) + options.movement_min_q15 &&
        Near(k3_pos->pan, k3_base->pan, options.hold_tolerance_q15);
    std::cout << (results.k3_tilt_positive ? "PASS" : "FAIL")
              << " K3 positive-Y STATUS direction\n";

    host.SetPhase("K3b - Tilt -8192 for 600 ms: return toward center");
    if (!host.RunControlBlock(600U, 0, -8192)) {
        host.Close();
        return 6;
    }
    const auto k3_return = Point(host);
    results.k3_tilt_return = k3_base.has_value() && k3_pos.has_value() && k3_return.has_value() &&
        k3_return->tilt < k3_pos->tilt;
    std::cout << (results.k3_tilt_return ? "PASS" : "FAIL") << " K3 return direction\n";

    host.SetPhase("K4a - Tilt -8192 for 600 ms: expected camera UP / Tilt PWM decrease");
    const auto k4_base = Point(host);
    if (!host.RunControlBlock(600U, 0, -8192)) {
        host.Close();
        return 6;
    }
    const auto k4_neg = Point(host);
    results.k4_tilt_negative = k4_base.has_value() && k4_neg.has_value() &&
        static_cast<int>(k4_neg->tilt) <= static_cast<int>(k4_base->tilt) - options.movement_min_q15 &&
        Near(k4_neg->pan, k4_base->pan, options.hold_tolerance_q15);
    std::cout << (results.k4_tilt_negative ? "PASS" : "FAIL")
              << " K4 negative-Y STATUS direction\n";

    host.SetPhase("K4b - Tilt +8192 for 600 ms: return toward center");
    if (!host.RunControlBlock(600U, 0, 8192)) {
        host.Close();
        return 6;
    }
    const auto k4_return = Point(host);
    results.k4_tilt_return = k4_base.has_value() && k4_neg.has_value() && k4_return.has_value() &&
        k4_return->tilt > k4_neg->tilt;
    std::cout << (results.k4_tilt_return ? "PASS" : "FAIL") << " K4 return direction\n";

    host.SetPhase("K5a - Full scale (+32767, +32767) for 200 ms");
    const auto k5_base = Point(host);
    if (!host.RunControlBlock(200U, 32767, 32767)) {
        host.Close();
        return 6;
    }
    const auto k5_pos = Point(host);

    host.SetPhase("K5b - Full scale (-32768, -32768) for 200 ms");
    if (!host.RunControlBlock(200U, std::numeric_limits<int16_t>::min(),
                             std::numeric_limits<int16_t>::min())) {
        host.Close();
        return 6;
    }
    const auto k5_neg = Point(host);
    results.k5_status_direction = k5_base.has_value() && k5_pos.has_value() && k5_neg.has_value() &&
        k5_pos->pan < k5_base->pan && k5_pos->tilt > k5_base->tilt &&
        k5_neg->pan > k5_pos->pan && k5_neg->tilt < k5_pos->tilt;
    std::cout << (results.k5_status_direction ? "PASS" : "FAIL")
              << " K5 coarse STATUS direction; 2 us/cycle still requires logic analyzer\n";

    host.SetPhase("K6 - Rapid sign change: alternate full scale every 100 ms for 2000 ms");
    const auto parser_before_k6 = host.ParserStats();
    const uint64_t boot_changes_before_k6 = host.PeerBootChanges();
    bool k6_transport_ok = true;
    for (int block = 0; block < 20 && !g_stop.load(std::memory_order_relaxed); ++block) {
        const bool a = (block % 2) == 0;
        const int16_t x = a ? 32767 : std::numeric_limits<int16_t>::min();
        const int16_t y = a ? 32767 : std::numeric_limits<int16_t>::min();
        if (!host.RunControlBlock(100U, x, y)) {
            k6_transport_ok = false;
            break;
        }
    }
    const auto parser_after_k6 = host.ParserStats();
    results.k6_uart_integrity = k6_transport_ok &&
        host.PeerBootChanges() == boot_changes_before_k6 &&
        parser_after_k6.crc_errors == parser_before_k6.crc_errors &&
        parser_after_k6.length_errors == parser_before_k6.length_errors &&
        parser_after_k6.version_errors == parser_before_k6.version_errors &&
        parser_after_k6.unknown_type_errors == parser_before_k6.unknown_type_errors &&
        parser_after_k6.oversize_errors == parser_before_k6.oversize_errors &&
        parser_after_k6.escape_errors == parser_before_k6.escape_errors &&
        parser_after_k6.timeout_errors == parser_before_k6.timeout_errors;
    std::cout << (results.k6_uart_integrity ? "PASS" : "FAIL")
              << " K6 UART/parser/reset integrity; PWM edge density requires logic analyzer\n";

    host.SetPhase("K7a - Valid (+8192, 0) for 500 ms");
    if (!host.RunControlBlock(500U, 8192, 0)) {
        host.Close();
        return 6;
    }
    const auto k7_active = Point(host);

    host.SetPhase("K7b - Stop CONTROL_UPDATE for 350 ms; HEARTBEAT continues");
    if (!host.RunHeartbeatOnly(350U)) {
        host.Close();
        return 6;
    }
    const auto k7_stale = Point(host);
    results.k7_stale = k7_active.has_value() && k7_stale.has_value() &&
        k7_active->control_valid != 0U && k7_stale->control_valid == 0U &&
        k7_stale->link_state == ready_link_state;
    std::cout << (results.k7_stale ? "PASS" : "FAIL")
              << " K7 stale: link remains READY-equivalent and control_valid=0\n";

    host.SetPhase("K7c - Fresh center generation (0, 0) at 50 Hz for 1000 ms");
    if (!host.RunControlBlock(1000U, 0, 0)) {
        host.Close();
        return 6;
    }
    const auto k7_fresh = Point(host);
    results.k7_fresh_reenable = k7_fresh.has_value() && k7_fresh->control_valid != 0U &&
        Near(k7_fresh->pan, 0, options.center_tolerance_q15) &&
        Near(k7_fresh->tilt, 0, options.center_tolerance_q15);
    std::cout << (results.k7_fresh_reenable ? "PASS" : "FAIL")
              << " K7 fresh generation re-enable / center\n";

    host.SetPhase("Final heartbeat/status and contract accounting");
    if (!host.RunHeartbeatOnly(120U) || !host.LastStatus().has_value()) {
        host.Close();
        return 7;
    }
    final_errors = ExtractMcuErrors(*host.LastStatus());
    const auto final_parser = host.ParserStats();

    results.heartbeat_contract = host.HeartbeatsSent() == host.StatusReceived() &&
                                 host.MaxHeartbeatGapMs() <= 120U;
    results.session_contract = initial_boot_id != 0U &&
                               host.SenderBootId() == initial_boot_id &&
                               host.NextWireSequence() > 0U &&
                               host.PeerBootChanges() == 0U;
    results.final_error_counters = SameMcuErrors(baseline_errors, final_errors) &&
        final_parser.crc_errors == 0U && final_parser.length_errors == 0U &&
        final_parser.version_errors == 0U && final_parser.unknown_type_errors == 0U &&
        final_parser.oversize_errors == 0U && final_parser.escape_errors == 0U &&
        final_parser.timeout_errors == 0U;

    std::cout << "Session accounting: boot_id=" << host.SenderBootId()
              << " next_wire_sequence=" << host.NextWireSequence()
              << " heartbeat/status=" << host.HeartbeatsSent() << '/' << host.StatusReceived()
              << " controls=" << host.ControlsSent()
              << " max_active_control_gap_ms=" << host.MaxActiveControlGapMs()
              << " max_heartbeat_gap_ms=" << host.MaxHeartbeatGapMs() << '\n';

    host.Close();
    results.clean_close = true;

    PrintManualEvidenceChecklist();
    const bool report_ok = WriteReport(options, results, host, baseline_errors,
                                       final_errors, true);
    const bool automated_pass = AllAutomatedPass(results) && report_ok &&
                                !g_stop.load(std::memory_order_relaxed);

    std::cout << "\n=== AUTOMATED RESULT ===\n"
              << "hello=" << results.hello << '\n'
              << "heartbeat_contract=" << results.heartbeat_contract << '\n'
              << "session_contract=" << results.session_contract << '\n'
              << "k0_center=" << results.k0_center << '\n'
              << "k0_dead_zone=" << results.k0_dead_zone << '\n'
              << "k1_pan_positive=" << results.k1_pan_positive << '\n'
              << "k1_pan_return=" << results.k1_pan_return << '\n'
              << "k2_pan_negative=" << results.k2_pan_negative << '\n'
              << "k2_pan_return=" << results.k2_pan_return << '\n'
              << "k3_tilt_positive=" << results.k3_tilt_positive << '\n'
              << "k3_tilt_return=" << results.k3_tilt_return << '\n'
              << "k4_tilt_negative=" << results.k4_tilt_negative << '\n'
              << "k4_tilt_return=" << results.k4_tilt_return << '\n'
              << "k5_status_direction=" << results.k5_status_direction << '\n'
              << "k6_uart_integrity=" << results.k6_uart_integrity << '\n'
              << "k7_stale=" << results.k7_stale << '\n'
              << "k7_fresh_reenable=" << results.k7_fresh_reenable << '\n'
              << "final_error_counters=" << results.final_error_counters << '\n'
              << "clean_close=" << results.clean_close << '\n'
              << "AUTOMATED_RESULT=" << (automated_pass ? "PASS" : "FAIL") << '\n'
              << "FINAL_STEP_K requires the manual/logic-analyzer checklist above.\n";

    return automated_pass ? 0 : 8;
}