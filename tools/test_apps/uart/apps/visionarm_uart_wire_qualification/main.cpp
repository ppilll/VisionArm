#include "uart/protocol.h"
#include "uart/serial_port.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <time.h>
#include <unistd.h>

namespace {

using visionarm::uart::Ack;
using visionarm::uart::AppendU16Le;
using visionarm::uart::AppendU32Le;
using visionarm::uart::AppendU8;
using visionarm::uart::ClearRemoteStop;
using visionarm::uart::Crc16CcittFalse;
using visionarm::uart::DecodeMessage;
using visionarm::uart::EncodeMessage;
using visionarm::uart::Frame;
using visionarm::uart::FrameParser;
using visionarm::uart::Header;
using visionarm::uart::Heartbeat;
using visionarm::uart::Hello;
using visionarm::uart::HelloAck;
using visionarm::uart::Message;
using visionarm::uart::MessageType;
using visionarm::uart::Nack;
using visionarm::uart::RemoteStopRequest;
using visionarm::uart::SerialPort;
using visionarm::uart::SerialPortConfig;
using visionarm::uart::Status;
using visionarm::uart::kEscapeXor;
using visionarm::uart::kFrameEscape;
using visionarm::uart::kFrameFlag;
using visionarm::uart::kHeaderSize;
using visionarm::uart::kMaxPayloadSize;
using visionarm::uart::kProtocolVersion;

uint64_t MonotonicMs() noexcept {
    timespec value{};
    if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0U;
    }
    return static_cast<uint64_t>(value.tv_sec) * 1000ULL +
           static_cast<uint64_t>(value.tv_nsec) / 1000000ULL;
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
    std::string mode = "transaction-edge";
    std::string report_path;
    uint32_t baud = 115200U;
    uint32_t response_timeout_ms = 800U;
};

void PrintUsage(const char* program) {
    std::cout
        << "VisionArm real-board UART wire qualification\n\n"
        << "Usage:\n  " << program << " [options]\n\n"
        << "Options:\n"
        << "  --device PATH                 default /dev/ttyS3\n"
        << "  --baud N                      default 115200\n"
        << "  --mode transaction-edge|malformed\n"
        << "  --response-timeout-ms N       default 800\n"
        << "  --report FILE                 write JSON report\n"
        << "  --help\n\n"
        << "This tool is real-hardware only. It opens the physical Linux tty and\n"
        << "talks to the real MCU; it does not contain a PTY or MCU simulator.\n";
}

bool ParseArguments(int argc, char** argv, Options* options) {
    if (options == nullptr) {
        return false;
    }
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
        } else if (arg == "--response-timeout-ms") {
            if (!ParseU32(value, &options->response_timeout_ms)) return false;
        } else {
            std::cerr << "unknown option: " << arg << '\n';
            return false;
        }
    }
    return !options->device.empty() && options->baud != 0U &&
           options->response_timeout_ms != 0U &&
           (options->mode == "transaction-edge" ||
            options->mode == "malformed");
}

uint32_t GenerateBootId() noexcept {
    uint32_t value = static_cast<uint32_t>(MonotonicMs()) ^
                     static_cast<uint32_t>(::getpid()) ^ 0xA55AA55AU;
    if (value == 0U) {
        value = 1U;
    }
    return value;
}

void AppendEscaped(std::vector<uint8_t>* output, uint8_t value) {
    if (value == kFrameFlag || value == kFrameEscape) {
        output->push_back(kFrameEscape);
        output->push_back(static_cast<uint8_t>(value ^ kEscapeXor));
    } else {
        output->push_back(value);
    }
}

std::vector<uint8_t> EscapeRaw(const std::vector<uint8_t>& raw) {
    std::vector<uint8_t> encoded;
    encoded.reserve((2U * raw.size()) + 2U);
    encoded.push_back(kFrameFlag);
    for (const uint8_t value : raw) {
        AppendEscaped(&encoded, value);
    }
    encoded.push_back(kFrameFlag);
    return encoded;
}

std::vector<uint8_t> BuildRaw(uint8_t version,
                              uint8_t type,
                              uint16_t declared_payload_length,
                              uint32_t wire_sequence,
                              uint32_t boot_id,
                              uint32_t uptime_ms,
                              const std::vector<uint8_t>& actual_payload,
                              bool valid_crc) {
    std::vector<uint8_t> raw;
    raw.reserve(kHeaderSize + actual_payload.size() + 2U);
    AppendU8(&raw, version);
    AppendU8(&raw, type);
    AppendU16Le(&raw, declared_payload_length);
    AppendU32Le(&raw, wire_sequence);
    AppendU32Le(&raw, boot_id);
    AppendU32Le(&raw, uptime_ms);
    raw.insert(raw.end(), actual_payload.begin(), actual_payload.end());
    uint16_t crc = Crc16CcittFalse(raw.data(), raw.size());
    if (!valid_crc) {
        crc ^= 0x0001U;
    }
    AppendU16Le(&raw, crc);
    return raw;
}

bool WriteAll(SerialPort* serial,
              const std::vector<uint8_t>& bytes,
              uint32_t timeout_ms,
              std::string* error) {
    if (serial == nullptr || bytes.empty()) {
        if (error != nullptr) *error = "invalid WriteAll arguments";
        return false;
    }
    std::size_t offset = 0U;
    const uint64_t deadline = MonotonicMs() + timeout_ms;
    while (offset < bytes.size()) {
        const uint64_t now = MonotonicMs();
        if (now >= deadline) {
            if (error != nullptr) *error = "serial write timeout";
            return false;
        }
        pollfd descriptor{};
        descriptor.fd = serial->Fd();
        descriptor.events = POLLOUT;
        const uint64_t remaining_ms = deadline - now;
        const int wait_ms = static_cast<int>(std::min<uint64_t>(remaining_ms, 100U));
        const int poll_result = ::poll(&descriptor, 1U, wait_ms);
        if (poll_result < 0) {
            if (errno == EINTR) continue;
            if (error != nullptr) *error = "poll(POLLOUT) failed";
            return false;
        }
        if (poll_result == 0) continue;
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            if (error != nullptr) *error = "tty error while writing";
            return false;
        }
        if ((descriptor.revents & POLLOUT) == 0) continue;

        std::string write_error;
        const ssize_t written = serial->Write(bytes.data() + offset,
                                               bytes.size() - offset,
                                               &write_error);
        if (written > 0) {
            offset += static_cast<std::size_t>(written);
        } else if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                   errno != EINTR) {
            if (error != nullptr) *error = write_error;
            return false;
        }
    }
    if (error != nullptr) error->clear();
    return true;
}

class WireSession {
public:
    explicit WireSession(const Options& options)
        : options_(options), parser_(100U), boot_id_(GenerateBootId()) {}

    bool Open(std::string* error) {
        SerialPortConfig config;
        config.device_path = options_.device;
        config.baud_rate = options_.baud;
        config.exclusive = true;
        config.flush_on_open = true;
        return serial_.Open(config, error);
    }

    uint32_t NextSequence() noexcept {
        return sequence_++;
    }

    Header MakeHeader(MessageType type) noexcept {
        Header header;
        header.protocol_version = kProtocolVersion;
        header.message_type = type;
        header.wire_sequence = NextSequence();
        header.sender_boot_id = boot_id_;
        header.sender_uptime_ms = static_cast<uint32_t>(MonotonicMs() - start_ms_);
        return header;
    }

    bool SendMessage(const Message& message,
                     std::vector<uint8_t>* encoded_copy,
                     std::string* error) {
        std::vector<uint8_t> encoded;
        if (!EncodeMessage(message, &encoded, error)) {
            return false;
        }
        if (encoded_copy != nullptr) {
            *encoded_copy = encoded;
        }
        return WriteAll(&serial_, encoded, options_.response_timeout_ms, error);
    }

    bool SendBytes(const std::vector<uint8_t>& bytes, std::string* error) {
        return WriteAll(&serial_, bytes, options_.response_timeout_ms, error);
    }

    std::optional<Message> WaitMessage(MessageType expected,
                                       uint32_t timeout_ms,
                                       std::string* error) {
        const uint64_t deadline = MonotonicMs() + timeout_ms;
        while (MonotonicMs() < deadline) {
            const uint64_t now = MonotonicMs();
            pollfd descriptor{};
            descriptor.fd = serial_.Fd();
            descriptor.events = POLLIN;
            const uint64_t remaining_ms = deadline - now;
            const int wait_ms = static_cast<int>(std::min<uint64_t>(remaining_ms, 100U));
            const int poll_result = ::poll(&descriptor, 1U, wait_ms);
            if (poll_result < 0) {
                if (errno == EINTR) continue;
                if (error != nullptr) *error = "poll(POLLIN) failed";
                return std::nullopt;
            }
            if (poll_result == 0) {
                parser_.Tick(MonotonicMs());
                continue;
            }
            if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                if (error != nullptr) *error = "tty error while reading";
                return std::nullopt;
            }
            if ((descriptor.revents & POLLIN) == 0) continue;

            std::array<uint8_t, 512U> buffer{};
            std::string read_error;
            const ssize_t read_size = serial_.Read(buffer.data(), buffer.size(),
                                                   &read_error);
            if (read_size > 0) {
                std::vector<Frame> frames;
                parser_.Feed(buffer.data(), static_cast<std::size_t>(read_size),
                             MonotonicMs(), &frames);
                for (const Frame& frame : frames) {
                    Message message;
                    std::string decode_error;
                    if (!DecodeMessage(frame, &message, &decode_error)) {
                        continue;
                    }
                    if (message.header.message_type == expected) {
                        if (error != nullptr) error->clear();
                        return message;
                    }
                }
            } else if (read_size < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                       errno != EINTR) {
                if (error != nullptr) *error = read_error;
                return std::nullopt;
            }
        }
        if (error != nullptr) *error = "response timeout";
        return std::nullopt;
    }

    bool Handshake(std::string* error) {
        Hello hello;
        hello.device_role = 1U;
        hello.minimum_protocol_version = kProtocolVersion;
        hello.maximum_protocol_version = kProtocolVersion;
        hello.max_payload = static_cast<uint16_t>(kMaxPayloadSize);
        hello.max_control_rate_hz = 30U;
        hello.software_version_major = 5U;
        hello.software_version_minor = 3U;
        hello.software_version_patch = 0U;

        const Header header = MakeHeader(MessageType::HELLO);
        if (!SendMessage(Message{header, hello}, nullptr, error)) {
            return false;
        }
        const auto response = WaitMessage(MessageType::HELLO_ACK,
                                          options_.response_timeout_ms, error);
        if (!response.has_value()) {
            return false;
        }
        const auto ack = std::get<HelloAck>(response->body);
        if (ack.hello_wire_sequence != header.wire_sequence ||
            ack.accepted == 0U ||
            ack.selected_protocol_version != kProtocolVersion ||
            ack.device_role != 2U) {
            if (error != nullptr) *error = "HELLO_ACK contract mismatch";
            return false;
        }
        peer_boot_id_ = response->header.sender_boot_id;
        std::cout << "HELLO_ACK peer_boot=" << peer_boot_id_ << '\n';
        return true;
    }

    std::optional<Status> HeartbeatStatus(std::string* error) {
        const Header header = MakeHeader(MessageType::HEARTBEAT);
        if (!SendMessage(Message{header, Heartbeat{}}, nullptr, error)) {
            return std::nullopt;
        }
        const auto response = WaitMessage(MessageType::STATUS,
                                          options_.response_timeout_ms, error);
        if (!response.has_value()) {
            return std::nullopt;
        }
        return std::get<Status>(response->body);
    }

    uint32_t BootId() const noexcept { return boot_id_; }
    uint32_t UptimeMs() const noexcept {
        return static_cast<uint32_t>(MonotonicMs() - start_ms_);
    }

private:
    Options options_;
    SerialPort serial_;
    FrameParser parser_;
    uint32_t boot_id_ = 0U;
    uint32_t peer_boot_id_ = 0U;
    uint32_t sequence_ = 0U;
    uint64_t start_ms_ = MonotonicMs();
};

bool IsMatchingAck(const Message& message,
                   MessageType request_type,
                   uint32_t transaction_id) {
    if (message.header.message_type != MessageType::ACK) {
        return false;
    }
    const Ack& ack = std::get<Ack>(message.body);
    return ack.acked_message_type == static_cast<uint8_t>(request_type) &&
           ack.transaction_id == transaction_id;
}

bool IsMatchingNack(const Message& message,
                    MessageType request_type,
                    uint32_t transaction_id) {
    if (message.header.message_type != MessageType::NACK) {
        return false;
    }
    const Nack& nack = std::get<Nack>(message.body);
    return nack.acked_message_type == static_cast<uint8_t>(request_type) &&
           nack.transaction_id == transaction_id;
}

struct TransactionResult {
    bool initial_stop_ack = false;
    bool stop_latched = false;
    bool duplicate_stop_ack = false;
    bool conflicting_duplicate_nack = false;
    bool clear_ack = false;
    bool clear_observed = false;
};

TransactionResult RunTransactionEdge(WireSession* session,
                                     const Options& options) {
    TransactionResult result;
    constexpr uint32_t transaction_id = 0x10203040U;
    constexpr uint32_t clear_transaction_id = 0x10203041U;

    Header stop_header = session->MakeHeader(MessageType::REMOTE_STOP_REQUEST);
    Message stop_message{stop_header, RemoteStopRequest{transaction_id, 1U}};
    std::vector<uint8_t> exact_stop_frame;
    std::string error;
    if (!session->SendMessage(stop_message, &exact_stop_frame, &error)) {
        std::cerr << "initial stop send failed: " << error << '\n';
    } else {
        auto response = session->WaitMessage(MessageType::ACK,
                                             options.response_timeout_ms, &error);
        result.initial_stop_ack = response.has_value() &&
            IsMatchingAck(*response, MessageType::REMOTE_STOP_REQUEST,
                          transaction_id);
        std::cout << "initial_stop_ack="
                  << (result.initial_stop_ack ? "yes" : "no") << '\n';
    }

    if (result.initial_stop_ack) {
        // Retry semantics require the exact duplicate to be adjacent: do not
        // advance the Linux wire sequence with a heartbeat before replaying
        // the original encoded reliable frame.
        if (!session->SendBytes(exact_stop_frame, &error)) {
            std::cerr << "duplicate stop send failed: " << error << '\n';
        } else {
            const auto response = session->WaitMessage(
                MessageType::ACK, options.response_timeout_ms, &error);
            result.duplicate_stop_ack = response.has_value() &&
                IsMatchingAck(*response, MessageType::REMOTE_STOP_REQUEST,
                              transaction_id);
        }
        std::cout << "duplicate_stop_ack="
                  << (result.duplicate_stop_ack ? "yes" : "no") << '\n';

        const auto status = session->HeartbeatStatus(&error);
        result.stop_latched =
            status.has_value() && status->remote_stop_latched != 0U;
        std::cout << "stop_latched="
                  << (result.stop_latched ? "yes" : "no") << '\n';

        Header conflict_header =
            session->MakeHeader(MessageType::REMOTE_STOP_REQUEST);
        Message conflict_message{
            conflict_header, RemoteStopRequest{transaction_id, 2U}};
        if (!session->SendMessage(conflict_message, nullptr, &error)) {
            std::cerr << "conflicting duplicate send failed: " << error << '\n';
        } else {
            const auto response = session->WaitMessage(
                MessageType::NACK, options.response_timeout_ms, &error);
            result.conflicting_duplicate_nack = response.has_value() &&
                IsMatchingNack(*response, MessageType::REMOTE_STOP_REQUEST,
                               transaction_id);
        }
        std::cout << "conflicting_duplicate_nack="
                  << (result.conflicting_duplicate_nack ? "yes" : "no")
                  << '\n';
    }

    // Safety cleanup is unconditional once the session is alive. Even if an
    // intermediate duplicate/conflict assertion fails, do not leave the MCU
    // remote-stop latch set merely because the qualification program exits.
    const Header clear_header = session->MakeHeader(MessageType::CLEAR_REMOTE_STOP);
    if (!session->SendMessage(
            Message{clear_header, ClearRemoteStop{clear_transaction_id}},
            nullptr, &error)) {
        std::cerr << "clear send failed: " << error << '\n';
    } else {
        const auto response = session->WaitMessage(
            MessageType::ACK, options.response_timeout_ms, &error);
        result.clear_ack = response.has_value() &&
            IsMatchingAck(*response, MessageType::CLEAR_REMOTE_STOP,
                          clear_transaction_id);
    }
    std::cout << "clear_ack=" << (result.clear_ack ? "yes" : "no") << '\n';

    const auto final_status = session->HeartbeatStatus(&error);
    result.clear_observed =
        final_status.has_value() && final_status->remote_stop_latched == 0U;
    std::cout << "clear_observed="
              << (result.clear_observed ? "yes" : "no") << '\n';
    return result;
}

struct MalformedResult {
    bool crc_recovered = false;
    bool length_recovered = false;
    bool version_recovered = false;
    bool type_recovered = false;
    bool escape_recovered = false;
    bool oversize_recovered = false;
    bool noise_recovered = false;
    bool dropped_byte_recovered = false;
    bool duplicate_byte_recovered = false;
    bool timeout_recovered = false;
    uint32_t crc_delta = 0U;
    uint32_t length_delta = 0U;
    uint32_t version_delta = 0U;
    uint32_t type_delta = 0U;
    uint32_t overflow_delta = 0U;
};

uint32_t Delta32(uint32_t before, uint32_t after) noexcept {
    return after - before;
}

bool SendFaultAndRecover(WireSession* session,
                         const std::vector<uint8_t>& fault,
                         uint32_t settle_ms,
                         Status* status,
                         std::string* error) {
    if (!session->SendBytes(fault, error)) {
        return false;
    }
    if (settle_ms != 0U) {
        std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
    }
    const auto recovered = session->HeartbeatStatus(error);
    if (!recovered.has_value()) {
        return false;
    }
    if (status != nullptr) {
        *status = *recovered;
    }
    return true;
}

MalformedResult RunMalformed(WireSession* session) {
    MalformedResult result;
    std::string error;
    const auto baseline_opt = session->HeartbeatStatus(&error);
    if (!baseline_opt.has_value()) {
        std::cerr << "cannot get baseline STATUS: " << error << '\n';
        return result;
    }
    const Status baseline = *baseline_opt;
    Status current = baseline;

    auto make_heartbeat_raw = [&](uint8_t version,
                                  uint8_t type,
                                  uint16_t declared_length,
                                  bool valid_crc) {
        return BuildRaw(version, type, declared_length,
                        session->NextSequence(), session->BootId(),
                        session->UptimeMs(), {}, valid_crc);
    };

    result.crc_recovered = SendFaultAndRecover(
        session,
        EscapeRaw(make_heartbeat_raw(kProtocolVersion,
                                     static_cast<uint8_t>(MessageType::HEARTBEAT),
                                     0U, false)),
        20U, &current, &error);
    result.crc_delta = Delta32(baseline.rx_crc_error_count,
                               current.rx_crc_error_count);
    std::cout << "fault=crc recovered=" << result.crc_recovered
              << " delta=" << result.crc_delta << '\n';

    const uint32_t length_before = current.rx_length_error_count;
    result.length_recovered = SendFaultAndRecover(
        session,
        EscapeRaw(make_heartbeat_raw(kProtocolVersion,
                                     static_cast<uint8_t>(MessageType::HEARTBEAT),
                                     1U, true)),
        20U, &current, &error);
    result.length_delta = Delta32(length_before, current.rx_length_error_count);
    std::cout << "fault=length recovered=" << result.length_recovered
              << " delta=" << result.length_delta << '\n';

    const uint32_t version_before = current.rx_version_error_count;
    result.version_recovered = SendFaultAndRecover(
        session,
        EscapeRaw(make_heartbeat_raw(2U,
                                     static_cast<uint8_t>(MessageType::HEARTBEAT),
                                     0U, true)),
        20U, &current, &error);
    result.version_delta = Delta32(version_before, current.rx_version_error_count);
    std::cout << "fault=version recovered=" << result.version_recovered
              << " delta=" << result.version_delta << '\n';

    const uint32_t type_before = current.rx_unknown_type_count;
    result.type_recovered = SendFaultAndRecover(
        session,
        EscapeRaw(make_heartbeat_raw(kProtocolVersion, 0xFEU, 0U, true)),
        20U, &current, &error);
    result.type_delta = Delta32(type_before, current.rx_unknown_type_count);
    std::cout << "fault=type recovered=" << result.type_recovered
              << " delta=" << result.type_delta << '\n';

    const std::vector<uint8_t> bad_escape = {
        kFrameFlag, kFrameEscape, 0x00U, kFrameFlag};
    result.escape_recovered = SendFaultAndRecover(
        session, bad_escape, 20U, &current, &error);
    std::cout << "fault=bad_escape recovered=" << result.escape_recovered << '\n';

    const uint32_t overflow_before = current.rx_overflow_count;
    std::vector<uint8_t> oversize;
    oversize.reserve(151U);
    oversize.push_back(kFrameFlag);
    for (std::size_t index = 0U; index < 149U; ++index) {
        oversize.push_back(0x11U);
    }
    oversize.push_back(kFrameFlag);
    result.oversize_recovered = SendFaultAndRecover(
        session, oversize, 20U, &current, &error);
    result.overflow_delta = Delta32(overflow_before, current.rx_overflow_count);
    std::cout << "fault=oversize recovered=" << result.oversize_recovered
              << " overflow_delta=" << result.overflow_delta << '\n';

    const std::vector<uint8_t> noise = {0x11U, 0x22U, 0x33U, 0x44U, 0x55U};
    result.noise_recovered = SendFaultAndRecover(
        session, noise, 20U, &current, &error);
    std::cout << "fault=noise recovered=" << result.noise_recovered << '\n';

    std::vector<uint8_t> dropped_raw = make_heartbeat_raw(
        kProtocolVersion, static_cast<uint8_t>(MessageType::HEARTBEAT), 0U, true);
    if (dropped_raw.size() > 8U) {
        dropped_raw.erase(dropped_raw.begin() + 7);
    }
    result.dropped_byte_recovered = SendFaultAndRecover(
        session, EscapeRaw(dropped_raw), 20U, &current, &error);
    std::cout << "fault=dropped_byte recovered="
              << result.dropped_byte_recovered << '\n';

    std::vector<uint8_t> duplicate_raw = make_heartbeat_raw(
        kProtocolVersion, static_cast<uint8_t>(MessageType::HEARTBEAT), 0U, true);
    if (duplicate_raw.size() > 8U) {
        duplicate_raw.insert(duplicate_raw.begin() + 7, duplicate_raw[7]);
    }
    result.duplicate_byte_recovered = SendFaultAndRecover(
        session, EscapeRaw(duplicate_raw), 20U, &current, &error);
    std::cout << "fault=duplicate_byte recovered="
              << result.duplicate_byte_recovered << '\n';

    const std::vector<uint8_t> partial = {
        kFrameFlag, kProtocolVersion,
        static_cast<uint8_t>(MessageType::HEARTBEAT), 0x00U};
    result.timeout_recovered = SendFaultAndRecover(
        session, partial, 180U, &current, &error);
    std::cout << "fault=assembly_timeout recovered="
              << result.timeout_recovered << '\n';

    return result;
}

bool TransactionPass(const TransactionResult& result) noexcept {
    return result.initial_stop_ack && result.stop_latched &&
           result.duplicate_stop_ack && result.conflicting_duplicate_nack &&
           result.clear_ack && result.clear_observed;
}

bool MalformedPass(const MalformedResult& result) noexcept {
    return result.crc_recovered && result.length_recovered &&
           result.version_recovered && result.type_recovered &&
           result.escape_recovered && result.oversize_recovered &&
           result.noise_recovered && result.dropped_byte_recovered &&
           result.duplicate_byte_recovered && result.timeout_recovered &&
           result.crc_delta > 0U && result.length_delta > 0U &&
           result.version_delta > 0U && result.type_delta > 0U;
}

bool WriteReport(const Options& options,
                 bool handshake_pass,
                 bool pass,
                 const TransactionResult& transaction,
                 const MalformedResult& malformed,
                 std::string* error) {
    if (options.report_path.empty()) {
        return true;
    }
    std::error_code ec;
    const std::filesystem::path path(options.report_path);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            if (error != nullptr) *error = ec.message();
            return false;
        }
    }
    std::ofstream out(path);
    if (!out) {
        if (error != nullptr) *error = "cannot open report file";
        return false;
    }
    out << "{\n"
        << "  \"mode\": \"" << options.mode << "\",\n"
        << "  \"device\": \"" << options.device << "\",\n"
        << "  \"baud\": " << options.baud << ",\n"
        << "  \"handshake_pass\": " << (handshake_pass ? "true" : "false") << ",\n"
        << "  \"pass\": " << (pass ? "true" : "false") << ",\n"
        << "  \"transaction_edge\": {\n"
        << "    \"initial_stop_ack\": " << (transaction.initial_stop_ack ? "true" : "false") << ",\n"
        << "    \"stop_latched\": " << (transaction.stop_latched ? "true" : "false") << ",\n"
        << "    \"duplicate_stop_ack\": " << (transaction.duplicate_stop_ack ? "true" : "false") << ",\n"
        << "    \"conflicting_duplicate_nack\": " << (transaction.conflicting_duplicate_nack ? "true" : "false") << ",\n"
        << "    \"clear_ack\": " << (transaction.clear_ack ? "true" : "false") << ",\n"
        << "    \"clear_observed\": " << (transaction.clear_observed ? "true" : "false") << "\n"
        << "  },\n"
        << "  \"malformed\": {\n"
        << "    \"crc_recovered\": " << (malformed.crc_recovered ? "true" : "false") << ",\n"
        << "    \"length_recovered\": " << (malformed.length_recovered ? "true" : "false") << ",\n"
        << "    \"version_recovered\": " << (malformed.version_recovered ? "true" : "false") << ",\n"
        << "    \"type_recovered\": " << (malformed.type_recovered ? "true" : "false") << ",\n"
        << "    \"escape_recovered\": " << (malformed.escape_recovered ? "true" : "false") << ",\n"
        << "    \"oversize_recovered\": " << (malformed.oversize_recovered ? "true" : "false") << ",\n"
        << "    \"noise_recovered\": " << (malformed.noise_recovered ? "true" : "false") << ",\n"
        << "    \"dropped_byte_recovered\": " << (malformed.dropped_byte_recovered ? "true" : "false") << ",\n"
        << "    \"duplicate_byte_recovered\": " << (malformed.duplicate_byte_recovered ? "true" : "false") << ",\n"
        << "    \"timeout_recovered\": " << (malformed.timeout_recovered ? "true" : "false") << ",\n"
        << "    \"crc_delta\": " << malformed.crc_delta << ",\n"
        << "    \"length_delta\": " << malformed.length_delta << ",\n"
        << "    \"version_delta\": " << malformed.version_delta << ",\n"
        << "    \"type_delta\": " << malformed.type_delta << ",\n"
        << "    \"overflow_delta\": " << malformed.overflow_delta << "\n"
        << "  }\n"
        << "}\n";
    if (error != nullptr) error->clear();
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseArguments(argc, argv, &options)) {
        PrintUsage(argv[0]);
        return 2;
    }

    WireSession session(options);
    std::string error;
    if (!session.Open(&error)) {
        std::cerr << "open failed: " << error << '\n';
        return 3;
    }

    const bool handshake_pass = session.Handshake(&error);
    if (!handshake_pass) {
        std::cerr << "handshake failed: " << error << '\n';
        TransactionResult transaction;
        MalformedResult malformed;
        std::string report_error;
        (void)WriteReport(options, false, false, transaction, malformed,
                          &report_error);
        return 4;
    }

    TransactionResult transaction;
    MalformedResult malformed;
    bool pass = false;
    if (options.mode == "transaction-edge") {
        transaction = RunTransactionEdge(&session, options);
        pass = TransactionPass(transaction);
    } else {
        malformed = RunMalformed(&session);
        pass = MalformedPass(malformed);
    }

    std::string report_error;
    if (!WriteReport(options, true, pass, transaction, malformed,
                     &report_error)) {
        std::cerr << "report failed: " << report_error << '\n';
        return 5;
    }

    std::cout << "RESULT=" << (pass ? "PASS" : "FAIL") << '\n';
    return pass ? 0 : 6;
}
