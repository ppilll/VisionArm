#include "control/uart_protocol.h"

#include <algorithm>

namespace visionarm::uart {

uint16_t Crc16CcittFalse(const uint8_t* data, std::size_t size) noexcept {
    uint16_t crc = 0xFFFFU;
    if (data == nullptr && size != 0U) {
        return crc;
    }

    for (std::size_t index = 0U; index < size; ++index) {
        crc ^= static_cast<uint16_t>(
            static_cast<uint16_t>(data[index]) << 8U);
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

}  // namespace visionarm::uart

#include <algorithm>

namespace visionarm::uart {
namespace {

void AppendEscaped(std::vector<uint8_t>* output, uint8_t value) {
    if (value == kFrameFlag || value == kFrameEscape) {
        output->push_back(kFrameEscape);
        output->push_back(static_cast<uint8_t>(value ^ kEscapeXor));
        return;
    }
    output->push_back(value);
}

void SetError(std::string* error, const char* message) {
    if (error != nullptr) {
        *error = message;
    }
}

}  // namespace

bool EncodeFrame(const Header& header,
                 const uint8_t* payload,
                 std::size_t payload_size,
                 std::vector<uint8_t>* encoded,
                 std::string* error) {
    if (encoded == nullptr) {
        SetError(error, "encoded output is null");
        return false;
    }
    if (payload_size > kMaxPayloadSize) {
        SetError(error, "payload exceeds V1 maximum");
        return false;
    }
    if (payload == nullptr && payload_size != 0U) {
        SetError(error, "non-empty payload pointer is null");
        return false;
    }
    if (header.protocol_version != kProtocolVersion) {
        SetError(error, "unsupported protocol version for encoder");
        return false;
    }
    if (!IsKnownMessageType(static_cast<uint8_t>(header.message_type))) {
        SetError(error, "unknown message type for encoder");
        return false;
    }

    std::vector<uint8_t> raw;
    raw.reserve(kHeaderSize + payload_size + kCrcSize);
    AppendU8(&raw, header.protocol_version);
    AppendU8(&raw, static_cast<uint8_t>(header.message_type));
    AppendU16Le(&raw, static_cast<uint16_t>(payload_size));
    AppendU32Le(&raw, header.wire_sequence);
    AppendU32Le(&raw, header.sender_boot_id);
    AppendU32Le(&raw, header.sender_uptime_ms);
    if (payload_size != 0U) {
        raw.insert(raw.end(), payload, payload + payload_size);
    }

    const uint16_t crc = Crc16CcittFalse(raw.data(), raw.size());
    AppendU16Le(&raw, crc);

    encoded->clear();
    encoded->reserve((2U * raw.size()) + 2U);
    encoded->push_back(kFrameFlag);
    for (const uint8_t value : raw) {
        AppendEscaped(encoded, value);
    }
    encoded->push_back(kFrameFlag);

    if (encoded->size() > kMaxEncodedFrameSize) {
        SetError(error, "encoded frame exceeded computed maximum");
        encoded->clear();
        return false;
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool DecodeRawFrame(const uint8_t* raw,
                    std::size_t raw_size,
                    Frame* frame,
                    FrameError* frame_error) noexcept {
    if (frame_error != nullptr) {
        *frame_error = FrameError::NONE;
    }
    if (raw == nullptr || frame == nullptr || raw_size < kHeaderSize + kCrcSize) {
        if (frame_error != nullptr) {
            *frame_error = FrameError::TOO_SHORT;
        }
        return false;
    }
    if (raw_size > kMaxRawFrameSize) {
        if (frame_error != nullptr) {
            *frame_error = FrameError::OVERSIZE;
        }
        return false;
    }

    std::size_t offset = 0U;
    uint8_t version = 0U;
    uint8_t type = 0U;
    uint16_t payload_length = 0U;
    uint32_t wire_sequence = 0U;
    uint32_t boot_id = 0U;
    uint32_t uptime_ms = 0U;

    if (!ReadU8(raw, raw_size, &offset, &version) ||
        !ReadU8(raw, raw_size, &offset, &type) ||
        !ReadU16Le(raw, raw_size, &offset, &payload_length) ||
        !ReadU32Le(raw, raw_size, &offset, &wire_sequence) ||
        !ReadU32Le(raw, raw_size, &offset, &boot_id) ||
        !ReadU32Le(raw, raw_size, &offset, &uptime_ms)) {
        if (frame_error != nullptr) {
            *frame_error = FrameError::TOO_SHORT;
        }
        return false;
    }

    const std::size_t expected_size =
        kHeaderSize + static_cast<std::size_t>(payload_length) + kCrcSize;
    if (payload_length > kMaxPayloadSize || raw_size != expected_size) {
        if (frame_error != nullptr) {
            *frame_error = FrameError::LENGTH;
        }
        return false;
    }

    std::size_t crc_offset = raw_size - kCrcSize;
    uint16_t received_crc = 0U;
    if (!ReadU16Le(raw, raw_size, &crc_offset, &received_crc)) {
        if (frame_error != nullptr) {
            *frame_error = FrameError::LENGTH;
        }
        return false;
    }
    const uint16_t computed_crc =
        Crc16CcittFalse(raw, raw_size - kCrcSize);
    if (received_crc != computed_crc) {
        if (frame_error != nullptr) {
            *frame_error = FrameError::CRC;
        }
        return false;
    }
    if (version != kProtocolVersion) {
        if (frame_error != nullptr) {
            *frame_error = FrameError::VERSION;
        }
        return false;
    }
    if (!IsKnownMessageType(type)) {
        if (frame_error != nullptr) {
            *frame_error = FrameError::UNKNOWN_TYPE;
        }
        return false;
    }

    frame->header.protocol_version = version;
    frame->header.message_type = static_cast<MessageType>(type);
    frame->header.payload_length = payload_length;
    frame->header.wire_sequence = wire_sequence;
    frame->header.sender_boot_id = boot_id;
    frame->header.sender_uptime_ms = uptime_ms;
    frame->payload_size = payload_length;
    std::fill(frame->payload.begin(), frame->payload.end(), 0U);
    std::copy_n(raw + kHeaderSize, payload_length, frame->payload.begin());
    return true;
}

}  // namespace visionarm::uart

namespace visionarm::uart {

FrameParser::FrameParser(uint64_t assembly_timeout_ms) noexcept
    : assembly_timeout_ms_(assembly_timeout_ms) {}

void FrameParser::Feed(const uint8_t* data,
                       std::size_t size,
                       uint64_t now_ms,
                       std::vector<Frame>* frames) noexcept {
    if (data == nullptr || frames == nullptr) {
        return;
    }

    Tick(now_ms);
    for (std::size_t index = 0U; index < size; ++index) {
        const uint8_t value = data[index];
        ++stats_.input_bytes;

        if (value == kFrameFlag) {
            HandleFlag(frames);
            continue;
        }

        if (!collecting_) {
            ++stats_.noise_bytes;
            continue;
        }

        has_activity_time_ = true;
        last_activity_ms_ = now_ms;

        if (discard_until_flag_) {
            continue;
        }

        if (escape_pending_) {
            escape_pending_ = false;
            if (value != static_cast<uint8_t>(kFrameFlag ^ kEscapeXor) &&
                value != static_cast<uint8_t>(kFrameEscape ^ kEscapeXor)) {
                ++stats_.escape_errors;
                DiscardUntilFlag();
                continue;
            }
            AppendRaw(static_cast<uint8_t>(value ^ kEscapeXor));
            continue;
        }

        if (value == kFrameEscape) {
            escape_pending_ = true;
            continue;
        }

        AppendRaw(value);
    }
}

void FrameParser::Tick(uint64_t now_ms) noexcept {
    if (!collecting_ || !has_activity_time_ || assembly_timeout_ms_ == 0U) {
        return;
    }
    if (now_ms >= last_activity_ms_ &&
        now_ms - last_activity_ms_ > assembly_timeout_ms_) {
        ++stats_.timeout_errors;
        ResetCandidate(false);
    }
}

void FrameParser::Reset() noexcept {
    stats_ = ParserStats{};
    ResetCandidate(false);
}

void FrameParser::HandleFlag(std::vector<Frame>* frames) noexcept {
    if (!collecting_) {
        ResetCandidate(true);
        return;
    }

    if (escape_pending_) {
        ++stats_.escape_errors;
        ResetCandidate(true);
        return;
    }

    if (discard_until_flag_) {
        ResetCandidate(true);
        return;
    }

    if (raw_size_ == 0U) {
        ++stats_.empty_frames;
        return;
    }

    Frame frame;
    FrameError error = FrameError::NONE;
    if (DecodeRawFrame(raw_.data(), raw_size_, &frame, &error)) {
        frames->push_back(frame);
        ++stats_.valid_frames;
    } else {
        AccountFrameError(error);
    }

    ResetCandidate(true);
}

void FrameParser::AppendRaw(uint8_t value) noexcept {
    if (raw_size_ >= raw_.size()) {
        ++stats_.oversize_errors;
        DiscardUntilFlag();
        return;
    }
    raw_[raw_size_] = value;
    ++raw_size_;
}

void FrameParser::DiscardUntilFlag() noexcept {
    discard_until_flag_ = true;
    escape_pending_ = false;
    raw_size_ = 0U;
}

void FrameParser::ResetCandidate(bool collecting) noexcept {
    collecting_ = collecting;
    escape_pending_ = false;
    discard_until_flag_ = false;
    has_activity_time_ = false;
    last_activity_ms_ = 0U;
    raw_size_ = 0U;
}

void FrameParser::AccountFrameError(FrameError error) noexcept {
    switch (error) {
        case FrameError::CRC:
            ++stats_.crc_errors;
            break;
        case FrameError::VERSION:
            ++stats_.version_errors;
            break;
        case FrameError::UNKNOWN_TYPE:
            ++stats_.unknown_type_errors;
            break;
        case FrameError::OVERSIZE:
            ++stats_.oversize_errors;
            break;
        case FrameError::ESCAPE:
            ++stats_.escape_errors;
            break;
        case FrameError::TIMEOUT:
            ++stats_.timeout_errors;
            break;
        case FrameError::TOO_SHORT:
        case FrameError::LENGTH:
            ++stats_.length_errors;
            break;
        case FrameError::NONE:
            break;
    }
}

}  // namespace visionarm::uart


#include "control/uart_protocol.h"

#include <type_traits>

namespace visionarm::uart {
namespace {

constexpr std::size_t kHelloPayloadSize = 20U;
constexpr std::size_t kHelloAckPayloadSize = 24U;
constexpr std::size_t kHeartbeatPayloadSize = 0U;
constexpr std::size_t kControlUpdatePayloadSize = 26U;
constexpr std::size_t kStatusPayloadSize = 52U;
constexpr std::size_t kRemoteStopRequestPayloadSize = 8U;
constexpr std::size_t kClearRemoteStopPayloadSize = 4U;
constexpr std::size_t kAckPayloadSize = 12U;
constexpr std::size_t kPingPayloadSize = 4U;
constexpr std::size_t kPongPayloadSize = 8U;

void SetPayloadError(std::string* error, const char* message) {
    if (error != nullptr) {
        *error = message;
    }
}

void AppendReservedU8(std::vector<uint8_t>* payload) {
    AppendU8(payload, 0U);
}

void AppendReservedU16(std::vector<uint8_t>* payload) {
    AppendU16Le(payload, 0U);
}

bool ReadReservedU8(const uint8_t* data,
                    std::size_t size,
                    std::size_t* offset) noexcept {
    uint8_t reserved = 0U;
    return ReadU8(data, size, offset, &reserved) && reserved == 0U;
}

bool ReadReservedU16(const uint8_t* data,
                     std::size_t size,
                     std::size_t* offset) noexcept {
    uint16_t reserved = 0U;
    return ReadU16Le(data, size, offset, &reserved) && reserved == 0U;
}

bool RequirePayloadSize(std::size_t actual,
                        std::size_t expected,
                        std::string* error) {
    if (actual != expected) {
        SetPayloadError(error, "payload size does not match message contract");
        return false;
    }
    return true;
}

bool DecodeAckLike(const uint8_t* payload,
                   std::size_t payload_size,
                   uint8_t* acked_message_type,
                   uint16_t* result_code,
                   uint32_t* transaction_id,
                   uint16_t* detail_code,
                   std::string* error) {
    if (!RequirePayloadSize(payload_size, kAckPayloadSize, error)) {
        return false;
    }
    std::size_t offset = 0U;
    if (!ReadU8(payload, payload_size, &offset, acked_message_type) ||
        !ReadReservedU8(payload, payload_size, &offset) ||
        !ReadU16Le(payload, payload_size, &offset, result_code) ||
        !ReadU32Le(payload, payload_size, &offset, transaction_id) ||
        !ReadU16Le(payload, payload_size, &offset, detail_code) ||
        !ReadReservedU16(payload, payload_size, &offset)) {
        SetPayloadError(error, "ACK/NACK payload decode or reserved-field validation failed");
        return false;
    }
    return true;
}

}  // namespace

MessageType MessageTypeOf(const MessageBody& body) noexcept {
    return std::visit(
        [](const auto& value) noexcept -> MessageType {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, Hello>) return MessageType::HELLO;
            if constexpr (std::is_same_v<T, HelloAck>) return MessageType::HELLO_ACK;
            if constexpr (std::is_same_v<T, Heartbeat>) return MessageType::HEARTBEAT;
            if constexpr (std::is_same_v<T, ControlUpdate>) return MessageType::CONTROL_UPDATE;
            if constexpr (std::is_same_v<T, Status>) return MessageType::STATUS;
            if constexpr (std::is_same_v<T, RemoteStopRequest>) return MessageType::REMOTE_STOP_REQUEST;
            if constexpr (std::is_same_v<T, ClearRemoteStop>) return MessageType::CLEAR_REMOTE_STOP;
            if constexpr (std::is_same_v<T, Ack>) return MessageType::ACK;
            if constexpr (std::is_same_v<T, Nack>) return MessageType::NACK;
            if constexpr (std::is_same_v<T, Ping>) return MessageType::PING;
            return MessageType::PONG;
        },
        body);
}

bool EncodePayload(const MessageBody& body,
                   std::vector<uint8_t>* payload,
                   std::string* error) {
    if (payload == nullptr) {
        SetPayloadError(error, "payload output is null");
        return false;
    }
    payload->clear();
    payload->reserve(kMaxPayloadSize);

    const bool ok = std::visit(
        [payload, error](const auto& value) -> bool {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, Hello>) {
                AppendU8(payload, value.device_role);
                AppendU8(payload, value.minimum_protocol_version);
                AppendU8(payload, value.maximum_protocol_version);
                AppendReservedU8(payload);
                AppendU16Le(payload, value.max_payload);
                AppendU16Le(payload, value.max_control_rate_hz);
                AppendU32Le(payload, value.capability_bits);
                AppendU16Le(payload, value.software_version_major);
                AppendU16Le(payload, value.software_version_minor);
                AppendU16Le(payload, value.software_version_patch);
                AppendReservedU16(payload);
            } else if constexpr (std::is_same_v<T, HelloAck>) {
                AppendU32Le(payload, value.hello_wire_sequence);
                AppendU8(payload, value.accepted);
                AppendU8(payload, value.selected_protocol_version);
                AppendU8(payload, value.device_role);
                AppendU8(payload, value.result_code);
                AppendU16Le(payload, value.max_payload);
                AppendU16Le(payload, value.max_control_rate_hz);
                AppendU32Le(payload, value.capability_bits);
                AppendU16Le(payload, value.firmware_version_major);
                AppendU16Le(payload, value.firmware_version_minor);
                AppendU16Le(payload, value.firmware_version_patch);
                AppendReservedU16(payload);
            } else if constexpr (std::is_same_v<T, Heartbeat>) {
                // Empty payload by contract.
            } else if constexpr (std::is_same_v<T, ControlUpdate>) {
                if ((value.control_flags & 0xFEU) != 0U) {
                    SetPayloadError(error, "CONTROL_UPDATE reserved control flags are non-zero");
                    return false;
                }
                AppendU32Le(payload, value.source_capture_session_id);
                AppendU32Le(payload, value.source_frame_id);
                AppendU32Le(payload, value.source_v4l2_sequence);
                AppendU8(payload, value.target_state);
                AppendU8(payload, value.control_flags);
                AppendI16Le(payload, value.dx_px);
                AppendI16Le(payload, value.dy_px);
                AppendI16Le(payload, value.error_x_q15);
                AppendI16Le(payload, value.error_y_q15);
                AppendU16Le(payload, value.confidence_u16);
                AppendU16Le(payload, value.capture_age_at_tx_ms);
            } else if constexpr (std::is_same_v<T, Status>) {
                AppendU8(payload, value.mcu_state);
                AppendU8(payload, value.link_state);
                AppendU8(payload, value.remote_stop_latched);
                AppendU8(payload, value.control_valid);
                AppendU32Le(payload, value.last_rx_wire_sequence);
                AppendU32Le(payload, value.last_control_wire_sequence);
                AppendU32Le(payload, value.rx_valid_frame_count);
                AppendU32Le(payload, value.rx_crc_error_count);
                AppendU32Le(payload, value.rx_length_error_count);
                AppendU32Le(payload, value.rx_version_error_count);
                AppendU32Le(payload, value.rx_unknown_type_count);
                AppendU32Le(payload, value.rx_sequence_gap_count);
                AppendU32Le(payload, value.rx_overflow_count);
                AppendU32Le(payload, value.control_mailbox_overwrite_count);
                AppendU32Le(payload, value.mcu_tick_ms);
                AppendI16Le(payload, value.pan_stub_q15);
                AppendI16Le(payload, value.tilt_stub_q15);
            } else if constexpr (std::is_same_v<T, RemoteStopRequest>) {
                AppendU32Le(payload, value.transaction_id);
                AppendU16Le(payload, value.reason_code);
                AppendReservedU16(payload);
            } else if constexpr (std::is_same_v<T, ClearRemoteStop>) {
                AppendU32Le(payload, value.transaction_id);
            } else if constexpr (std::is_same_v<T, Ack> ||
                                 std::is_same_v<T, Nack>) {
                AppendU8(payload, value.acked_message_type);
                AppendReservedU8(payload);
                AppendU16Le(payload, value.result_code);
                AppendU32Le(payload, value.transaction_id);
                AppendU16Le(payload, value.detail_code);
                AppendReservedU16(payload);
            } else if constexpr (std::is_same_v<T, Ping>) {
                AppendU32Le(payload, value.ping_id);
            } else if constexpr (std::is_same_v<T, Pong>) {
                AppendU32Le(payload, value.ping_id);
                AppendU32Le(payload, value.ping_wire_sequence);
            }
            return true;
        },
        body);

    if (!ok || payload->size() > kMaxPayloadSize) {
        if (payload->size() > kMaxPayloadSize) {
            SetPayloadError(error, "encoded payload exceeds protocol maximum");
        }
        payload->clear();
        return false;
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool DecodePayload(MessageType type,
                   const uint8_t* payload,
                   std::size_t payload_size,
                   MessageBody* body,
                   std::string* error) {
    if (body == nullptr || (payload == nullptr && payload_size != 0U)) {
        SetPayloadError(error, "invalid payload decode arguments");
        return false;
    }
    std::size_t offset = 0U;

    switch (type) {
        case MessageType::HELLO: {
            if (!RequirePayloadSize(payload_size, kHelloPayloadSize, error)) return false;
            Hello value;
            if (!ReadU8(payload, payload_size, &offset, &value.device_role) ||
                !ReadU8(payload, payload_size, &offset, &value.minimum_protocol_version) ||
                !ReadU8(payload, payload_size, &offset, &value.maximum_protocol_version) ||
                !ReadReservedU8(payload, payload_size, &offset) ||
                !ReadU16Le(payload, payload_size, &offset, &value.max_payload) ||
                !ReadU16Le(payload, payload_size, &offset, &value.max_control_rate_hz) ||
                !ReadU32Le(payload, payload_size, &offset, &value.capability_bits) ||
                !ReadU16Le(payload, payload_size, &offset, &value.software_version_major) ||
                !ReadU16Le(payload, payload_size, &offset, &value.software_version_minor) ||
                !ReadU16Le(payload, payload_size, &offset, &value.software_version_patch) ||
                !ReadReservedU16(payload, payload_size, &offset)) {
                SetPayloadError(error, "HELLO payload decode failed");
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::HELLO_ACK: {
            if (!RequirePayloadSize(payload_size, kHelloAckPayloadSize, error)) return false;
            HelloAck value;
            if (!ReadU32Le(payload, payload_size, &offset, &value.hello_wire_sequence) ||
                !ReadU8(payload, payload_size, &offset, &value.accepted) ||
                !ReadU8(payload, payload_size, &offset, &value.selected_protocol_version) ||
                !ReadU8(payload, payload_size, &offset, &value.device_role) ||
                !ReadU8(payload, payload_size, &offset, &value.result_code) ||
                !ReadU16Le(payload, payload_size, &offset, &value.max_payload) ||
                !ReadU16Le(payload, payload_size, &offset, &value.max_control_rate_hz) ||
                !ReadU32Le(payload, payload_size, &offset, &value.capability_bits) ||
                !ReadU16Le(payload, payload_size, &offset, &value.firmware_version_major) ||
                !ReadU16Le(payload, payload_size, &offset, &value.firmware_version_minor) ||
                !ReadU16Le(payload, payload_size, &offset, &value.firmware_version_patch) ||
                !ReadReservedU16(payload, payload_size, &offset)) {
                SetPayloadError(error, "HELLO_ACK payload decode failed");
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::HEARTBEAT:
            if (!RequirePayloadSize(payload_size, kHeartbeatPayloadSize, error)) return false;
            *body = Heartbeat{};
            break;
        case MessageType::CONTROL_UPDATE: {
            if (!RequirePayloadSize(payload_size, kControlUpdatePayloadSize, error)) return false;
            ControlUpdate value;
            if (!ReadU32Le(payload, payload_size, &offset, &value.source_capture_session_id) ||
                !ReadU32Le(payload, payload_size, &offset, &value.source_frame_id) ||
                !ReadU32Le(payload, payload_size, &offset, &value.source_v4l2_sequence) ||
                !ReadU8(payload, payload_size, &offset, &value.target_state) ||
                !ReadU8(payload, payload_size, &offset, &value.control_flags) ||
                !ReadI16Le(payload, payload_size, &offset, &value.dx_px) ||
                !ReadI16Le(payload, payload_size, &offset, &value.dy_px) ||
                !ReadI16Le(payload, payload_size, &offset, &value.error_x_q15) ||
                !ReadI16Le(payload, payload_size, &offset, &value.error_y_q15) ||
                !ReadU16Le(payload, payload_size, &offset, &value.confidence_u16) ||
                !ReadU16Le(payload, payload_size, &offset, &value.capture_age_at_tx_ms) ||
                (value.control_flags & 0xFEU) != 0U) {
                SetPayloadError(error, "CONTROL_UPDATE payload decode failed");
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::STATUS: {
            if (!RequirePayloadSize(payload_size, kStatusPayloadSize, error)) return false;
            Status value;
            if (!ReadU8(payload, payload_size, &offset, &value.mcu_state) ||
                !ReadU8(payload, payload_size, &offset, &value.link_state) ||
                !ReadU8(payload, payload_size, &offset, &value.remote_stop_latched) ||
                !ReadU8(payload, payload_size, &offset, &value.control_valid) ||
                !ReadU32Le(payload, payload_size, &offset, &value.last_rx_wire_sequence) ||
                !ReadU32Le(payload, payload_size, &offset, &value.last_control_wire_sequence) ||
                !ReadU32Le(payload, payload_size, &offset, &value.rx_valid_frame_count) ||
                !ReadU32Le(payload, payload_size, &offset, &value.rx_crc_error_count) ||
                !ReadU32Le(payload, payload_size, &offset, &value.rx_length_error_count) ||
                !ReadU32Le(payload, payload_size, &offset, &value.rx_version_error_count) ||
                !ReadU32Le(payload, payload_size, &offset, &value.rx_unknown_type_count) ||
                !ReadU32Le(payload, payload_size, &offset, &value.rx_sequence_gap_count) ||
                !ReadU32Le(payload, payload_size, &offset, &value.rx_overflow_count) ||
                !ReadU32Le(payload, payload_size, &offset, &value.control_mailbox_overwrite_count) ||
                !ReadU32Le(payload, payload_size, &offset, &value.mcu_tick_ms) ||
                !ReadI16Le(payload, payload_size, &offset, &value.pan_stub_q15) ||
                !ReadI16Le(payload, payload_size, &offset, &value.tilt_stub_q15)) {
                SetPayloadError(error, "STATUS payload decode failed");
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::REMOTE_STOP_REQUEST: {
            if (!RequirePayloadSize(payload_size, kRemoteStopRequestPayloadSize, error)) return false;
            RemoteStopRequest value;
            if (!ReadU32Le(payload, payload_size, &offset, &value.transaction_id) ||
                !ReadU16Le(payload, payload_size, &offset, &value.reason_code) ||
                !ReadReservedU16(payload, payload_size, &offset)) {
                SetPayloadError(error, "REMOTE_STOP_REQUEST payload decode failed");
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::CLEAR_REMOTE_STOP: {
            if (!RequirePayloadSize(payload_size, kClearRemoteStopPayloadSize, error)) return false;
            ClearRemoteStop value;
            if (!ReadU32Le(payload, payload_size, &offset, &value.transaction_id)) {
                SetPayloadError(error, "CLEAR_REMOTE_STOP payload decode failed");
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::ACK: {
            Ack value;
            if (!DecodeAckLike(payload, payload_size,
                               &value.acked_message_type, &value.result_code,
                               &value.transaction_id, &value.detail_code, error)) {
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::NACK: {
            Nack value;
            if (!DecodeAckLike(payload, payload_size,
                               &value.acked_message_type, &value.result_code,
                               &value.transaction_id, &value.detail_code, error)) {
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::PING: {
            if (!RequirePayloadSize(payload_size, kPingPayloadSize, error)) return false;
            Ping value;
            if (!ReadU32Le(payload, payload_size, &offset, &value.ping_id)) {
                SetPayloadError(error, "PING payload decode failed");
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::PONG: {
            if (!RequirePayloadSize(payload_size, kPongPayloadSize, error)) return false;
            Pong value;
            if (!ReadU32Le(payload, payload_size, &offset, &value.ping_id) ||
                !ReadU32Le(payload, payload_size, &offset, &value.ping_wire_sequence)) {
                SetPayloadError(error, "PONG payload decode failed");
                return false;
            }
            *body = value;
            break;
        }
    }

    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool EncodeMessage(const Message& message,
                   std::vector<uint8_t>* encoded,
                   std::string* error) {
    const MessageType actual_type = MessageTypeOf(message.body);
    if (message.header.message_type != actual_type) {
        SetPayloadError(error, "header message type does not match body variant");
        return false;
    }
    std::vector<uint8_t> payload;
    if (!EncodePayload(message.body, &payload, error)) {
        return false;
    }
    return EncodeFrame(message.header,
                       payload.empty() ? nullptr : payload.data(),
                       payload.size(), encoded, error);
}

bool DecodeMessage(const Frame& frame,
                   Message* message,
                   std::string* error) {
    if (message == nullptr || frame.payload_size != frame.header.payload_length) {
        SetPayloadError(error, "frame/message decode arguments are inconsistent");
        return false;
    }
    MessageBody body;
    if (!DecodePayload(frame.header.message_type,
                       frame.payload.data(), frame.payload_size,
                       &body, error)) {
        return false;
    }
    message->header = frame.header;
    message->body = std::move(body);
    return true;
}

}  // namespace visionarm::uart


#include "control/uart_protocol.h"

#include <utility>

namespace visionarm::uart {

bool SequenceIsNewer(uint32_t candidate, uint32_t reference) noexcept {
    const uint32_t delta = candidate - reference;
    return delta != 0U && delta < 0x80000000U;
}

SequenceRelation SequenceTracker::Observe(uint32_t sender_boot_id,
                                          uint32_t wire_sequence) noexcept {
    if (!has_baseline_ || sender_boot_id != boot_id_) {
        has_baseline_ = true;
        boot_id_ = sender_boot_id;
        last_sequence_ = wire_sequence;
        return SequenceRelation::FIRST;
    }

    if (wire_sequence == last_sequence_) {
        return SequenceRelation::DUPLICATE;
    }
    if (!SequenceIsNewer(wire_sequence, last_sequence_)) {
        return SequenceRelation::OLDER;
    }

    const uint32_t delta = wire_sequence - last_sequence_;
    last_sequence_ = wire_sequence;
    if (delta == 1U) {
        return SequenceRelation::NEWER_CONTIGUOUS;
    }
    gap_count_ += static_cast<uint64_t>(delta - 1U);
    return SequenceRelation::NEWER_GAP;
}

void SequenceTracker::Reset() noexcept {
    has_baseline_ = false;
    boot_id_ = 0U;
    last_sequence_ = 0U;
    gap_count_ = 0U;
}

}  // namespace visionarm::uart

#include <utility>

namespace visionarm::uart {

bool ReliableTransaction::Begin(MessageType message_type,
                                uint32_t transaction_id,
                                uint32_t wire_sequence,
                                std::vector<uint8_t> encoded_frame,
                                uint64_t now_ms,
                                uint64_t timeout_ms,
                                uint32_t maximum_retries) noexcept {
    if (outstanding_ || encoded_frame.empty() || timeout_ms == 0U) {
        return false;
    }
    outstanding_ = true;
    message_type_ = message_type;
    transaction_id_ = transaction_id;
    wire_sequence_ = wire_sequence;
    encoded_frame_ = std::move(encoded_frame);
    timeout_ms_ = timeout_ms;
    maximum_retries_ = maximum_retries;
    retries_performed_ = 0U;
    deadline_ms_ = now_ms + timeout_ms_;
    return true;
}

ReliablePollResult ReliableTransaction::Poll(uint64_t now_ms) noexcept {
    if (!outstanding_ || now_ms < deadline_ms_) {
        return {};
    }
    if (retries_performed_ >= maximum_retries_) {
        Cancel();
        return {ReliablePollAction::FAILED, nullptr};
    }
    ++retries_performed_;
    deadline_ms_ = now_ms + timeout_ms_;
    return {ReliablePollAction::RETRY, &encoded_frame_};
}

ReliableResponse ReliableTransaction::HandleResponse(
    MessageType acked_message_type,
    uint32_t transaction_id,
    bool acknowledged,
    uint16_t result_code,
    uint16_t detail_code) noexcept {
    if (!outstanding_ || acked_message_type != message_type_ ||
        transaction_id != transaction_id_) {
        return {};
    }
    ReliableResponse response;
    response.matched = true;
    response.acknowledged = acknowledged;
    response.result_code = result_code;
    response.detail_code = detail_code;
    Cancel();
    return response;
}

void ReliableTransaction::Cancel() noexcept {
    outstanding_ = false;
    message_type_ = MessageType::REMOTE_STOP_REQUEST;
    transaction_id_ = 0U;
    wire_sequence_ = 0U;
    encoded_frame_.clear();
    deadline_ms_ = 0U;
    timeout_ms_ = 0U;
    maximum_retries_ = 0U;
    retries_performed_ = 0U;
}


}  // namespace visionarm::uart
