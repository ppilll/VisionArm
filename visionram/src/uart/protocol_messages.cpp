#include "uart/protocol.h"

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

void SetError(std::string* error, const char* message) {
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
        SetError(error, "payload size does not match message contract");
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
        SetError(error, "ACK/NACK payload decode or reserved-field validation failed");
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
        SetError(error, "payload output is null");
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
                    SetError(error, "CONTROL_UPDATE reserved control flags are non-zero");
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
            SetError(error, "encoded payload exceeds protocol maximum");
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
        SetError(error, "invalid payload decode arguments");
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
                SetError(error, "HELLO payload decode failed");
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
                SetError(error, "HELLO_ACK payload decode failed");
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
                SetError(error, "CONTROL_UPDATE payload decode failed");
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
                SetError(error, "STATUS payload decode failed");
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
                SetError(error, "REMOTE_STOP_REQUEST payload decode failed");
                return false;
            }
            *body = value;
            break;
        }
        case MessageType::CLEAR_REMOTE_STOP: {
            if (!RequirePayloadSize(payload_size, kClearRemoteStopPayloadSize, error)) return false;
            ClearRemoteStop value;
            if (!ReadU32Le(payload, payload_size, &offset, &value.transaction_id)) {
                SetError(error, "CLEAR_REMOTE_STOP payload decode failed");
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
                SetError(error, "PING payload decode failed");
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
                SetError(error, "PONG payload decode failed");
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
        SetError(error, "header message type does not match body variant");
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
        SetError(error, "frame/message decode arguments are inconsistent");
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
