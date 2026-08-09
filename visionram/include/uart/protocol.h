#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace visionarm::uart {

inline constexpr uint8_t kFrameFlag = 0x7EU;
inline constexpr uint8_t kFrameEscape = 0x7DU;
inline constexpr uint8_t kEscapeXor = 0x20U;
inline constexpr uint8_t kProtocolVersion = 1U;
inline constexpr std::size_t kHeaderSize = 16U;
inline constexpr std::size_t kCrcSize = 2U;
inline constexpr std::size_t kMaxPayloadSize = 128U;
inline constexpr std::size_t kMaxRawFrameSize =
    kHeaderSize + kMaxPayloadSize + kCrcSize;
inline constexpr std::size_t kMaxEncodedFrameSize =
    2U + (2U * kMaxRawFrameSize);

static_assert(kMaxRawFrameSize == 146U, "V1 raw-frame contract changed");
static_assert(kMaxEncodedFrameSize == 294U,
              "V1 encoded-frame contract changed");

enum class MessageType : uint8_t {
    HELLO = 0x01U,
    HELLO_ACK = 0x02U,
    HEARTBEAT = 0x03U,
    CONTROL_UPDATE = 0x04U,
    STATUS = 0x05U,
    REMOTE_STOP_REQUEST = 0x06U,
    CLEAR_REMOTE_STOP = 0x07U,
    ACK = 0x08U,
    NACK = 0x09U,
    PING = 0x0AU,
    PONG = 0x0BU,
};

[[nodiscard]] constexpr bool IsKnownMessageType(uint8_t value) noexcept {
    return value >= static_cast<uint8_t>(MessageType::HELLO) &&
           value <= static_cast<uint8_t>(MessageType::PONG);
}

struct Header {
    uint8_t protocol_version = kProtocolVersion;
    MessageType message_type = MessageType::HEARTBEAT;
    uint16_t payload_length = 0U;
    uint32_t wire_sequence = 0U;
    uint32_t sender_boot_id = 0U;
    uint32_t sender_uptime_ms = 0U;
};

struct Frame {
    Header header;
    std::array<uint8_t, kMaxPayloadSize> payload{};
    std::size_t payload_size = 0U;
};

enum class FrameError {
    NONE,
    TOO_SHORT,
    LENGTH,
    CRC,
    VERSION,
    UNKNOWN_TYPE,
    OVERSIZE,
    ESCAPE,
    TIMEOUT,
};

struct ParserStats {
    uint64_t input_bytes = 0U;
    uint64_t noise_bytes = 0U;
    uint64_t valid_frames = 0U;
    uint64_t empty_frames = 0U;
    uint64_t crc_errors = 0U;
    uint64_t length_errors = 0U;
    uint64_t version_errors = 0U;
    uint64_t unknown_type_errors = 0U;
    uint64_t oversize_errors = 0U;
    uint64_t escape_errors = 0U;
    uint64_t timeout_errors = 0U;
};

inline void AppendU8(std::vector<uint8_t>* output, uint8_t value) {
    output->push_back(value);
}

inline void AppendU16Le(std::vector<uint8_t>* output, uint16_t value) {
    output->push_back(static_cast<uint8_t>(value & 0xFFU));
    output->push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

inline void AppendI16Le(std::vector<uint8_t>* output, int16_t value) {
    AppendU16Le(output, static_cast<uint16_t>(value));
}

inline void AppendU32Le(std::vector<uint8_t>* output, uint32_t value) {
    output->push_back(static_cast<uint8_t>(value & 0xFFU));
    output->push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
    output->push_back(static_cast<uint8_t>((value >> 16U) & 0xFFU));
    output->push_back(static_cast<uint8_t>((value >> 24U) & 0xFFU));
}

inline bool ReadU8(const uint8_t* data,
                   std::size_t size,
                   std::size_t* offset,
                   uint8_t* value) noexcept {
    if (data == nullptr || offset == nullptr || value == nullptr ||
        *offset >= size) {
        return false;
    }
    *value = data[*offset];
    ++(*offset);
    return true;
}

inline bool ReadU16Le(const uint8_t* data,
                      std::size_t size,
                      std::size_t* offset,
                      uint16_t* value) noexcept {
    if (data == nullptr || offset == nullptr || value == nullptr ||
        *offset > size || size - *offset < 2U) {
        return false;
    }
    *value = static_cast<uint16_t>(data[*offset]) |
             static_cast<uint16_t>(
                 static_cast<uint16_t>(data[*offset + 1U]) << 8U);
    *offset += 2U;
    return true;
}

inline bool ReadI16Le(const uint8_t* data,
                      std::size_t size,
                      std::size_t* offset,
                      int16_t* value) noexcept {
    uint16_t raw = 0U;
    if (!ReadU16Le(data, size, offset, &raw)) {
        return false;
    }
    *value = static_cast<int16_t>(raw);
    return true;
}

inline bool ReadU32Le(const uint8_t* data,
                      std::size_t size,
                      std::size_t* offset,
                      uint32_t* value) noexcept {
    if (data == nullptr || offset == nullptr || value == nullptr ||
        *offset > size || size - *offset < 4U) {
        return false;
    }
    *value = static_cast<uint32_t>(data[*offset]) |
             (static_cast<uint32_t>(data[*offset + 1U]) << 8U) |
             (static_cast<uint32_t>(data[*offset + 2U]) << 16U) |
             (static_cast<uint32_t>(data[*offset + 3U]) << 24U);
    *offset += 4U;
    return true;
}

[[nodiscard]] uint16_t Crc16CcittFalse(const uint8_t* data,
                                       std::size_t size) noexcept;

[[nodiscard]] bool EncodeFrame(const Header& header,
                               const uint8_t* payload,
                               std::size_t payload_size,
                               std::vector<uint8_t>* encoded,
                               std::string* error);

[[nodiscard]] bool DecodeRawFrame(const uint8_t* raw,
                                  std::size_t raw_size,
                                  Frame* frame,
                                  FrameError* frame_error) noexcept;

class FrameParser {
public:
    explicit FrameParser(uint64_t assembly_timeout_ms) noexcept;

    void Feed(const uint8_t* data,
              std::size_t size,
              uint64_t now_ms,
              std::vector<Frame>* frames) noexcept;
    void Tick(uint64_t now_ms) noexcept;
    void Reset() noexcept;

    [[nodiscard]] const ParserStats& Stats() const noexcept { return stats_; }

private:
    void HandleFlag(std::vector<Frame>* frames) noexcept;
    void AppendRaw(uint8_t value) noexcept;
    void DiscardUntilFlag() noexcept;
    void ResetCandidate(bool collecting) noexcept;
    void AccountFrameError(FrameError error) noexcept;

    uint64_t assembly_timeout_ms_ = 0U;
    bool collecting_ = false;
    bool escape_pending_ = false;
    bool discard_until_flag_ = false;
    bool has_activity_time_ = false;
    uint64_t last_activity_ms_ = 0U;
    std::array<uint8_t, kMaxRawFrameSize> raw_{};
    std::size_t raw_size_ = 0U;
    ParserStats stats_;
};

struct Hello {
    uint8_t device_role = 0U;
    uint8_t minimum_protocol_version = kProtocolVersion;
    uint8_t maximum_protocol_version = kProtocolVersion;
    uint16_t max_payload = static_cast<uint16_t>(kMaxPayloadSize);
    uint16_t max_control_rate_hz = 0U;
    uint32_t capability_bits = 0U;
    uint16_t software_version_major = 0U;
    uint16_t software_version_minor = 0U;
    uint16_t software_version_patch = 0U;
};

struct HelloAck {
    uint32_t hello_wire_sequence = 0U;
    uint8_t accepted = 0U;
    uint8_t selected_protocol_version = kProtocolVersion;
    uint8_t device_role = 0U;
    uint8_t result_code = 0U;
    uint16_t max_payload = static_cast<uint16_t>(kMaxPayloadSize);
    uint16_t max_control_rate_hz = 0U;
    uint32_t capability_bits = 0U;
    uint16_t firmware_version_major = 0U;
    uint16_t firmware_version_minor = 0U;
    uint16_t firmware_version_patch = 0U;
};

struct Heartbeat {};

struct ControlUpdate {
    uint32_t source_capture_session_id = 0U;
    uint32_t source_frame_id = 0U;
    uint32_t source_v4l2_sequence = 0U;
    uint8_t target_state = 0U;
    uint8_t control_flags = 0U;
    int16_t dx_px = 0;
    int16_t dy_px = 0;
    int16_t error_x_q15 = 0;
    int16_t error_y_q15 = 0;
    uint16_t confidence_u16 = 0U;
    uint16_t capture_age_at_tx_ms = 0U;
};

struct Status {
    uint8_t mcu_state = 0U;
    uint8_t link_state = 0U;
    uint8_t remote_stop_latched = 0U;
    uint8_t control_valid = 0U;
    uint32_t last_rx_wire_sequence = 0U;
    uint32_t last_control_wire_sequence = 0U;
    uint32_t rx_valid_frame_count = 0U;
    uint32_t rx_crc_error_count = 0U;
    uint32_t rx_length_error_count = 0U;
    uint32_t rx_version_error_count = 0U;
    uint32_t rx_unknown_type_count = 0U;
    uint32_t rx_sequence_gap_count = 0U;
    uint32_t rx_overflow_count = 0U;
    uint32_t control_mailbox_overwrite_count = 0U;
    uint32_t mcu_tick_ms = 0U;
    int16_t pan_stub_q15 = 0;
    int16_t tilt_stub_q15 = 0;
};

struct RemoteStopRequest {
    uint32_t transaction_id = 0U;
    uint16_t reason_code = 0U;
};

struct ClearRemoteStop {
    uint32_t transaction_id = 0U;
};

struct Ack {
    uint8_t acked_message_type = 0U;
    uint16_t result_code = 0U;
    uint32_t transaction_id = 0U;
    uint16_t detail_code = 0U;
};

struct Nack {
    uint8_t acked_message_type = 0U;
    uint16_t result_code = 0U;
    uint32_t transaction_id = 0U;
    uint16_t detail_code = 0U;
};

struct Ping {
    uint32_t ping_id = 0U;
};

struct Pong {
    uint32_t ping_id = 0U;
    uint32_t ping_wire_sequence = 0U;
};

using MessageBody = std::variant<Hello,
                                 HelloAck,
                                 Heartbeat,
                                 ControlUpdate,
                                 Status,
                                 RemoteStopRequest,
                                 ClearRemoteStop,
                                 Ack,
                                 Nack,
                                 Ping,
                                 Pong>;

struct Message {
    Header header;
    MessageBody body;
};

[[nodiscard]] MessageType MessageTypeOf(const MessageBody& body) noexcept;
[[nodiscard]] bool EncodePayload(const MessageBody& body,
                                 std::vector<uint8_t>* payload,
                                 std::string* error);
[[nodiscard]] bool DecodePayload(MessageType type,
                                 const uint8_t* payload,
                                 std::size_t payload_size,
                                 MessageBody* body,
                                 std::string* error);
[[nodiscard]] bool EncodeMessage(const Message& message,
                                 std::vector<uint8_t>* encoded,
                                 std::string* error);
[[nodiscard]] bool DecodeMessage(const Frame& frame,
                                 Message* message,
                                 std::string* error);

enum class SequenceRelation {
    FIRST,
    DUPLICATE,
    NEWER_CONTIGUOUS,
    NEWER_GAP,
    OLDER,
};

[[nodiscard]] bool SequenceIsNewer(uint32_t candidate,
                                   uint32_t reference) noexcept;

class SequenceTracker {
public:
    [[nodiscard]] SequenceRelation Observe(uint32_t sender_boot_id,
                                           uint32_t wire_sequence) noexcept;
    void Reset() noexcept;

    [[nodiscard]] bool HasBaseline() const noexcept { return has_baseline_; }
    [[nodiscard]] uint32_t BootId() const noexcept { return boot_id_; }
    [[nodiscard]] uint32_t LastSequence() const noexcept { return last_sequence_; }
    [[nodiscard]] uint64_t GapCount() const noexcept { return gap_count_; }

private:
    bool has_baseline_ = false;
    uint32_t boot_id_ = 0U;
    uint32_t last_sequence_ = 0U;
    uint64_t gap_count_ = 0U;
};

enum class ReliablePollAction {
    NONE,
    RETRY,
    FAILED,
};

struct ReliablePollResult {
    ReliablePollAction action = ReliablePollAction::NONE;
    const std::vector<uint8_t>* encoded_frame = nullptr;
};

struct ReliableResponse {
    bool matched = false;
    bool acknowledged = false;
    uint16_t result_code = 0U;
    uint16_t detail_code = 0U;
};

class ReliableTransaction {
public:
    [[nodiscard]] bool Begin(MessageType message_type,
                             uint32_t transaction_id,
                             uint32_t wire_sequence,
                             std::vector<uint8_t> encoded_frame,
                             uint64_t now_ms,
                             uint64_t timeout_ms,
                             uint32_t maximum_retries) noexcept;

    [[nodiscard]] ReliablePollResult Poll(uint64_t now_ms) noexcept;

    [[nodiscard]] ReliableResponse HandleResponse(
        MessageType acked_message_type,
        uint32_t transaction_id,
        bool acknowledged,
        uint16_t result_code,
        uint16_t detail_code) noexcept;

    void Cancel() noexcept;

    [[nodiscard]] bool IsOutstanding() const noexcept { return outstanding_; }
    [[nodiscard]] uint32_t TransactionId() const noexcept { return transaction_id_; }
    [[nodiscard]] uint32_t WireSequence() const noexcept { return wire_sequence_; }
    [[nodiscard]] uint32_t RetriesPerformed() const noexcept { return retries_performed_; }

private:
    bool outstanding_ = false;
    MessageType message_type_ = MessageType::REMOTE_STOP_REQUEST;
    uint32_t transaction_id_ = 0U;
    uint32_t wire_sequence_ = 0U;
    std::vector<uint8_t> encoded_frame_;
    uint64_t deadline_ms_ = 0U;
    uint64_t timeout_ms_ = 0U;
    uint32_t maximum_retries_ = 0U;
    uint32_t retries_performed_ = 0U;
};

}  // namespace visionarm::uart
