#include "uart/protocol.h"

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
