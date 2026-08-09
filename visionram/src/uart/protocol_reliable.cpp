#include "uart/protocol.h"

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
