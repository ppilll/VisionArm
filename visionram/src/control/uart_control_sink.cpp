#include "control/uart_control_sink.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace visionarm {
namespace {

constexpr uint8_t kProtocolNoTarget = 0U;
constexpr uint8_t kProtocolCandidate = 1U;
constexpr uint8_t kProtocolDetected = 2U;
constexpr uint8_t kProtocolLost = 3U;
constexpr uint8_t kProtocolInvalid = 4U;
constexpr uint8_t kProtocolStale = 5U;

struct MappingResult {
    uart::ControlUpdateInput input;
    bool state_known = false;
    bool nonfinite_invalidation = false;
    bool invalid_timestamp = false;
    uint64_t identity_truncations = 0U;
};

uint8_t MapTargetState(TargetState state, bool* known) noexcept {
    if (known == nullptr) {
        return kProtocolInvalid;
    }

    *known = true;
    switch (state) {
        case TargetState::NO_TARGET:
            return kProtocolNoTarget;
        case TargetState::CANDIDATE:
            return kProtocolCandidate;
        case TargetState::DETECTED:
            return kProtocolDetected;
        case TargetState::LOST:
            return kProtocolLost;
        case TargetState::INVALID:
            return kProtocolInvalid;
        case TargetState::STALE:
            return kProtocolStale;
    }

    *known = false;
    return kProtocolInvalid;
}

int16_t RoundSaturateInt16(float value) noexcept {
    const double clamped = std::clamp(
        static_cast<double>(value),
        static_cast<double>(std::numeric_limits<int16_t>::min()),
        static_cast<double>(std::numeric_limits<int16_t>::max()));
    return static_cast<int16_t>(std::lround(clamped));
}

int16_t NormalizedToQ15(float value) noexcept {
    const double clamped = std::clamp(static_cast<double>(value), -1.0, 1.0);
    return static_cast<int16_t>(std::lround(clamped * 32767.0));
}

uint16_t ConfidenceToU16(float value) noexcept {
    const double clamped = std::clamp(static_cast<double>(value), 0.0, 1.0);
    return static_cast<uint16_t>(std::lround(clamped * 65535.0));
}

bool ControlNumbersFinite(const ControlResult& result) noexcept {
    return std::isfinite(result.error.dx_px) &&
           std::isfinite(result.error.dy_px) &&
           std::isfinite(result.error.error_x_normalized) &&
           std::isfinite(result.error.error_y_normalized) &&
           std::isfinite(result.error.confidence);
}

MappingResult MapControlResult(const ControlResult& result) noexcept {
    MappingResult mapped;

    mapped.input.source_capture_session_id =
        static_cast<uint32_t>(result.identity.capture_session_id);
    mapped.input.source_frame_id = static_cast<uint32_t>(result.identity.frame_id);
    mapped.input.source_v4l2_sequence = result.identity.v4l2_sequence;

    if (result.identity.capture_session_id >
        static_cast<uint64_t>(std::numeric_limits<uint32_t>::max())) {
        ++mapped.identity_truncations;
    }
    if (result.identity.frame_id >
        static_cast<uint64_t>(std::numeric_limits<uint32_t>::max())) {
        ++mapped.identity_truncations;
    }

    mapped.input.target_state = MapTargetState(result.state, &mapped.state_known);

    if (result.capture_timestamp_ns > 0) {
        mapped.input.capture_monotonic_ns =
            static_cast<uint64_t>(result.capture_timestamp_ns);
    } else {
        mapped.invalid_timestamp = true;
        mapped.input.capture_monotonic_ns = 0U;
    }

    const bool requested_valid =
        mapped.state_known &&
        result.state == TargetState::DETECTED &&
        result.valid &&
        result.error.valid;

    const bool finite = !requested_valid || ControlNumbersFinite(result);
    if (requested_valid && !finite) {
        mapped.nonfinite_invalidation = true;
    }

    mapped.input.valid = requested_valid && finite && !mapped.invalid_timestamp;

    if (mapped.input.valid) {
        mapped.input.dx_px = RoundSaturateInt16(result.error.dx_px);
        mapped.input.dy_px = RoundSaturateInt16(result.error.dy_px);
        mapped.input.error_x_q15 =
            NormalizedToQ15(result.error.error_x_normalized);
        mapped.input.error_y_q15 =
            NormalizedToQ15(result.error.error_y_normalized);
        mapped.input.confidence_u16 = ConfidenceToU16(result.error.confidence);
    }

    return mapped;
}

}  // namespace

UartControlSink::UartControlSink(uart::UartLink& link) noexcept
    : link_(&link) {}

bool UartControlSink::Submit(const ControlResult& result) noexcept {
    try {
        const MappingResult mapped = MapControlResult(result);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++snapshot_.metrics.submissions;

            const std::size_t state_index =
                static_cast<std::size_t>(result.state);
            if (mapped.state_known &&
                state_index < snapshot_.metrics.state_counts.size()) {
                ++snapshot_.metrics.state_counts[state_index];
            } else {
                ++snapshot_.metrics.invalid_state_invalidations;
            }

            snapshot_.metrics.identity_truncations +=
                mapped.identity_truncations;
            if (mapped.nonfinite_invalidation) {
                ++snapshot_.metrics.nonfinite_invalidations;
            }
            if (mapped.invalid_timestamp) {
                ++snapshot_.metrics.invalid_timestamp_invalidations;
            }
            if (mapped.input.valid) {
                ++snapshot_.metrics.valid_inputs;
            } else {
                ++snapshot_.metrics.invalid_inputs;
            }
            snapshot_.latest_input = mapped.input;
        }

        const bool accepted =
            link_ != nullptr && link_->SubmitLatestControl(mapped.input);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (accepted) {
                ++snapshot_.metrics.accepted;
            } else {
                ++snapshot_.metrics.rejected;
            }
        }
        return accepted;
    } catch (...) {
        try {
            std::lock_guard<std::mutex> lock(mutex_);
            ++snapshot_.metrics.rejected;
        } catch (...) {
        }
        return false;
    }
}

UartControlSinkSnapshot UartControlSink::Snapshot() const noexcept {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    } catch (...) {
        return {};
    }
}

}  // namespace visionarm
