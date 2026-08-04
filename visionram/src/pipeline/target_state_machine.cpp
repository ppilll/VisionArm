#include "pipeline/target_state_machine.h"

#include "common/monotonic_clock.h"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace visionarm {
namespace {

[[nodiscard]] bool RawDetected(const PerceptionPacket& packet) noexcept {
    return packet.result.target.has_value() &&
        packet.result.target->state == TargetState::DETECTED &&
        packet.result.target->valid && packet.result.error.valid;
}

[[nodiscard]] bool RawInvalid(const PerceptionPacket& packet) noexcept {
    return (packet.result.target.has_value() &&
            packet.result.target->state == TargetState::INVALID) ||
           packet.result.error.state == TargetState::INVALID;
}

}  // namespace

TargetStateMachine::TargetStateMachine(
    TargetStateMachineConfig config,
    IControlSink* control_sink,
    IPerceptionSink* finalized_perception_sink,
    ClockFunction clock)
    : config_(config),
      control_sink_(control_sink),
      finalized_perception_sink_(finalized_perception_sink),
      clock_(std::move(clock)) {
    if (config_.acquire_hits == 0U || config_.lost_misses == 0U ||
        config_.max_result_age_ns <= 0 || control_sink_ == nullptr) {
        throw std::invalid_argument("invalid target state-machine config");
    }
    if (!clock_) {
        clock_ = [] { return MonotonicNowNs(); };
    }
}

TargetState TargetStateMachine::ClassifyAndUpdate(
    bool raw_detected,
    bool raw_invalid,
    bool stale) noexcept {
    if (stale) {
        snapshot_.consecutive_hits = 0U;
        ++snapshot_.consecutive_misses;
        if (snapshot_.acquired &&
            snapshot_.consecutive_misses >= config_.lost_misses) {
            snapshot_.acquired = false;
            snapshot_.lost_latched = true;
        }
        return TargetState::STALE;
    }

    if (raw_invalid) {
        snapshot_.consecutive_hits = 0U;
        ++snapshot_.consecutive_misses;
        if (snapshot_.acquired &&
            snapshot_.consecutive_misses >= config_.lost_misses) {
            snapshot_.acquired = false;
            snapshot_.lost_latched = true;
        }
        return TargetState::INVALID;
    }

    if (raw_detected) {
        snapshot_.consecutive_misses = 0U;
        ++snapshot_.consecutive_hits;
        snapshot_.lost_latched = false;
        if (snapshot_.acquired ||
            snapshot_.consecutive_hits >= config_.acquire_hits) {
            snapshot_.acquired = true;
            return TargetState::DETECTED;
        }
        return TargetState::CANDIDATE;
    }

    snapshot_.consecutive_hits = 0U;
    ++snapshot_.consecutive_misses;
    if (snapshot_.acquired &&
        snapshot_.consecutive_misses >= config_.lost_misses) {
        snapshot_.acquired = false;
        snapshot_.lost_latched = true;
    }
    return snapshot_.lost_latched
        ? TargetState::LOST
        : TargetState::NO_TARGET;
}

TargetObservation TargetStateMachine::MakeObservation(
    const PerceptionPacket& packet,
    TargetState state) noexcept {
    TargetObservation observation;
    if (packet.result.target.has_value()) {
        observation = *packet.result.target;
    }
    observation.state = state;
    observation.valid = state == TargetState::DETECTED;
    return observation;
}

TargetError TargetStateMachine::MakeControlError(
    const PerceptionPacket& packet,
    TargetState state) noexcept {
    TargetError error = packet.result.error;
    error.state = state;
    error.valid = state == TargetState::DETECTED && packet.result.error.valid;
    if (!error.valid) {
        error.dx_px = 0.0F;
        error.dy_px = 0.0F;
        error.error_x_normalized = 0.0F;
        error.error_y_normalized = 0.0F;
    }
    return error;
}

void TargetStateMachine::Publish(PerceptionPacket packet) {
    const int64_t generated_ns = clock_();
    const bool timestamp_valid =
        packet.identity.capture_timestamp_ns > 0 &&
        generated_ns >= packet.identity.capture_timestamp_ns;
    const int64_t age_ns = timestamp_valid
        ? generated_ns - packet.identity.capture_timestamp_ns
        : 0;
    const bool stale = timestamp_valid &&
        age_ns > config_.max_result_age_ns;

    ControlResult control;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.processed_packets;
        if (!timestamp_valid) {
            ++snapshot_.invalid_timestamp_packets;
        }

        const TargetState state = ClassifyAndUpdate(
            RawDetected(packet),
            RawInvalid(packet) || !timestamp_valid,
            stale);

        snapshot_.latest_state = state;
        snapshot_.latest_age_ns = age_ns;
        snapshot_.maximum_age_ns = std::max(
            snapshot_.maximum_age_ns, age_ns);
        const std::size_t state_index = static_cast<std::size_t>(state);
        if (state_index < snapshot_.state_counts.size()) {
            ++snapshot_.state_counts[state_index];
        }

        packet.generated_timestamp_ns = generated_ns;
        packet.result_age_ns = age_ns;
        packet.result.target = MakeObservation(packet, state);
        packet.result.error = MakeControlError(packet, state);

        control.identity = packet.identity;
        control.state = state;
        control.valid = packet.result.error.valid;
        control.observation = *packet.result.target;
        control.error = packet.result.error;
        control.capture_timestamp_ns = packet.identity.capture_timestamp_ns;
        control.generated_timestamp_ns = generated_ns;
        control.age_ns = age_ns;
        control.consecutive_hits = snapshot_.consecutive_hits;
        control.consecutive_misses = snapshot_.consecutive_misses;
        if (control.valid) {
            ++snapshot_.valid_controls;
        }
    }

    if (!control_sink_->Submit(control)) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.control_sink_failures;
    }

    if (finalized_perception_sink_ != nullptr) {
        try {
            finalized_perception_sink_->Publish(std::move(packet));
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            ++snapshot_.perception_sink_failures;
        }
    }
}

TargetStateMachineSnapshot TargetStateMachine::Snapshot() const noexcept {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    } catch (...) {
        return {};
    }
}

void TargetStateMachine::Reset() noexcept {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = {};
    } catch (...) {
    }
}

}  // namespace visionarm
