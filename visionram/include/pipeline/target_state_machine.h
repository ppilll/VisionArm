#pragma once

#include "control/control_sink.h"
#include "pipeline/latest_result_store.h"

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>

namespace visionarm {

struct TargetStateMachineConfig {
    uint32_t acquire_hits = 2U;
    uint32_t lost_misses = 3U;
    int64_t max_result_age_ns = 150'000'000LL;
};

struct TargetStateMachineSnapshot {
    uint64_t processed_packets = 0U;
    uint64_t valid_controls = 0U;
    uint64_t control_sink_failures = 0U;
    uint64_t perception_sink_failures = 0U;
    uint64_t invalid_timestamp_packets = 0U;
    std::array<uint64_t, 6U> state_counts{};
    uint32_t consecutive_hits = 0U;
    uint32_t consecutive_misses = 0U;
    bool acquired = false;
    bool lost_latched = false;
    TargetState latest_state = TargetState::NO_TARGET;
    int64_t latest_age_ns = 0;
    int64_t maximum_age_ns = 0;
};

// Stateful adapter between raw postprocess results and control/business sinks.
// It never reuses an old TargetError. Any state other than DETECTED produces
// valid=false and zero control error components.
class TargetStateMachine final : public IPerceptionSink {
public:
    using ClockFunction = std::function<int64_t()>;

    TargetStateMachine(
        TargetStateMachineConfig config,
        IControlSink* control_sink,
        IPerceptionSink* finalized_perception_sink = nullptr,
        ClockFunction clock = {});

    void Publish(PerceptionPacket packet) override;

    [[nodiscard]] TargetStateMachineSnapshot Snapshot() const noexcept;
    void Reset() noexcept;

private:
    [[nodiscard]] TargetState ClassifyAndUpdate(
        bool raw_detected,
        bool raw_invalid,
        bool stale) noexcept;

    static TargetObservation MakeObservation(
        const PerceptionPacket& packet,
        TargetState state) noexcept;
    static TargetError MakeControlError(
        const PerceptionPacket& packet,
        TargetState state) noexcept;

    TargetStateMachineConfig config_;
    IControlSink* control_sink_ = nullptr;
    IPerceptionSink* finalized_perception_sink_ = nullptr;
    ClockFunction clock_;

    mutable std::mutex mutex_;
    TargetStateMachineSnapshot snapshot_;
};

}  // namespace visionarm
