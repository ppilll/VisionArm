#pragma once

#include "common/pipeline_types.h"

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>

namespace visionarm {

struct ControlResult {
    FrameIdentity identity;
    TargetState state = TargetState::NO_TARGET;
    bool valid = false;

    TargetObservation observation;
    TargetError error;

    int64_t capture_timestamp_ns = 0;
    int64_t generated_timestamp_ns = 0;
    int64_t age_ns = 0;

    uint32_t consecutive_hits = 0U;
    uint32_t consecutive_misses = 0U;
};

class IControlSink {
public:
    virtual ~IControlSink() = default;
    virtual bool Submit(const ControlResult& result) noexcept = 0;
};

struct MockControlSinkSnapshot {
    uint64_t submissions = 0U;
    uint64_t valid_controls = 0U;
    uint64_t invalid_controls = 0U;
    uint64_t submission_failures = 0U;
    std::array<uint64_t, 6U> state_counts{};
    std::optional<ControlResult> latest;
};

// V4 sink: stores only the latest control result and counters. It performs no
// UART, file or terminal I/O in the real-time path.
class MockControlSink final : public IControlSink {
public:
    bool Submit(const ControlResult& result) noexcept override;
    [[nodiscard]] MockControlSinkSnapshot Snapshot() const noexcept;

private:
    mutable std::mutex mutex_;
    MockControlSinkSnapshot snapshot_;
};

}  // namespace visionarm
