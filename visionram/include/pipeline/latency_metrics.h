#pragma once

#include "common/pipeline_types.h"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace visionarm {

// Bounded latency collector. Mean/max cover the complete run; percentiles are
// calculated from the retained fixed-capacity window. Configure capacity large
// enough for the intended stability test to avoid truncation.
class LatencyAccumulator final {
public:
    explicit LatencyAccumulator(std::size_t capacity = 65536U);

    LatencyAccumulator(const LatencyAccumulator&) = delete;
    LatencyAccumulator& operator=(const LatencyAccumulator&) = delete;

    void Add(int64_t nanoseconds) noexcept;
    [[nodiscard]] LatencyDistributionSnapshot Snapshot() const;
    void Reset() noexcept;

private:
    mutable std::mutex mutex_;
    std::vector<int64_t> samples_;
    std::size_t next_index_ = 0U;
    uint64_t total_samples_ = 0U;
    long double total_nanoseconds_ = 0.0L;
    int64_t maximum_nanoseconds_ = 0;
};

}  // namespace visionarm
