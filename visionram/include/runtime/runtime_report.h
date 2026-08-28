#pragma once

#include "common/pipeline_types.h"

#include <cstdint>
#include <iosfwd>

namespace visionarm::runtime {

void WriteLatency(std::ostream& stream,
                  const char* name,
                  const LatencyDistributionSnapshot& value);
void WriteQueue(std::ostream& stream,
                const char* name,
                const QueueStatsSnapshot& value);
[[nodiscard]] bool QueueBounded(const QueueStatsSnapshot& value) noexcept;
[[nodiscard]] bool QueueDrained(const QueueStatsSnapshot& value) noexcept;
[[nodiscard]] bool LatencyComplete(
    const LatencyDistributionSnapshot& value,
    std::uint64_t expected_samples) noexcept;

}  // namespace visionarm::runtime
