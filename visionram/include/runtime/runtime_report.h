#pragma once

#include "common/pipeline_types.h"

#include <cstdint>
#include <iosfwd>
#include <string>
#include <string_view>

namespace visionarm::runtime {

struct RuntimeOptions;

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

[[nodiscard]] int PublishRuntimeReport(
    const RuntimeOptions& options,
    bool passed,
    double observed_duration_seconds,
    const PipelineStatsSnapshot& stats,
    std::string diagnostic_report);
[[nodiscard]] int WriteFaultRuntimeReport(
    const RuntimeOptions* options,
    int argc,
    char** argv,
    std::string_view error_message);

}  // namespace visionarm::runtime
