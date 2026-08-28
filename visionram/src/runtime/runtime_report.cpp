#include "runtime/runtime_report.h"

#include <ostream>

namespace visionarm::runtime {

void WriteLatency(std::ostream& stream,
                  const char* name,
                  const LatencyDistributionSnapshot& value) {
    stream << name << ".total_samples=" << value.total_samples << '\n'
           << name << ".retained_samples=" << value.retained_samples << '\n'
           << name << ".truncated=" << (value.truncated ? 1 : 0) << '\n'
           << name << ".mean_ms=" << value.mean_ms << '\n'
           << name << ".p50_ms=" << value.p50_ms << '\n'
           << name << ".p95_ms=" << value.p95_ms << '\n'
           << name << ".p99_ms=" << value.p99_ms << '\n'
           << name << ".maximum_ms=" << value.maximum_ms << '\n';
}

void WriteQueue(std::ostream& stream,
                const char* name,
                const QueueStatsSnapshot& value) {
    stream << name << ".capacity=" << value.capacity << '\n'
           << name << ".high_watermark=" << value.high_watermark << '\n'
           << name << ".current_size=" << value.current_size << '\n'
           << name << ".pushed=" << value.pushed << '\n'
           << name << ".popped=" << value.popped << '\n'
           << name << ".replaced_oldest=" << value.replaced_oldest << '\n'
           << name << ".stopped=" << (value.stopped ? 1 : 0) << '\n';
}

bool QueueBounded(const QueueStatsSnapshot& value) noexcept {
    return value.capacity > 0U &&
        value.high_watermark <= value.capacity &&
        value.current_size <= value.capacity;
}

bool QueueDrained(const QueueStatsSnapshot& value) noexcept {
    return QueueBounded(value) && value.stopped && value.current_size == 0U &&
        value.pushed == value.popped + value.replaced_oldest &&
        value.replaced_oldest == 0U;
}

bool LatencyComplete(const LatencyDistributionSnapshot& value,
                     std::uint64_t expected_samples) noexcept {
    return expected_samples > 0U &&
        value.total_samples == expected_samples &&
        value.retained_samples == value.total_samples &&
        !value.truncated;
}

}  // namespace visionarm::runtime
