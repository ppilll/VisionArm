#include "metrics/latency_accumulator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace visionarm {
namespace {

[[nodiscard]] double ToMilliseconds(int64_t nanoseconds) noexcept {
    return static_cast<double>(nanoseconds) / 1.0e6;
}

[[nodiscard]] double Percentile(
    const std::vector<int64_t>& sorted,
    double probability) noexcept {
    if (sorted.empty()) {
        return 0.0;
    }
    const double position = probability *
        static_cast<double>(sorted.size() - 1U);
    const std::size_t index = static_cast<std::size_t>(std::llround(position));
    return ToMilliseconds(sorted[index]);
}

}  // namespace

LatencyAccumulator::LatencyAccumulator(std::size_t capacity) {
    if (capacity == 0U) {
        throw std::invalid_argument("latency capacity must be > 0");
    }
    samples_.reserve(capacity);
}

void LatencyAccumulator::Add(int64_t nanoseconds) noexcept {
    if (nanoseconds < 0) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    ++total_samples_;
    total_nanoseconds_ += static_cast<long double>(nanoseconds);
    maximum_nanoseconds_ = std::max(maximum_nanoseconds_, nanoseconds);

    if (samples_.size() < samples_.capacity()) {
        samples_.push_back(nanoseconds);
        return;
    }
    samples_[next_index_] = nanoseconds;
    next_index_ = (next_index_ + 1U) % samples_.capacity();
}

LatencyDistributionSnapshot LatencyAccumulator::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    LatencyDistributionSnapshot snapshot;
    snapshot.total_samples = total_samples_;
    snapshot.retained_samples = samples_.size();
    snapshot.truncated = total_samples_ > samples_.size();
    if (total_samples_ == 0U) {
        return snapshot;
    }

    snapshot.mean_ms = static_cast<double>(
        total_nanoseconds_ / static_cast<long double>(total_samples_)) / 1.0e6;
    snapshot.maximum_ms = ToMilliseconds(maximum_nanoseconds_);

    std::vector<int64_t> sorted = samples_;
    std::sort(sorted.begin(), sorted.end());
    snapshot.p50_ms = Percentile(sorted, 0.50);
    snapshot.p95_ms = Percentile(sorted, 0.95);
    snapshot.p99_ms = Percentile(sorted, 0.99);
    return snapshot;
}

void LatencyAccumulator::Reset() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
    next_index_ = 0U;
    total_samples_ = 0U;
    total_nanoseconds_ = 0.0L;
    maximum_nanoseconds_ = 0;
}

}  // namespace visionarm
