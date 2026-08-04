#pragma once

#include <cstdint>

namespace visionarm {

[[nodiscard]] int64_t MonotonicNowNs();
[[nodiscard]] int64_t TimevalToNs(long seconds, long microseconds) noexcept;

}  // namespace visionarm
