#include "common/monotonic_clock.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <time.h>

namespace visionarm {

int64_t MonotonicNowNs() {
    timespec now{};
    if (::clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        throw std::runtime_error(
            std::string("clock_gettime(CLOCK_MONOTONIC): ") +
            std::strerror(errno));
    }

    return static_cast<int64_t>(now.tv_sec) * 1'000'000'000LL +
           static_cast<int64_t>(now.tv_nsec);
}

int64_t TimevalToNs(long seconds, long microseconds) noexcept {
    return static_cast<int64_t>(seconds) * 1'000'000'000LL +
           static_cast<int64_t>(microseconds) * 1'000LL;
}

}  // namespace visionarm
