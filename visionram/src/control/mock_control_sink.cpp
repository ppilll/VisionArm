#include "control/control_sink.h"

#include <cstddef>

namespace visionarm {

bool MockControlSink::Submit(const ControlResult& result) noexcept {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.submissions;
        if (result.valid) {
            ++snapshot_.valid_controls;
        } else {
            ++snapshot_.invalid_controls;
        }
        const std::size_t index = static_cast<std::size_t>(result.state);
        if (index < snapshot_.state_counts.size()) {
            ++snapshot_.state_counts[index];
        }
        snapshot_.latest = result;
        return true;
    } catch (...) {
        return false;
    }
}

MockControlSinkSnapshot MockControlSink::Snapshot() const noexcept {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    } catch (...) {
        MockControlSinkSnapshot failed;
        failed.submission_failures = 1U;
        return failed;
    }
}

}  // namespace visionarm
