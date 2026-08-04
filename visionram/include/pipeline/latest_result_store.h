#pragma once

#include "common/pipeline_types.h"

#include <mutex>
#include <optional>
#include <utility>

namespace visionarm {

class IPerceptionSink {
public:
    virtual ~IPerceptionSink() = default;
    virtual void Publish(PerceptionPacket packet) = 0;
};

// Stores only the most recent result. This avoids accumulating historical
// inference results when downstream business logic is slower than the NPU.
class LatestResultStore final : public IPerceptionSink {
public:
    void Publish(PerceptionPacket packet) override {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(packet);
    }

    [[nodiscard]] std::optional<PerceptionPacket> GetCopy() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }

private:
    mutable std::mutex mutex_;
    std::optional<PerceptionPacket> latest_;
};

}  // namespace visionarm
