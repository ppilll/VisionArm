#pragma once

#include <cstdint>
#include <string>

namespace visionarm {

struct SensorFrameRate {
    uint32_t numerator = 0;    // frames / second numerator
    uint32_t denominator = 0;  // frames / second denominator

    [[nodiscard]] bool valid() const noexcept {
        return numerator != 0U && denominator != 0U;
    }

    [[nodiscard]] double AsDouble() const noexcept {
        return valid()
            ? static_cast<double>(numerator) /
                  static_cast<double>(denominator)
            : 0.0;
    }
};

struct V4L2SensorControllerConfig {
    std::string device;
    uint32_t pad = 0;
};

class V4L2SensorController final {
public:
    explicit V4L2SensorController(V4L2SensorControllerConfig config);

    V4L2SensorController(const V4L2SensorController&) = delete;
    V4L2SensorController& operator=(const V4L2SensorController&) = delete;

    [[nodiscard]] SensorFrameRate ConfigureFrameRate(uint32_t requested_fps) const;
    [[nodiscard]] SensorFrameRate GetFrameRate() const;

    [[nodiscard]] static bool IsAllowedProductFps(uint32_t fps) noexcept;

private:
    [[nodiscard]] SensorFrameRate SetOrGetFrameRate(
        bool set,
        uint32_t requested_fps) const;

    V4L2SensorControllerConfig config_;
};

}  // namespace visionarm
