#include "camera/v4l2_sensor_controller.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <numeric>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

namespace visionarm {
namespace {

[[nodiscard]] std::runtime_error SystemError(const std::string& operation) {
    return std::runtime_error(
        operation + ": " + std::strerror(errno) +
        " (errno=" + std::to_string(errno) + ")");
}

int Xioctl(int fd, unsigned long request, void* argument) {
    int result = 0;
    do {
        result = ::ioctl(fd, request, argument);
    } while (result == -1 && errno == EINTR);
    return result;
}

[[nodiscard]] SensorFrameRate NormalizeInterval(
    const v4l2_fract& interval) {

    if (interval.numerator == 0U || interval.denominator == 0U) {
        throw std::runtime_error("sensor returned an invalid frame interval");
    }

    // V4L2 exposes seconds/frame. Convert to frames/second and reduce the
    // rational so MPP receives e.g. 60/1 instead of 600000/10000.
    const uint32_t gcd = std::gcd(interval.denominator, interval.numerator);
    SensorFrameRate rate;
    rate.numerator = interval.denominator / gcd;
    rate.denominator = interval.numerator / gcd;
    return rate;
}

}  // namespace

V4L2SensorController::V4L2SensorController(
    V4L2SensorControllerConfig config)
    : config_(std::move(config)) {

    if (config_.device.empty()) {
        throw std::invalid_argument("sensor subdev path must not be empty");
    }
}

bool V4L2SensorController::IsAllowedProductFps(uint32_t fps) noexcept {
    return fps == 30U || fps == 60U || fps == 90U;
}

SensorFrameRate V4L2SensorController::ConfigureFrameRate(
    uint32_t requested_fps) const {

    if (!IsAllowedProductFps(requested_fps)) {
        throw std::invalid_argument(
            "unsupported camera fps; allowed values are 30, 60, 90");
    }

    const SensorFrameRate configured =
        SetOrGetFrameRate(true, requested_fps);

    // Product policy is discrete, not nearest-match. The kernel UAPI allows a
    // driver to adjust a request to a nearby supported interval, but VisionArm
    // treats that as a configuration failure so requested/configured FPS can
    // never silently diverge.
    const uint64_t lhs = static_cast<uint64_t>(configured.numerator);
    const uint64_t rhs =
        static_cast<uint64_t>(requested_fps) * configured.denominator;
    if (lhs != rhs) {
        throw std::runtime_error(
            "sensor adjusted requested fps=" +
            std::to_string(requested_fps) + " to configured fps=" +
            std::to_string(configured.numerator) + "/" +
            std::to_string(configured.denominator));
    }

    const SensorFrameRate verified = GetFrameRate();
    if (verified.numerator != configured.numerator ||
        verified.denominator != configured.denominator) {
        throw std::runtime_error(
            "sensor frame-rate readback differs from configured value");
    }
    return verified;
}

SensorFrameRate V4L2SensorController::GetFrameRate() const {
    return SetOrGetFrameRate(false, 0U);
}

SensorFrameRate V4L2SensorController::SetOrGetFrameRate(
    bool set,
    uint32_t requested_fps) const {

    struct stat status{};
    if (::stat(config_.device.c_str(), &status) != 0) {
        throw SystemError("stat(" + config_.device + ")");
    }
    if (!S_ISCHR(status.st_mode)) {
        throw std::runtime_error(
            config_.device + " is not a character device");
    }

    const int fd = ::open(config_.device.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        throw SystemError("open(" + config_.device + ")");
    }

    v4l2_subdev_frame_interval frame_interval{};
    frame_interval.pad = config_.pad;
    if (set) {
        frame_interval.interval.numerator = 1U;
        frame_interval.interval.denominator = requested_fps;
    }

    const unsigned long request = set
        ? VIDIOC_SUBDEV_S_FRAME_INTERVAL
        : VIDIOC_SUBDEV_G_FRAME_INTERVAL;
    const int ioctl_result = Xioctl(fd, request, &frame_interval);
    const int saved_errno = errno;
    (void)::close(fd);
    errno = saved_errno;

    if (ioctl_result != 0) {
        throw SystemError(
            set ? "VIDIOC_SUBDEV_S_FRAME_INTERVAL"
                : "VIDIOC_SUBDEV_G_FRAME_INTERVAL");
    }

    return NormalizeInterval(frame_interval.interval);
}

}  // namespace visionarm
