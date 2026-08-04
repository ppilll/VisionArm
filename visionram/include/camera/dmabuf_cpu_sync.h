#pragma once

#include "common/pipeline_types.h"

#include <array>
#include <cstddef>

namespace visionarm {

enum class DmabufCpuAccessMode {
    READ,
    WRITE,
    READ_WRITE,
};

constexpr std::size_t kMaxDmabufCpuSyncFds = 16U;

class DmabufCpuAccessGuard final {
public:
    DmabufCpuAccessGuard() = default;
    ~DmabufCpuAccessGuard();

    DmabufCpuAccessGuard(const DmabufCpuAccessGuard&) = delete;
    DmabufCpuAccessGuard& operator=(const DmabufCpuAccessGuard&) = delete;

    [[nodiscard]] bool Begin(
        const CaptureFrameView& frame,
        DmabufCpuAccessMode mode,
        int* failed_fd = nullptr,
        int* error_number = nullptr) noexcept;

    [[nodiscard]] bool BeginFd(
        int fd,
        DmabufCpuAccessMode mode,
        int* error_number = nullptr) noexcept;

    [[nodiscard]] bool BeginFds(
        const int* fds,
        std::size_t count,
        DmabufCpuAccessMode mode,
        int* failed_fd = nullptr,
        int* error_number = nullptr) noexcept;

    [[nodiscard]] bool End(
        int* failed_fd = nullptr,
        int* error_number = nullptr) noexcept;

    [[nodiscard]] bool active() const noexcept {
        return active_count_ != 0U;
    }

private:
    [[nodiscard]] static bool SyncIoctl(
        int fd,
        unsigned long flags,
        int* error_number) noexcept;

    std::array<int, kMaxDmabufCpuSyncFds> active_fds_{};
    std::size_t active_count_ = 0U;
    unsigned long access_flags_ = 0UL;
};

}  // namespace visionarm
