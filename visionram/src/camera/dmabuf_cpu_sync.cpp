#include "camera/dmabuf_cpu_sync.h"

#include <cerrno>
#include <cstdint>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>

namespace visionarm {
namespace {

unsigned long AccessFlags(DmabufCpuAccessMode mode) noexcept {
    switch (mode) {
        case DmabufCpuAccessMode::READ: return DMA_BUF_SYNC_READ;
        case DmabufCpuAccessMode::WRITE: return DMA_BUF_SYNC_WRITE;
        case DmabufCpuAccessMode::READ_WRITE: return DMA_BUF_SYNC_RW;
    }
    return DMA_BUF_SYNC_RW;
}

}  // namespace

DmabufCpuAccessGuard::~DmabufCpuAccessGuard() {
    (void)End();
}

bool DmabufCpuAccessGuard::SyncIoctl(
    int fd,
    unsigned long flags,
    int* error_number) noexcept {
    dma_buf_sync sync{};
    sync.flags = static_cast<__u64>(flags);
    while (::ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) != 0) {
        if (errno == EINTR || errno == EAGAIN) {
            continue;
        }
        if (error_number != nullptr) {
            *error_number = errno;
        }
        return false;
    }
    if (error_number != nullptr) {
        *error_number = 0;
    }
    return true;
}

bool DmabufCpuAccessGuard::Begin(
    const CaptureFrameView& frame,
    DmabufCpuAccessMode mode,
    int* failed_fd,
    int* error_number) noexcept {
    if (frame.plane_count > kMaxFramePlanes) {
        if (error_number != nullptr) {
            *error_number = EINVAL;
        }
        return false;
    }
    std::array<int, kMaxFramePlanes> fds{};
    for (uint32_t index = 0U; index < frame.plane_count; ++index) {
        fds[index] = frame.planes[index].dma_fd;
    }
    return BeginFds(
        fds.data(), frame.plane_count, mode, failed_fd, error_number);
}

bool DmabufCpuAccessGuard::BeginFd(
    int fd,
    DmabufCpuAccessMode mode,
    int* error_number) noexcept {
    int failed_fd = -1;
    return BeginFds(&fd, 1U, mode, &failed_fd, error_number);
}

bool DmabufCpuAccessGuard::BeginFds(
    const int* fds,
    std::size_t count,
    DmabufCpuAccessMode mode,
    int* failed_fd,
    int* error_number) noexcept {
    if (active()) {
        if (error_number != nullptr) {
            *error_number = EBUSY;
        }
        return false;
    }
    if ((count != 0U && fds == nullptr) ||
        count > active_fds_.size()) {
        if (error_number != nullptr) {
            *error_number = EINVAL;
        }
        return false;
    }

    access_flags_ = AccessFlags(mode);
    for (std::size_t input_index = 0U;
         input_index < count;
         ++input_index) {
        const int fd = fds[input_index];
        if (fd < 0) {
            if (failed_fd != nullptr) {
                *failed_fd = fd;
            }
            if (error_number != nullptr) {
                *error_number = EBADF;
            }
            (void)End();
            return false;
        }

        bool duplicate = false;
        for (std::size_t active_index = 0U;
             active_index < active_count_;
             ++active_index) {
            if (active_fds_[active_index] == fd) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        int sync_error = 0;
        if (!SyncIoctl(
                fd,
                DMA_BUF_SYNC_START | access_flags_,
                &sync_error)) {
            if (failed_fd != nullptr) {
                *failed_fd = fd;
            }
            if (error_number != nullptr) {
                *error_number = sync_error;
            }
            (void)End();
            return false;
        }
        active_fds_[active_count_++] = fd;
    }

    if (failed_fd != nullptr) {
        *failed_fd = -1;
    }
    if (error_number != nullptr) {
        *error_number = 0;
    }
    return true;
}

bool DmabufCpuAccessGuard::End(
    int* failed_fd,
    int* error_number) noexcept {
    bool ok = true;
    int first_failed_fd = -1;
    int first_error = 0;
    while (active_count_ > 0U) {
        --active_count_;
        const int fd = active_fds_[active_count_];
        int sync_error = 0;
        if (!SyncIoctl(
                fd,
                DMA_BUF_SYNC_END | access_flags_,
                &sync_error)) {
            if (ok) {
                first_failed_fd = fd;
                first_error = sync_error;
            }
            ok = false;
        }
        active_fds_[active_count_] = -1;
    }
    access_flags_ = 0UL;
    if (failed_fd != nullptr) {
        *failed_fd = first_failed_fd;
    }
    if (error_number != nullptr) {
        *error_number = first_error;
    }
    return ok;
}

}  // namespace visionarm
