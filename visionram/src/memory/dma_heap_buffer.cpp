#include "memory/dma_heap_buffer.h"

#if __has_include(<linux/dma-heap.h>)
#include <linux/dma-heap.h>
#else
#include <linux/ioctl.h>
#include <linux/types.h>

struct dma_heap_allocation_data {
    __u64 len;
    __u32 fd;
    __u32 fd_flags;
    __u64 heap_flags;
};

#define DMA_HEAP_IOC_MAGIC 'H'
#define DMA_HEAP_IOCTL_ALLOC \
    _IOWR(DMA_HEAP_IOC_MAGIC, 0x0, struct dma_heap_allocation_data)
#endif

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>

namespace visionarm {
namespace {

[[nodiscard]] std::string ErrorMessage(
    const std::string& operation,
    int error_number) {
    return operation + " failed: " + std::strerror(error_number) +
        " (errno=" + std::to_string(error_number) + ")";
}

[[nodiscard]] std::size_t AlignUp(
    std::size_t value,
    std::size_t alignment) {
    if (value == 0U || alignment == 0U ||
        (alignment & (alignment - 1U)) != 0U ||
        value > std::numeric_limits<std::size_t>::max() - (alignment - 1U)) {
        throw std::invalid_argument("invalid DMA heap allocation size");
    }
    return (value + alignment - 1U) & ~(alignment - 1U);
}

}  // namespace

DmaHeapBuffer::~DmaHeapBuffer() {
    Reset();
}

DmaHeapBuffer::DmaHeapBuffer(DmaHeapBuffer&& other) noexcept
    : fd_(std::exchange(other.fd_, -1)),
      address_(std::exchange(other.address_, nullptr)),
      size_(std::exchange(other.size_, 0U)),
      heap_path_(std::move(other.heap_path_)) {}

DmaHeapBuffer& DmaHeapBuffer::operator=(DmaHeapBuffer&& other) noexcept {
    if (this != &other) {
        Reset();
        fd_ = std::exchange(other.fd_, -1);
        address_ = std::exchange(other.address_, nullptr);
        size_ = std::exchange(other.size_, 0U);
        heap_path_ = std::move(other.heap_path_);
    }
    return *this;
}

DmaHeapBuffer DmaHeapBuffer::Allocate(
    const std::string& heap_path,
    std::size_t requested_size) {
    if (heap_path.empty()) {
        throw std::invalid_argument("DMA heap path is empty");
    }

    const long page_size_value = ::sysconf(_SC_PAGESIZE);
    if (page_size_value <= 0) {
        throw std::runtime_error("failed to query system page size");
    }
    const std::size_t page_size =
        static_cast<std::size_t>(page_size_value);
    const std::size_t allocation_size =
        AlignUp(requested_size, page_size);

    const int heap_fd = ::open(heap_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (heap_fd < 0) {
        const int saved_errno = errno;
        throw std::runtime_error(
            ErrorMessage("open(" + heap_path + ")", saved_errno));
    }

    dma_heap_allocation_data allocation{};
    allocation.len = static_cast<__u64>(allocation_size);
    allocation.fd_flags = static_cast<__u32>(O_RDWR | O_CLOEXEC);
    allocation.heap_flags = 0U;

    if (::ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &allocation) < 0) {
        const int saved_errno = errno;
        (void)::close(heap_fd);
        throw std::runtime_error(
            ErrorMessage("DMA_HEAP_IOCTL_ALLOC(" + heap_path + ")",
                         saved_errno));
    }
    (void)::close(heap_fd);

    const int buffer_fd = static_cast<int>(allocation.fd);
    void* address = ::mmap(
        nullptr,
        allocation_size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        buffer_fd,
        0);
    if (address == MAP_FAILED) {
        const int saved_errno = errno;
        (void)::close(buffer_fd);
        throw std::runtime_error(
            ErrorMessage("mmap(DMA-BUF)", saved_errno));
    }

    DmaHeapBuffer result;
    result.fd_ = buffer_fd;
    result.address_ = address;
    result.size_ = allocation_size;
    result.heap_path_ = heap_path;
    return result;
}

void DmaHeapBuffer::Reset() noexcept {
    if (address_ != nullptr && size_ != 0U) {
        if (::munmap(address_, size_) != 0) {
            // Destructors and shutdown paths must remain noexcept. The fd is
            // still closed below so the exporter can reclaim the allocation.
        }
    }
    if (fd_ >= 0) {
        (void)::close(fd_);
    }
    fd_ = -1;
    address_ = nullptr;
    size_ = 0U;
    heap_path_.clear();
}

}  // namespace visionarm
