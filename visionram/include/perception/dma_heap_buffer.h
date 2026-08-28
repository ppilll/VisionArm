#pragma once

#include <cstddef>
#include <string>

namespace visionarm {

// Owns one DMA-BUF allocated from a Linux dma-heap device.
//
// The selected heap path determines the allocation properties. On the
// Rockchip RK3588 BSP, /dev/dma_heap/system-uncached-dma32 is normally used
// when an RGA2-only operation must access every page below the 4 GiB boundary.
class DmaHeapBuffer final {
public:
    DmaHeapBuffer() = default;
    ~DmaHeapBuffer();

    DmaHeapBuffer(const DmaHeapBuffer&) = delete;
    DmaHeapBuffer& operator=(const DmaHeapBuffer&) = delete;

    DmaHeapBuffer(DmaHeapBuffer&& other) noexcept;
    DmaHeapBuffer& operator=(DmaHeapBuffer&& other) noexcept;

    [[nodiscard]] static DmaHeapBuffer Allocate(
        const std::string& heap_path,
        std::size_t requested_size);

    void Reset() noexcept;

    [[nodiscard]] bool valid() const noexcept {
        return fd_ >= 0 && address_ != nullptr && size_ != 0U;
    }
    [[nodiscard]] int fd() const noexcept { return fd_; }
    [[nodiscard]] void* address() const noexcept { return address_; }
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] const std::string& heap_path() const noexcept {
        return heap_path_;
    }

private:
    int fd_ = -1;
    void* address_ = nullptr;
    std::size_t size_ = 0U;
    std::string heap_path_;
};

}  // namespace visionarm
