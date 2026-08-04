#pragma once

#include "camera/capture_buffer_contract.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace visionarm {

enum class V4L2DmabufContractError : uint8_t {
    NONE = 0,
    STRUCTURAL_FRAME_ERROR,
    UNSUPPORTED_PIXEL_FORMAT,
    ODD_FRAME_DIMENSION,
    PLANE_COUNT_MISMATCH,
    STRIDE_TOO_SMALL,
    SIZE_IMAGE_TOO_SMALL,
    BYTES_USED_TOO_SMALL,
    ALLOCATION_TOO_SMALL,
    INVENTORY_EMPTY,
    INVENTORY_PLANE_COUNT_MISMATCH,
    INVENTORY_INVALID_FD,
    INVENTORY_CLOEXEC_MISSING,
    INVENTORY_SIZE_QUERY_FAILED,
    INVENTORY_FD_SIZE_TOO_SMALL,
};

struct V4L2DmabufContractResult {
    bool ok = false;
    V4L2DmabufContractError error =
        V4L2DmabufContractError::STRUCTURAL_FRAME_ERROR;
    uint32_t buffer_index = 0U;
    uint32_t plane_index = 0U;
    std::size_t expected_minimum = 0U;
    std::size_t actual = 0U;
};

struct V4L2DmabufPlaneInventory {
    uint32_t buffer_index = 0U;
    uint32_t plane_index = 0U;
    int dma_fd = -1;
    std::size_t mmap_length = 0U;
    uint32_t stride = 0U;
    uint32_t size_image = 0U;
    bool fd_cloexec = false;
    bool size_query_ok = false;
    uint64_t fd_size = 0U;
    uint64_t device_id = 0U;
    uint64_t inode = 0U;
};

struct V4L2DmabufBufferInventory {
    uint32_t buffer_index = 0U;
    std::vector<V4L2DmabufPlaneInventory> planes;
};

struct V4L2DmabufInventory {
    uint32_t width = 0U;
    uint32_t height = 0U;
    uint32_t pixel_format = 0U;
    bool multiplanar_api = false;
    uint32_t plane_count = 0U;
    uint32_t colorspace = 0U;
    uint32_t ycbcr_encoding = 0U;
    uint32_t quantization = 0U;
    uint32_t transfer_function = 0U;
    std::vector<V4L2DmabufBufferInventory> buffers;
};

struct V4L2DmabufInventoryOptions {
    bool require_cloexec = true;
    bool require_size_query = true;
    bool require_fd_size_at_least_mmap_length = true;
};

[[nodiscard]] V4L2DmabufContractResult ValidateNv12DmabufFrame(
    const CaptureFrameView& frame) noexcept;

[[nodiscard]] V4L2DmabufContractResult ValidateV4L2DmabufInventory(
    const V4L2DmabufInventory& inventory,
    const V4L2DmabufInventoryOptions& options = {}) noexcept;

[[nodiscard]] const char* ToString(
    V4L2DmabufContractError error) noexcept;

}  // namespace visionarm
