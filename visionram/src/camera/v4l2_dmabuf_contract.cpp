#include "camera/v4l2_dmabuf_contract.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <linux/videodev2.h>

namespace visionarm {
namespace {

V4L2DmabufContractResult Failure(
    V4L2DmabufContractError error,
    uint32_t buffer_index,
    uint32_t plane_index,
    std::size_t expected,
    std::size_t actual) noexcept {
    return {false, error, buffer_index, plane_index, expected, actual};
}

V4L2DmabufContractResult ValidatePlane(
    const CaptureFrameView& frame,
    uint32_t plane_index,
    std::size_t expected_bytes) noexcept {

    const CapturePlaneView& plane = frame.planes[plane_index];
    if (plane.stride < static_cast<uint32_t>(frame.width)) {
        return Failure(
            V4L2DmabufContractError::STRIDE_TOO_SMALL,
            frame.buffer_index,
            plane_index,
            static_cast<std::size_t>(frame.width),
            plane.stride);
    }
    if (plane.size_image < expected_bytes) {
        return Failure(
            V4L2DmabufContractError::SIZE_IMAGE_TOO_SMALL,
            frame.buffer_index,
            plane_index,
            expected_bytes,
            plane.size_image);
    }
    if (plane.bytes_used < expected_bytes) {
        return Failure(
            V4L2DmabufContractError::BYTES_USED_TOO_SMALL,
            frame.buffer_index,
            plane_index,
            expected_bytes,
            plane.bytes_used);
    }
    const std::size_t available =
        plane.allocation_length - plane.data_offset;
    if (available < expected_bytes) {
        return Failure(
            V4L2DmabufContractError::ALLOCATION_TOO_SMALL,
            frame.buffer_index,
            plane_index,
            expected_bytes,
            available);
    }
    return {true, V4L2DmabufContractError::NONE, frame.buffer_index, 0U, 0U, 0U};
}

}  // namespace

V4L2DmabufContractResult ValidateNv12DmabufFrame(
    const CaptureFrameView& frame) noexcept {

    const CaptureFrameValidation structural =
        ValidateCaptureFrameView(frame, true);
    if (!structural.ok) {
        return Failure(
            V4L2DmabufContractError::STRUCTURAL_FRAME_ERROR,
            frame.buffer_index,
            structural.plane_index,
            0U,
            0U);
    }

    if (frame.pixel_format != V4L2_PIX_FMT_NV12 &&
        frame.pixel_format != V4L2_PIX_FMT_NV12M) {
        return Failure(
            V4L2DmabufContractError::UNSUPPORTED_PIXEL_FORMAT,
            frame.buffer_index,
            0U,
            0U,
            frame.pixel_format);
    }
    if ((frame.width & 1) != 0 || (frame.height & 1) != 0) {
        return Failure(
            V4L2DmabufContractError::ODD_FRAME_DIMENSION,
            frame.buffer_index,
            0U,
            0U,
            0U);
    }

    const std::size_t height = static_cast<std::size_t>(frame.height);
    const std::size_t chroma_height = height / 2U;

    if (frame.pixel_format == V4L2_PIX_FMT_NV12) {
        if (frame.plane_count != 1U) {
            return Failure(
                V4L2DmabufContractError::PLANE_COUNT_MISMATCH,
                frame.buffer_index,
                0U,
                1U,
                frame.plane_count);
        }
        const std::size_t stride = frame.planes[0].stride;
        return ValidatePlane(
            frame,
            0U,
            stride * height + stride * chroma_height);
    }

    if (frame.plane_count != 2U) {
        return Failure(
            V4L2DmabufContractError::PLANE_COUNT_MISMATCH,
            frame.buffer_index,
            0U,
            2U,
            frame.plane_count);
    }

    const V4L2DmabufContractResult y_result = ValidatePlane(
        frame,
        0U,
        static_cast<std::size_t>(frame.planes[0].stride) * height);
    if (!y_result.ok) {
        return y_result;
    }
    return ValidatePlane(
        frame,
        1U,
        static_cast<std::size_t>(frame.planes[1].stride) * chroma_height);
}

V4L2DmabufContractResult ValidateV4L2DmabufInventory(
    const V4L2DmabufInventory& inventory,
    const V4L2DmabufInventoryOptions& options) noexcept {

    if (inventory.buffers.empty()) {
        return Failure(
            V4L2DmabufContractError::INVENTORY_EMPTY,
            0U,
            0U,
            1U,
            0U);
    }

    for (const V4L2DmabufBufferInventory& buffer : inventory.buffers) {
        if (buffer.planes.size() != inventory.plane_count) {
            return Failure(
                V4L2DmabufContractError::INVENTORY_PLANE_COUNT_MISMATCH,
                buffer.buffer_index,
                0U,
                inventory.plane_count,
                buffer.planes.size());
        }
        for (const V4L2DmabufPlaneInventory& plane : buffer.planes) {
            if (plane.dma_fd < 0) {
                return Failure(
                    V4L2DmabufContractError::INVENTORY_INVALID_FD,
                    buffer.buffer_index,
                    plane.plane_index,
                    0U,
                    0U);
            }
            if (options.require_cloexec && !plane.fd_cloexec) {
                return Failure(
                    V4L2DmabufContractError::INVENTORY_CLOEXEC_MISSING,
                    buffer.buffer_index,
                    plane.plane_index,
                    1U,
                    0U);
            }
            if (options.require_size_query && !plane.size_query_ok) {
                return Failure(
                    V4L2DmabufContractError::INVENTORY_SIZE_QUERY_FAILED,
                    buffer.buffer_index,
                    plane.plane_index,
                    1U,
                    0U);
            }
            if (options.require_fd_size_at_least_mmap_length &&
                plane.size_query_ok && plane.fd_size < plane.mmap_length) {
                return Failure(
                    V4L2DmabufContractError::INVENTORY_FD_SIZE_TOO_SMALL,
                    buffer.buffer_index,
                    plane.plane_index,
                    plane.mmap_length,
                    plane.fd_size);
            }
        }
    }

    return {true, V4L2DmabufContractError::NONE, 0U, 0U, 0U, 0U};
}

const char* ToString(V4L2DmabufContractError error) noexcept {
    switch (error) {
        case V4L2DmabufContractError::NONE:
            return "NONE";
        case V4L2DmabufContractError::STRUCTURAL_FRAME_ERROR:
            return "STRUCTURAL_FRAME_ERROR";
        case V4L2DmabufContractError::UNSUPPORTED_PIXEL_FORMAT:
            return "UNSUPPORTED_PIXEL_FORMAT";
        case V4L2DmabufContractError::ODD_FRAME_DIMENSION:
            return "ODD_FRAME_DIMENSION";
        case V4L2DmabufContractError::PLANE_COUNT_MISMATCH:
            return "PLANE_COUNT_MISMATCH";
        case V4L2DmabufContractError::STRIDE_TOO_SMALL:
            return "STRIDE_TOO_SMALL";
        case V4L2DmabufContractError::SIZE_IMAGE_TOO_SMALL:
            return "SIZE_IMAGE_TOO_SMALL";
        case V4L2DmabufContractError::BYTES_USED_TOO_SMALL:
            return "BYTES_USED_TOO_SMALL";
        case V4L2DmabufContractError::ALLOCATION_TOO_SMALL:
            return "ALLOCATION_TOO_SMALL";
        case V4L2DmabufContractError::INVENTORY_EMPTY:
            return "INVENTORY_EMPTY";
        case V4L2DmabufContractError::INVENTORY_PLANE_COUNT_MISMATCH:
            return "INVENTORY_PLANE_COUNT_MISMATCH";
        case V4L2DmabufContractError::INVENTORY_INVALID_FD:
            return "INVENTORY_INVALID_FD";
        case V4L2DmabufContractError::INVENTORY_CLOEXEC_MISSING:
            return "INVENTORY_CLOEXEC_MISSING";
        case V4L2DmabufContractError::INVENTORY_SIZE_QUERY_FAILED:
            return "INVENTORY_SIZE_QUERY_FAILED";
        case V4L2DmabufContractError::INVENTORY_FD_SIZE_TOO_SMALL:
            return "INVENTORY_FD_SIZE_TOO_SMALL";
    }
    return "UNKNOWN";
}

}  // namespace visionarm
