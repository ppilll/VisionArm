#include "video/nv12_mpp_layout.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <linux/videodev2.h>

namespace visionarm {

bool DeriveNv12MppLayout(
    const CaptureFrameView& frame,
    int configured_vertical_stride,
    Nv12MppLayout* layout) noexcept {
    if (layout == nullptr || frame.width <= 0 || frame.height <= 0 ||
        frame.pixel_format != V4L2_PIX_FMT_NV12 || frame.plane_count != 1U) {
        return false;
    }

    const CapturePlaneView& plane = frame.planes[0];
    if (plane.dma_fd < 0 || plane.data_offset != 0U ||
        plane.stride < static_cast<uint32_t>(frame.width) ||
        plane.allocation_length == 0U) {
        return false;
    }

    const uint64_t stride = plane.stride;
    int vertical_stride = configured_vertical_stride;
    if (vertical_stride <= 0) {
        const uint64_t encoded_bytes =
            plane.size_image != 0U
                ? static_cast<uint64_t>(plane.size_image)
                : static_cast<uint64_t>(plane.allocation_length);
        const uint64_t denominator = stride * 3U;
        if (denominator == 0U ||
            encoded_bytes > std::numeric_limits<uint64_t>::max() / 2U) {
            return false;
        }
        const uint64_t numerator = encoded_bytes * 2U;
        if (numerator % denominator != 0U) {
            return false;
        }
        const uint64_t derived = numerator / denominator;
        if (derived < static_cast<uint64_t>(frame.height) ||
            derived > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
            return false;
        }
        vertical_stride = static_cast<int>(derived);
    }

    if (vertical_stride < frame.height) {
        return false;
    }

    const uint64_t minimum =
        stride * static_cast<uint64_t>(vertical_stride) * 3U / 2U;
    if (minimum > plane.allocation_length || minimum > plane.size_image) {
        return false;
    }

    layout->width = frame.width;
    layout->height = frame.height;
    layout->horizontal_stride = static_cast<int>(plane.stride);
    layout->vertical_stride = vertical_stride;
    layout->minimum_bytes = minimum;
    return true;
}

}  // namespace visionarm
