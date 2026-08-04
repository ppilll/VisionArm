#pragma once

#include "common/pipeline_types.h"

#include <cstdint>

namespace visionarm {

struct Nv12MppLayout {
    int width = 0;
    int height = 0;
    int horizontal_stride = 0;
    int vertical_stride = 0;
    uint64_t minimum_bytes = 0U;
};

// Derives the physical NV12 vertical stride from the frozen V4L2 contract.
// The source must be one-plane linear NV12 with data_offset == 0.
[[nodiscard]] bool DeriveNv12MppLayout(
    const CaptureFrameView& frame,
    int configured_vertical_stride,
    Nv12MppLayout* layout) noexcept;

}  // namespace visionarm
