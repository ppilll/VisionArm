#pragma once

#include "preprocess/image_preprocessor.h"
#include "preprocess/letterbox_geometry.h"

#include <cstddef>
#include <cstdint>
#include <vector>

#include <im2d.hpp>

namespace visionarm {

struct RgaLetterboxConfig {
    int model_width = 960;
    int model_height = 544;
    uint8_t padding_value = 114;
    ResizeGeometryPolicy resize_policy{};

    // These capacities must match the already-opened V4L2 camera buffer count
    // and the initialized RKNN engine input slot count. Handles are imported
    // lazily and then reused; no per-frame import/release is performed.
    std::size_t max_source_buffers = 0U;
    std::size_t max_destination_slots = 0U;

    // The current ping-pong model and OpenCV baseline use RGB with the normal
    // limited-range NV12 conversion. Change only after an image-level parity
    // test demonstrates that the camera advertises a different colorimetry.
    int color_space_mode = IM_YUV_TO_RGB_BT601_LIMIT;
};

struct RgaPreprocessorSnapshot {
    uint64_t process_calls = 0U;
    uint64_t process_successes = 0U;
    uint64_t validation_failures = 0U;
    uint64_t import_failures = 0U;
    uint64_t fill_failures = 0U;
    uint64_t process_failures = 0U;
    uint64_t source_imports = 0U;
    uint64_t destination_imports = 0U;
    uint64_t direct_resize_successes = 0U;
    uint64_t letterbox_successes = 0U;
    uint64_t fill_operations = 0U;
};

// Synchronous RGA implementation for linear, single-allocation NV12.
//
// The current public im2d buffer contract represents one image with one fd or
// one imported handle. Therefore separate-fd NV12M is intentionally rejected;
// do not silently treat plane 0 as if it also contained UV data.
//
// Process() is intended to be called by one dedicated preprocess thread.
class RgaLetterboxPreprocessor final : public IImagePreprocessor {
public:
    explicit RgaLetterboxPreprocessor(RgaLetterboxConfig config);
    ~RgaLetterboxPreprocessor() override;

    RgaLetterboxPreprocessor(const RgaLetterboxPreprocessor&) = delete;
    RgaLetterboxPreprocessor& operator=(
        const RgaLetterboxPreprocessor&) = delete;

    [[nodiscard]] PreprocessorSourceAccess source_access()
        const noexcept override {
        return PreprocessorSourceAccess::DMA_DEVICE_READ;
    }

    [[nodiscard]] PreprocessorDestinationAccess destination_access()
        const noexcept override {
        return PreprocessorDestinationAccess::DMA_DEVICE_WRITE;
    }

    [[nodiscard]] bool Process(
        const CaptureFrameView& source,
        const ModelInputBufferView& destination,
        PreprocessTransform* transform) const override;

    [[nodiscard]] RgaPreprocessorSnapshot snapshot() const noexcept;

private:
    struct ImportedBuffer {
        int fd = -1;
        std::size_t size = 0U;
        int width = 0;
        int height = 0;
        int width_stride = 0;
        int height_stride = 0;
        int format = RK_FORMAT_UNKNOWN;
        rga_buffer_handle_t handle = 0U;
    };

    [[nodiscard]] bool Validate(
        const CaptureFrameView& source,
        const ModelInputBufferView& destination) const noexcept;

    [[nodiscard]] bool EnsureSourceImported(
        const CaptureFrameView& source,
        ImportedBuffer** imported) const noexcept;

    [[nodiscard]] bool EnsureDestinationImported(
        const ModelInputBufferView& destination,
        ImportedBuffer** imported) const noexcept;

    static void ReleaseImported(ImportedBuffer* imported) noexcept;
    void ReleaseAll() noexcept;

    RgaLetterboxConfig config_;
    mutable std::vector<ImportedBuffer> source_buffers_;
    mutable std::vector<ImportedBuffer> destination_buffers_;
    mutable RgaPreprocessorSnapshot snapshot_;
};

}  // namespace visionarm
