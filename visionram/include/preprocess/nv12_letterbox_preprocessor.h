#pragma once

#include "preprocess/image_preprocessor.h"
#include "preprocess/letterbox_geometry.h"

#include <cstdint>

namespace visionarm {

struct Nv12LetterboxConfig {
    int model_width = 960;
    int model_height = 544;
    uint8_t padding_value = 114;
    ResizeGeometryPolicy resize_policy{};
};

// CPU/OpenCV fallback implementation. It consumes the V4L2 MMAP view directly
// and writes into a caller-provided persistent RKNN input slot. A production
// RGA implementation should implement IImagePreprocessor with the same
// interface and use source.planes[*].dma_fd -> destination.dma_fd.
class Nv12LetterboxPreprocessor final : public IImagePreprocessor {
public:
    explicit Nv12LetterboxPreprocessor(Nv12LetterboxConfig config);

    [[nodiscard]] PreprocessorSourceAccess source_access() const noexcept override {
        return PreprocessorSourceAccess::CPU_MMAP_READ;
    }

    [[nodiscard]] PreprocessorDestinationAccess destination_access()
        const noexcept override {
        return PreprocessorDestinationAccess::CPU_MMAP_WRITE;
    }

    [[nodiscard]] bool Process(
        const CaptureFrameView& source,
        const ModelInputBufferView& destination,
        PreprocessTransform* transform) const override;

private:
    Nv12LetterboxConfig config_;
};

}  // namespace visionarm
