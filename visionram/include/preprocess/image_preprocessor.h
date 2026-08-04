#pragma once

#include "common/pipeline_types.h"

namespace visionarm {

enum class PreprocessorSourceAccess {
    CPU_MMAP_READ,
    DMA_DEVICE_READ,
};

enum class PreprocessorDestinationAccess {
    CPU_MMAP_WRITE,
    DMA_DEVICE_WRITE,
};

class IImagePreprocessor {
public:
    virtual ~IImagePreprocessor() = default;

    [[nodiscard]] virtual PreprocessorSourceAccess source_access()
        const noexcept = 0;

    [[nodiscard]] virtual PreprocessorDestinationAccess destination_access()
        const noexcept = 0;

    // Process must finish all source-buffer reads before returning. The caller
    // releases its FrameLease immediately after this function returns.
    [[nodiscard]] virtual bool Process(
        const CaptureFrameView& source,
        const ModelInputBufferView& destination,
        PreprocessTransform* transform) const = 0;
};

}  // namespace visionarm
