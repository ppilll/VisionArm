#pragma once

#include "common/pipeline_types.h"

namespace visionarm {

struct LetterboxGeometry {
    int resized_width = 0;
    int resized_height = 0;
    int pad_left = 0;
    int pad_top = 0;
    int pad_right = 0;
    int pad_bottom = 0;
    float scale = 0.0F;
};

[[nodiscard]] bool ComputeCenteredLetterbox(
    int source_width,
    int source_height,
    int destination_width,
    int destination_height,
    LetterboxGeometry* geometry,
    PreprocessTransform* transform) noexcept;

}  // namespace visionarm
