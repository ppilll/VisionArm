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

// The production default is aspect-preserving centered letterbox for every
// source resolution. For a 16:9 source and a 960x544 model tensor this produces
// a 960x540 active image with 2 rows of padding at both the top and bottom.
//
// The optional stretch mode is retained only for backward compatibility and
// controlled experiments. Production callers should leave it disabled.
struct ResizeGeometryPolicy {
    bool stretch_matching_source_aspect_ratio = false;
    int source_aspect_width = 16;
    int source_aspect_height = 9;
};

[[nodiscard]] bool ComputeCenteredLetterbox(
    int source_width,
    int source_height,
    int destination_width,
    int destination_height,
    LetterboxGeometry* geometry,
    PreprocessTransform* transform) noexcept;

[[nodiscard]] bool ComputeModelResizeGeometry(
    int source_width,
    int source_height,
    int destination_width,
    int destination_height,
    const ResizeGeometryPolicy& policy,
    LetterboxGeometry* geometry,
    PreprocessTransform* transform) noexcept;

}  // namespace visionarm
