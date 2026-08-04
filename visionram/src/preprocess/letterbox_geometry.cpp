#include "preprocess/letterbox_geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace visionarm {
namespace {

[[nodiscard]] bool MatchesAspectRatio(
    int width,
    int height,
    int aspect_width,
    int aspect_height) noexcept {

    if (width <= 0 || height <= 0 ||
        aspect_width <= 0 || aspect_height <= 0) {
        return false;
    }

    return static_cast<int64_t>(width) *
               static_cast<int64_t>(aspect_height) ==
           static_cast<int64_t>(height) *
               static_cast<int64_t>(aspect_width);
}

}  // namespace

bool ComputeCenteredLetterbox(
    int source_width,
    int source_height,
    int destination_width,
    int destination_height,
    LetterboxGeometry* geometry,
    PreprocessTransform* transform) noexcept {

    if (source_width <= 0 || source_height <= 0 ||
        destination_width <= 0 || destination_height <= 0 ||
        geometry == nullptr || transform == nullptr) {
        return false;
    }

    const float scale = std::min(
        static_cast<float>(destination_width) /
            static_cast<float>(source_width),
        static_cast<float>(destination_height) /
            static_cast<float>(source_height));

    const int resized_width = std::clamp(
        static_cast<int>(std::lround(
            static_cast<float>(source_width) * scale)),
        1,
        destination_width);
    const int resized_height = std::clamp(
        static_cast<int>(std::lround(
            static_cast<float>(source_height) * scale)),
        1,
        destination_height);

    const int remaining_width = destination_width - resized_width;
    const int remaining_height = destination_height - resized_height;
    const int pad_left = remaining_width / 2;
    const int pad_top = remaining_height / 2;

    *geometry = LetterboxGeometry{
        resized_width,
        resized_height,
        pad_left,
        pad_top,
        remaining_width - pad_left,
        remaining_height - pad_top,
        scale,
    };

    *transform = PreprocessTransform{
        source_width,
        source_height,
        destination_width,
        destination_height,
        scale,
        scale,
        scale,
        pad_left,
        pad_top,
        remaining_width - pad_left,
        remaining_height - pad_top,
        true,
    };
    return true;
}

bool ComputeModelResizeGeometry(
    int source_width,
    int source_height,
    int destination_width,
    int destination_height,
    const ResizeGeometryPolicy& policy,
    LetterboxGeometry* geometry,
    PreprocessTransform* transform) noexcept {

    if (source_width <= 0 || source_height <= 0 ||
        destination_width <= 0 || destination_height <= 0 ||
        geometry == nullptr || transform == nullptr) {
        return false;
    }

    if (policy.stretch_matching_source_aspect_ratio) {
        if (policy.source_aspect_width <= 0 ||
            policy.source_aspect_height <= 0) {
            return false;
        }

        if (MatchesAspectRatio(
                source_width,
                source_height,
                policy.source_aspect_width,
                policy.source_aspect_height)) {
            const float scale_x =
                static_cast<float>(destination_width) /
                static_cast<float>(source_width);
            const float scale_y =
                static_cast<float>(destination_height) /
                static_cast<float>(source_height);

            *geometry = LetterboxGeometry{
                destination_width,
                destination_height,
                0,
                0,
                0,
                0,
                0.0F,
            };

            *transform = PreprocessTransform{
                source_width,
                source_height,
                destination_width,
                destination_height,
                scale_x,
                scale_y,
                0.0F,
                0,
                0,
                0,
                0,
                false,
            };
            return true;
        }
    }

    return ComputeCenteredLetterbox(
        source_width,
        source_height,
        destination_width,
        destination_height,
        geometry,
        transform);
}

}  // namespace visionarm
