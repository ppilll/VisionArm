#include "preprocess/letterbox_geometry.h"

#include <cmath>
#include <iostream>

namespace {

bool NearlyEqual(float first, float second) {
    return std::fabs(first - second) < 1.0e-5F;
}

bool TestExact16By9UsesCenteredLetterbox() {
    const visionarm::ResizeGeometryPolicy policy{};

    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    if (!visionarm::ComputeModelResizeGeometry(
            1920, 1080, 960, 544, policy, &geometry, &transform)) {
        return false;
    }

    return geometry.resized_width == 960 &&
        geometry.resized_height == 540 &&
        geometry.pad_left == 0 && geometry.pad_right == 0 &&
        geometry.pad_top == 2 && geometry.pad_bottom == 2 &&
        transform.letterbox &&
        NearlyEqual(geometry.scale, 0.5F) &&
        NearlyEqual(transform.scale_x, 0.5F) &&
        NearlyEqual(transform.scale_y, 0.5F) &&
        NearlyEqual(transform.uniform_scale, 0.5F);
}

bool TestEverySupported16By9ResolutionUsesCenteredLetterbox() {
    const visionarm::ResizeGeometryPolicy policy{};
    const int sizes[][2] = {
        {3840, 2160},
        {1920, 1080},
        {1280, 720},
    };

    for (const auto& size : sizes) {
        visionarm::LetterboxGeometry geometry;
        visionarm::PreprocessTransform transform;
        if (!visionarm::ComputeModelResizeGeometry(
                size[0], size[1], 960, 544,
                policy, &geometry, &transform)) {
            return false;
        }
        if (!transform.letterbox ||
            geometry.resized_width != 960 ||
            geometry.resized_height != 540 ||
            geometry.pad_left != 0 || geometry.pad_right != 0 ||
            geometry.pad_top != 2 || geometry.pad_bottom != 2) {
            return false;
        }
    }
    return true;
}

bool TestNon16By9UsesCenteredLetterbox() {
    const visionarm::ResizeGeometryPolicy policy{};

    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    if (!visionarm::ComputeModelResizeGeometry(
            1280, 800, 960, 544, policy, &geometry, &transform)) {
        return false;
    }

    return geometry.resized_width == 870 &&
        geometry.resized_height == 544 &&
        geometry.pad_left == 45 && geometry.pad_right == 45 &&
        geometry.pad_top == 0 && geometry.pad_bottom == 0 &&
        transform.letterbox &&
        NearlyEqual(geometry.scale, 0.68F) &&
        NearlyEqual(transform.uniform_scale, 0.68F);
}

bool TestPortraitUsesCenteredLetterbox() {
    const visionarm::ResizeGeometryPolicy policy{};

    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    if (!visionarm::ComputeModelResizeGeometry(
            720, 1280, 960, 544, policy, &geometry, &transform)) {
        return false;
    }

    return geometry.resized_width == 306 &&
        geometry.resized_height == 544 &&
        geometry.pad_left == 327 && geometry.pad_right == 327 &&
        geometry.pad_top == 0 && geometry.pad_bottom == 0 &&
        transform.letterbox;
}

bool TestOptionalStretchModeRemainsAvailable() {
    visionarm::ResizeGeometryPolicy policy{};
    policy.stretch_matching_source_aspect_ratio = true;

    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    if (!visionarm::ComputeModelResizeGeometry(
            1920, 1080, 960, 544, policy, &geometry, &transform)) {
        return false;
    }

    return geometry.resized_width == 960 &&
        geometry.resized_height == 544 &&
        geometry.pad_left == 0 && geometry.pad_right == 0 &&
        geometry.pad_top == 0 && geometry.pad_bottom == 0 &&
        !transform.letterbox &&
        NearlyEqual(transform.scale_x, 0.5F) &&
        NearlyEqual(transform.scale_y, 544.0F / 1080.0F) &&
        NearlyEqual(transform.uniform_scale, 0.0F);
}

bool TestInvalid() {
    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;

    visionarm::ResizeGeometryPolicy policy{};
    const bool invalid_source = !visionarm::ComputeModelResizeGeometry(
        0, 720, 960, 544, policy, &geometry, &transform);

    policy.stretch_matching_source_aspect_ratio = true;
    policy.source_aspect_width = 0;
    const bool invalid_stretch_policy = !visionarm::ComputeModelResizeGeometry(
        1280, 720, 960, 544, policy, &geometry, &transform);

    return invalid_source && invalid_stretch_policy;
}

}  // namespace

int main() {
    const bool passed =
        TestExact16By9UsesCenteredLetterbox() &&
        TestEverySupported16By9ResolutionUsesCenteredLetterbox() &&
        TestNon16By9UsesCenteredLetterbox() &&
        TestPortraitUsesCenteredLetterbox() &&
        TestOptionalStretchModeRemainsAvailable() &&
        TestInvalid();

    std::cout << (passed ? "letterbox_geometry_test PASSED\n"
                         : "letterbox_geometry_test FAILED\n");
    return passed ? 0 : 1;
}
