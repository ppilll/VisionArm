#include "preprocess/letterbox_geometry.h"

#include <cmath>
#include <iostream>

namespace {

bool NearlyEqual(float first, float second) {
    return std::fabs(first - second) < 1.0e-6F;
}

bool Test1280x720To960() {
    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    if (!visionarm::ComputeCenteredLetterbox(
            1280, 720, 960, 960, &geometry, &transform)) {
        return false;
    }
    return geometry.resized_width == 960 &&
        geometry.resized_height == 540 &&
        geometry.pad_left == 0 && geometry.pad_right == 0 &&
        geometry.pad_top == 210 && geometry.pad_bottom == 210 &&
        NearlyEqual(geometry.scale, 0.75F) &&
        transform.pad_top == geometry.pad_top;
}

bool Test1920x1080To960() {
    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    if (!visionarm::ComputeCenteredLetterbox(
            1920, 1080, 960, 960, &geometry, &transform)) {
        return false;
    }
    return geometry.resized_width == 960 &&
        geometry.resized_height == 540 &&
        geometry.pad_top == 210 && geometry.pad_bottom == 210 &&
        NearlyEqual(geometry.scale, 0.5F);
}

bool TestPortrait() {
    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    if (!visionarm::ComputeCenteredLetterbox(
            720, 1280, 960, 960, &geometry, &transform)) {
        return false;
    }
    return geometry.resized_width == 540 &&
        geometry.resized_height == 960 &&
        geometry.pad_left == 210 && geometry.pad_right == 210 &&
        geometry.pad_top == 0 && geometry.pad_bottom == 0;
}

bool TestInvalid() {
    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    return !visionarm::ComputeCenteredLetterbox(
        0, 720, 960, 960, &geometry, &transform);
}

}  // namespace

int main() {
    const bool passed = Test1280x720To960() &&
        Test1920x1080To960() && TestPortrait() && TestInvalid();
    std::cout << (passed ? "letterbox_geometry_test PASSED\n"
                         : "letterbox_geometry_test FAILED\n");
    return passed ? 0 : 1;
}
