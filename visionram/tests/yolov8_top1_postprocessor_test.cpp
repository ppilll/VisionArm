#include "postprocess/yolov8_top1_postprocessor.h"
#include "preprocess/letterbox_geometry.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

struct OwnedTensor {
    std::vector<int8_t> storage;
    visionarm::TensorView view;
};

OwnedTensor MakeTensor(
    uint32_t index,
    int channels,
    int height,
    int width,
    int8_t initial_value,
    float scale) {

    const std::size_t elements =
        static_cast<std::size_t>(channels) *
        static_cast<std::size_t>(height) *
        static_cast<std::size_t>(width);

    OwnedTensor tensor;
    tensor.storage.assign(elements, initial_value);
    tensor.view.index = index;
    tensor.view.data = tensor.storage.data();
    tensor.view.bytes = tensor.storage.size();
    tensor.view.dims = {
        1U,
        static_cast<uint32_t>(channels),
        static_cast<uint32_t>(height),
        static_cast<uint32_t>(width),
    };
    tensor.view.data_type = visionarm::TensorDataType::INT8;
    tensor.view.layout = visionarm::TensorLayout::NCHW;
    tensor.view.quantization =
        visionarm::TensorQuantization::AFFINE_ASYMMETRIC;
    tensor.view.zero_point = 0;
    tensor.view.scale = scale;
    return tensor;
}

std::size_t PlaneOffset(int y, int x, int width) {
    return static_cast<std::size_t>(y) *
               static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x);
}

bool NearlyEqual(float first, float second, float tolerance = 1.0e-3F) {
    return std::fabs(first - second) <= tolerance;
}

bool TestSingleClassRectangularTop1() {
    OwnedTensor box0 = MakeTensor(0U, 64, 68, 120, 0, 0.1F);
    OwnedTensor cls0 = MakeTensor(1U, 1, 68, 120, -128, 0.01F);
    OwnedTensor box1 = MakeTensor(3U, 64, 34, 60, 0, 0.1F);
    OwnedTensor cls1 = MakeTensor(4U, 1, 34, 60, -128, 0.01F);
    OwnedTensor box2 = MakeTensor(6U, 64, 17, 30, 0, 0.1F);
    OwnedTensor cls2 = MakeTensor(7U, 1, 17, 30, -128, 0.01F);

    cls0.storage[PlaneOffset(20, 10, 120)] = 80;
    cls1.storage[PlaneOffset(8, 5, 60)] = 70;
    cls2.storage[PlaneOffset(4, 3, 30)] = 60;

    // Refresh data pointers after all vector writes.
    cls0.view.data = cls0.storage.data();
    cls1.view.data = cls1.storage.data();
    cls2.view.data = cls2.storage.data();

    const std::vector<visionarm::TensorView> outputs{
        cls2.view,
        box0.view,
        cls0.view,
        box2.view,
        box1.view,
        cls1.view,
    };

    visionarm::YoloV8Top1PostprocessConfig config;
    config.decoder.model_width = 960;
    config.decoder.model_height = 544;
    config.decoder.class_count = 1;
    config.decoder.target_class_id = 0;
    config.decoder.dfl_bins = 16;
    config.decoder.confidence_threshold = 0.25F;

    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    if (!visionarm::ComputeModelResizeGeometry(
            1920,
            1080,
            960,
            544,
            visionarm::ResizeGeometryPolicy{},
            &geometry,
            &transform)) {
        return false;
    }

    const visionarm::YoloV8Top1Postprocessor postprocessor(config);
    const visionarm::PostprocessResult result =
        postprocessor.Process(outputs, transform);

    if (!result.selected_grid.has_value() ||
        !result.model_detection.has_value() ||
        !result.original_detection.has_value() ||
        !result.target.has_value()) {
        return false;
    }

    const visionarm::BestGridLocation& selected = *result.selected_grid;
    const visionarm::Detection& model = *result.model_detection;
    const visionarm::Detection& original = *result.original_detection;

    return selected.branch_index == 0 &&
        selected.grid_x == 10 && selected.grid_y == 20 &&
        selected.stride == 8 &&
        NearlyEqual(selected.confidence, 0.8F) &&
        model.class_id == 0 &&
        NearlyEqual(model.x1, 24.0F) &&
        NearlyEqual(model.y1, 104.0F) &&
        NearlyEqual(model.x2, 144.0F) &&
        NearlyEqual(model.y2, 224.0F) &&
        original.class_id == 0 &&
        original.space == visionarm::CoordinateSpace::ORIGINAL_FRAME &&
        NearlyEqual(original.x1, 48.0F) &&
        NearlyEqual(original.y1, 204.0F) &&
        NearlyEqual(original.x2, 288.0F) &&
        NearlyEqual(original.y2, 444.0F) &&
        transform.letterbox &&
        transform.pad_top == 2 && transform.pad_bottom == 2 &&
        NearlyEqual(transform.uniform_scale, 0.5F) &&
        result.target->valid &&
        result.target->state == visionarm::TargetState::DETECTED &&
        result.target->class_id == 0 &&
        result.error.valid;
}

bool TestThresholdRejectsTop1() {
    OwnedTensor box0 = MakeTensor(0U, 64, 68, 120, 0, 0.1F);
    OwnedTensor cls0 = MakeTensor(1U, 1, 68, 120, -128, 0.01F);
    OwnedTensor box1 = MakeTensor(3U, 64, 34, 60, 0, 0.1F);
    OwnedTensor cls1 = MakeTensor(4U, 1, 34, 60, -128, 0.01F);
    OwnedTensor box2 = MakeTensor(6U, 64, 17, 30, 0, 0.1F);
    OwnedTensor cls2 = MakeTensor(7U, 1, 17, 30, -128, 0.01F);

    cls0.storage[0] = 50;
    cls0.view.data = cls0.storage.data();

    const std::vector<visionarm::TensorView> outputs{
        box0.view, cls0.view, box1.view, cls1.view, box2.view, cls2.view};

    visionarm::YoloV8Top1PostprocessConfig config;
    config.decoder.confidence_threshold = 0.75F;
    const visionarm::YoloV8Top1Postprocessor postprocessor(config);

    visionarm::LetterboxGeometry geometry;
    visionarm::PreprocessTransform transform;
    if (!visionarm::ComputeModelResizeGeometry(
            1280,
            720,
            960,
            544,
            visionarm::ResizeGeometryPolicy{},
            &geometry,
            &transform)) {
        return false;
    }

    const visionarm::PostprocessResult result =
        postprocessor.Process(outputs, transform);
    return !result.selected_grid.has_value() &&
        !result.model_detection.has_value() &&
        !result.original_detection.has_value() &&
        !result.target.has_value() &&
        !result.error.valid;
}

}  // namespace

int main() {
    const bool passed =
        TestSingleClassRectangularTop1() &&
        TestThresholdRejectsTop1();

    std::cout << (passed ? "yolov8_top1_postprocessor_test PASSED\n"
                         : "yolov8_top1_postprocessor_test FAILED\n");
    return passed ? 0 : 1;
}
