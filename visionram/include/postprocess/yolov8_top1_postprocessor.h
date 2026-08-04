#pragma once

#include "../common/pipeline_types.h"
#include "../common/tensor_view.h"

#include <vector>

namespace visionarm {

struct YoloV8Top1DecoderConfig {
    int model_width = 960;
    int model_height = 960;
    int class_count = 2;
    int target_class_id = 1;
    int dfl_bins = 16;
    float confidence_threshold = 0.25F;
    bool class_scores_are_logits = false;
};

struct YoloV8Top1PostprocessConfig {
    YoloV8Top1DecoderConfig decoder;
    float minimum_box_area_px = 4.0F;
};

class ITargetPostprocessor {
public:
    virtual ~ITargetPostprocessor() = default;

    [[nodiscard]] virtual PostprocessResult Process(
        const std::vector<TensorView>& outputs,
        const PreprocessTransform& transform) const = 0;
};

class YoloV8Top1Postprocessor final : public ITargetPostprocessor {
public:
    explicit YoloV8Top1Postprocessor(
        YoloV8Top1PostprocessConfig config);

    [[nodiscard]] PostprocessResult Process(
        const std::vector<TensorView>& outputs,
        const PreprocessTransform& transform) const override;

private:
    YoloV8Top1PostprocessConfig config_;
};

}  // namespace visionarm
