#include "postprocess/yolov8_top1_postprocessor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace visionarm {
namespace {

constexpr int kBranchCount = 3;
constexpr int kBoxSides = 4;
constexpr std::array<uint32_t, kBranchCount> kBoxIndices{0U, 3U, 6U};
constexpr std::array<uint32_t, kBranchCount> kClassIndices{1U, 4U, 7U};

struct QuantizedTensorView {
    const int8_t* data = nullptr;
    int batch = 1;
    int channels = 0;
    int height = 0;
    int width = 0;
    int32_t zero_point = 0;
    float scale = 0.0F;
    std::size_t element_count = 0;
    uint32_t tensor_index = 0;
};

struct YoloBranchView {
    QuantizedTensorView box;
    QuantizedTensorView classes;
    int stride = 0;
    int branch_index = -1;
};

[[nodiscard]] bool IsFinite(float value) noexcept {
    return std::isfinite(value);
}

[[nodiscard]] bool IsFinite(const Detection& detection) noexcept {
    return IsFinite(detection.x1) && IsFinite(detection.y1) &&
           IsFinite(detection.x2) && IsFinite(detection.y2) &&
           IsFinite(detection.confidence);
}

[[nodiscard]] float Sigmoid(float value) noexcept {
    if (value >= 0.0F) {
        const float exponential = std::exp(-value);
        return 1.0F / (1.0F + exponential);
    }
    const float exponential = std::exp(value);
    return exponential / (1.0F + exponential);
}

[[nodiscard]] float DequantizeInt8(
    int8_t value,
    int32_t zero_point,
    float scale) {

    if (!IsFinite(scale) || scale <= 0.0F) {
        throw std::invalid_argument("tensor scale must be positive");
    }
    return static_cast<float>(
        static_cast<int32_t>(value) - zero_point) * scale;
}

[[nodiscard]] std::size_t NchwIndex(
    int channel,
    int y,
    int x,
    int height,
    int width) {

    if (channel < 0 || y < 0 || x < 0 ||
        height <= 0 || width <= 0 || y >= height || x >= width) {
        throw std::out_of_range("invalid NCHW index");
    }
    return static_cast<std::size_t>(channel) *
               static_cast<std::size_t>(height) *
               static_cast<std::size_t>(width) +
           static_cast<std::size_t>(y) *
               static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x);
}

[[nodiscard]] const TensorView& FindTensor(
    const std::vector<TensorView>& outputs,
    uint32_t index) {

    const auto iterator = std::find_if(
        outputs.begin(),
        outputs.end(),
        [index](const TensorView& tensor) {
            return tensor.index == index;
        });
    if (iterator == outputs.end()) {
        throw std::invalid_argument(
            "required RKNN output tensor is missing: " +
            std::to_string(index));
    }
    return *iterator;
}

[[nodiscard]] QuantizedTensorView ConvertTensor(
    const TensorView& tensor) {

    if (tensor.data == nullptr || tensor.dims.size() != 4U ||
        tensor.dims[0] != 1U ||
        tensor.data_type != TensorDataType::INT8 ||
        tensor.layout != TensorLayout::NCHW ||
        tensor.quantization != TensorQuantization::AFFINE_ASYMMETRIC ||
        !IsFinite(tensor.scale) || tensor.scale <= 0.0F) {
        throw std::invalid_argument(
            "RKNN output tensor does not match INT8 NCHW affine contract");
    }

    const std::size_t elements =
        static_cast<std::size_t>(tensor.dims[0]) *
        static_cast<std::size_t>(tensor.dims[1]) *
        static_cast<std::size_t>(tensor.dims[2]) *
        static_cast<std::size_t>(tensor.dims[3]);
    if (tensor.bytes < elements) {
        throw std::invalid_argument("RKNN output tensor buffer is too small");
    }

    return QuantizedTensorView{
        static_cast<const int8_t*>(tensor.data),
        static_cast<int>(tensor.dims[0]),
        static_cast<int>(tensor.dims[1]),
        static_cast<int>(tensor.dims[2]),
        static_cast<int>(tensor.dims[3]),
        tensor.zero_point,
        tensor.scale,
        elements,
        tensor.index,
    };
}

[[nodiscard]] std::array<YoloBranchView, kBranchCount> BuildBranches(
    const std::vector<TensorView>& outputs,
    int model_width,
    int model_height) {

    std::array<YoloBranchView, kBranchCount> branches{};
    for (int branch_index = 0; branch_index < kBranchCount; ++branch_index) {
        const std::size_t branch_offset =
            static_cast<std::size_t>(branch_index);
        const QuantizedTensorView box = ConvertTensor(
            FindTensor(outputs, kBoxIndices[branch_offset]));
        const QuantizedTensorView classes = ConvertTensor(
            FindTensor(outputs, kClassIndices[branch_offset]));

        if (box.height <= 0 || box.width <= 0 ||
            model_height % box.height != 0 ||
            model_width % box.width != 0) {
            throw std::invalid_argument("cannot derive YOLO branch stride");
        }

        const int stride_x = model_width / box.width;
        const int stride_y = model_height / box.height;
        if (stride_x != stride_y) {
            throw std::invalid_argument("YOLO x/y strides differ");
        }

        branches[branch_offset] =
            YoloBranchView{box, classes, stride_x, branch_index};
    }
    return branches;
}

class Top1Decoder final {
public:
    explicit Top1Decoder(YoloV8Top1DecoderConfig config)
        : config_(std::move(config)) {
        ValidateConfig();
    }

    [[nodiscard]] std::optional<BestGridLocation> FindBest(
        const std::array<YoloBranchView, kBranchCount>& branches) const {

        std::optional<BestGridLocation> global_best;
        for (const YoloBranchView& branch : branches) {
            ValidateBranch(branch);
            const auto branch_best = FindBranchMaximum(branch);
            if (!branch_best.has_value()) {
                continue;
            }
            if (!global_best.has_value() ||
                IsBetter(*branch_best, *global_best)) {
                global_best = branch_best;
            }
        }

        if (!global_best.has_value() ||
            global_best->confidence < config_.confidence_threshold) {
            return std::nullopt;
        }
        return global_best;
    }

    [[nodiscard]] Detection Decode(
        const std::array<YoloBranchView, kBranchCount>& branches,
        const BestGridLocation& selected) const {

        if (selected.branch_index < 0 ||
            selected.branch_index >= kBranchCount) {
            throw std::out_of_range("selected YOLO branch is invalid");
        }

        const YoloBranchView& branch = branches[
            static_cast<std::size_t>(selected.branch_index)];
        ValidateBranch(branch);
        if (selected.grid_x < 0 || selected.grid_x >= branch.box.width ||
            selected.grid_y < 0 || selected.grid_y >= branch.box.height ||
            selected.stride != branch.stride) {
            throw std::invalid_argument(
                "selected grid does not match YOLO branch");
        }

        const float left = DecodeDfl(branch.box, 0, selected.grid_y, selected.grid_x);
        const float top = DecodeDfl(branch.box, 1, selected.grid_y, selected.grid_x);
        const float right = DecodeDfl(branch.box, 2, selected.grid_y, selected.grid_x);
        const float bottom = DecodeDfl(branch.box, 3, selected.grid_y, selected.grid_x);

        const float center_x = static_cast<float>(selected.grid_x) + 0.5F;
        const float center_y = static_cast<float>(selected.grid_y) + 0.5F;
        const float stride = static_cast<float>(selected.stride);

        Detection detection;
        detection.x1 = (center_x - left) * stride;
        detection.y1 = (center_y - top) * stride;
        detection.x2 = (center_x + right) * stride;
        detection.y2 = (center_y + bottom) * stride;
        detection.confidence = selected.confidence;
        detection.class_id = config_.target_class_id;
        detection.space = CoordinateSpace::MODEL_INPUT;

        if (!IsFinite(detection) || detection.x2 <= detection.x1 ||
            detection.y2 <= detection.y1) {
            throw std::runtime_error("decoded YOLO box is invalid");
        }
        return detection;
    }

private:
    void ValidateConfig() const {
        if (config_.model_width <= 0 || config_.model_height <= 0 ||
            config_.class_count <= 0 || config_.target_class_id < 0 ||
            config_.target_class_id >= config_.class_count ||
            config_.dfl_bins <= 1 ||
            !IsFinite(config_.confidence_threshold) ||
            config_.confidence_threshold < 0.0F ||
            config_.confidence_threshold > 1.0F) {
            throw std::invalid_argument("invalid YOLOv8 Top-1 decoder config");
        }
    }

    void ValidateBranch(const YoloBranchView& branch) const {
        if (branch.branch_index < 0 || branch.branch_index >= kBranchCount ||
            branch.stride <= 0 ||
            config_.model_width % branch.stride != 0 ||
            config_.model_height % branch.stride != 0) {
            throw std::invalid_argument("invalid YOLO branch stride");
        }

        const int expected_width = config_.model_width / branch.stride;
        const int expected_height = config_.model_height / branch.stride;

        if (
            branch.box.batch != 1 || branch.classes.batch != 1 ||
            branch.box.channels != kBoxSides * config_.dfl_bins ||
            branch.classes.channels != config_.class_count ||
            branch.box.width != expected_width ||
            branch.box.height != expected_height ||
            branch.classes.width != expected_width ||
            branch.classes.height != expected_height) {
            throw std::invalid_argument("YOLO branch shape does not match contract");
        }
    }

    [[nodiscard]] float ConvertClassValue(
        int8_t raw,
        const QuantizedTensorView& tensor) const {
        float score = DequantizeInt8(raw, tensor.zero_point, tensor.scale);
        if (config_.class_scores_are_logits) {
            score = Sigmoid(score);
        }
        return score;
    }

    [[nodiscard]] std::optional<BestGridLocation> FindBranchMaximum(
        const YoloBranchView& branch) const {

        const std::size_t plane_size =
            static_cast<std::size_t>(branch.classes.height) *
            static_cast<std::size_t>(branch.classes.width);
        const std::size_t plane_offset =
            static_cast<std::size_t>(config_.target_class_id) * plane_size;
        const int8_t* target_plane = branch.classes.data + plane_offset;

        int8_t best_raw = std::numeric_limits<int8_t>::min();
        std::size_t best_offset = 0;
        for (std::size_t offset = 0; offset < plane_size; ++offset) {
            if (target_plane[offset] > best_raw) {
                best_raw = target_plane[offset];
                best_offset = offset;
            }
        }

        const float score = ConvertClassValue(best_raw, branch.classes);
        if (!IsFinite(score)) {
            return std::nullopt;
        }

        const int width = branch.classes.width;
        return BestGridLocation{
            branch.branch_index,
            static_cast<int>(best_offset % static_cast<std::size_t>(width)),
            static_cast<int>(best_offset / static_cast<std::size_t>(width)),
            branch.stride,
            score,
        };
    }

    [[nodiscard]] static bool IsBetter(
        const BestGridLocation& candidate,
        const BestGridLocation& current) noexcept {

        constexpr float epsilon = 1.0e-7F;
        if (candidate.confidence > current.confidence + epsilon) {
            return true;
        }
        if (std::fabs(candidate.confidence - current.confidence) <= epsilon) {
            if (candidate.stride != current.stride) {
                return candidate.stride < current.stride;
            }
            if (candidate.grid_y != current.grid_y) {
                return candidate.grid_y < current.grid_y;
            }
            return candidate.grid_x < current.grid_x;
        }
        return false;
    }

    [[nodiscard]] float DecodeDfl(
        const QuantizedTensorView& box,
        int side,
        int y,
        int x) const {

        if (side < 0 || side >= kBoxSides) {
            throw std::out_of_range("DFL side is invalid");
        }

        int8_t maximum_raw = std::numeric_limits<int8_t>::min();
        for (int bin = 0; bin < config_.dfl_bins; ++bin) {
            const int channel = side * config_.dfl_bins + bin;
            maximum_raw = std::max(
                maximum_raw,
                box.data[NchwIndex(channel, y, x, box.height, box.width)]);
        }

        float denominator = 0.0F;
        float numerator = 0.0F;
        for (int bin = 0; bin < config_.dfl_bins; ++bin) {
            const int channel = side * config_.dfl_bins + bin;
            const int8_t raw =
                box.data[NchwIndex(channel, y, x, box.height, box.width)];
            const float shifted_logit =
                static_cast<float>(
                    static_cast<int32_t>(raw) -
                    static_cast<int32_t>(maximum_raw)) * box.scale;
            const float weight = std::exp(shifted_logit);
            denominator += weight;
            numerator += weight * static_cast<float>(bin);
        }

        if (!IsFinite(denominator) || denominator <= 0.0F ||
            !IsFinite(numerator)) {
            throw std::runtime_error("DFL softmax produced invalid value");
        }
        return numerator / denominator;
    }

    YoloV8Top1DecoderConfig config_;
};

[[nodiscard]] float DetectionArea(const Detection& detection) noexcept {
    return std::max(0.0F, detection.x2 - detection.x1) *
           std::max(0.0F, detection.y2 - detection.y1);
}

[[nodiscard]] std::optional<Detection> MapToOriginal(
    const Detection& input,
    const PreprocessTransform& transform,
    float minimum_area_px) {

    if (transform.source_width <= 0 || transform.source_height <= 0 ||
        transform.model_width <= 0 || transform.model_height <= 0 ||
        input.space != CoordinateSpace::MODEL_INPUT || !IsFinite(input)) {
        return std::nullopt;
    }

    Detection output = input;
    if (transform.letterbox) {
        if (!IsFinite(transform.uniform_scale) ||
            transform.uniform_scale <= 0.0F) {
            throw std::invalid_argument("invalid letterbox transform");
        }
        output.x1 =
            (input.x1 - static_cast<float>(transform.pad_left)) /
            transform.uniform_scale;
        output.y1 =
            (input.y1 - static_cast<float>(transform.pad_top)) /
            transform.uniform_scale;
        output.x2 =
            (input.x2 - static_cast<float>(transform.pad_left)) /
            transform.uniform_scale;
        output.y2 =
            (input.y2 - static_cast<float>(transform.pad_top)) /
            transform.uniform_scale;
    } else {
        if (!IsFinite(transform.scale_x) || !IsFinite(transform.scale_y) ||
            transform.scale_x <= 0.0F || transform.scale_y <= 0.0F) {
            throw std::invalid_argument("invalid stretch transform");
        }
        output.x1 = input.x1 / transform.scale_x;
        output.y1 = input.y1 / transform.scale_y;
        output.x2 = input.x2 / transform.scale_x;
        output.y2 = input.y2 / transform.scale_y;
    }

    const float maximum_x = static_cast<float>(transform.source_width - 1);
    const float maximum_y = static_cast<float>(transform.source_height - 1);
    output.x1 = std::clamp(output.x1, 0.0F, maximum_x);
    output.y1 = std::clamp(output.y1, 0.0F, maximum_y);
    output.x2 = std::clamp(output.x2, 0.0F, maximum_x);
    output.y2 = std::clamp(output.y2, 0.0F, maximum_y);
    output.space = CoordinateSpace::ORIGINAL_FRAME;

    if (!IsFinite(output) || output.x2 <= output.x1 ||
        output.y2 <= output.y1 ||
        DetectionArea(output) < minimum_area_px) {
        return std::nullopt;
    }
    return output;
}

[[nodiscard]] TargetObservation MakeObservation(
    const Detection& detection,
    int source_width,
    int source_height) {

    TargetObservation observation;
    if (detection.space != CoordinateSpace::ORIGINAL_FRAME ||
        !IsFinite(detection) || detection.x2 <= detection.x1 ||
        detection.y2 <= detection.y1 ||
        source_width <= 0 || source_height <= 0) {
        observation.state = TargetState::INVALID;
        return observation;
    }

    observation.state = TargetState::DETECTED;
    observation.valid = true;
    observation.confidence = detection.confidence;
    observation.class_id = detection.class_id;
    observation.x1 = detection.x1;
    observation.y1 = detection.y1;
    observation.x2 = detection.x2;
    observation.y2 = detection.y2;
    observation.width = detection.x2 - detection.x1;
    observation.height = detection.y2 - detection.y1;
    observation.area = observation.width * observation.height;
    observation.center_x = 0.5F * (detection.x1 + detection.x2);
    observation.center_y = 0.5F * (detection.y1 + detection.y2);
    observation.source_width = source_width;
    observation.source_height = source_height;
    return observation;
}

[[nodiscard]] TargetError CalculateError(
    const std::optional<TargetObservation>& target) {

    TargetError error;
    if (!target.has_value()) {
        return error;
    }

    error.state = target->state;
    error.confidence = target->confidence;
    if (!target->valid || target->state != TargetState::DETECTED ||
        target->source_width <= 0 || target->source_height <= 0) {
        return error;
    }

    const float center_x = static_cast<float>(target->source_width) * 0.5F;
    const float center_y = static_cast<float>(target->source_height) * 0.5F;
    error.dx_px = target->center_x - center_x;
    error.dy_px = target->center_y - center_y;
    error.error_x_normalized = std::clamp(
        2.0F * error.dx_px / static_cast<float>(target->source_width),
        -1.0F,
        1.0F);
    error.error_y_normalized = std::clamp(
        2.0F * error.dy_px / static_cast<float>(target->source_height),
        -1.0F,
        1.0F);
    error.valid = true;
    return error;
}

}  // namespace

YoloV8Top1Postprocessor::YoloV8Top1Postprocessor(
    YoloV8Top1PostprocessConfig config)
    : config_(std::move(config)) {

    if (!IsFinite(config_.minimum_box_area_px) ||
        config_.minimum_box_area_px < 0.0F) {
        throw std::invalid_argument("invalid minimum box area");
    }
    (void)Top1Decoder(config_.decoder);
}

PostprocessResult YoloV8Top1Postprocessor::Process(
    const std::vector<TensorView>& outputs,
    const PreprocessTransform& transform) const {

    if (transform.model_width != config_.decoder.model_width ||
        transform.model_height != config_.decoder.model_height) {
        throw std::invalid_argument(
            "preprocess and postprocess model dimensions differ");
    }

    const auto branches = BuildBranches(
        outputs,
        config_.decoder.model_width,
        config_.decoder.model_height);
    const Top1Decoder decoder(config_.decoder);

    PostprocessResult result;
    result.selected_grid = decoder.FindBest(branches);
    if (!result.selected_grid.has_value()) {
        result.error = CalculateError(result.target);
        return result;
    }

    result.model_detection = decoder.Decode(
        branches,
        *result.selected_grid);
    result.original_detection = MapToOriginal(
        *result.model_detection,
        transform,
        config_.minimum_box_area_px);

    if (!result.original_detection.has_value()) {
        TargetObservation invalid;
        invalid.state = TargetState::INVALID;
        invalid.confidence = result.model_detection->confidence;
        invalid.class_id = result.model_detection->class_id;
        result.target = invalid;
        result.error = CalculateError(result.target);
        return result;
    }

    result.target = MakeObservation(
        *result.original_detection,
        transform.source_width,
        transform.source_height);
    result.error = CalculateError(result.target);
    return result;
}

}  // namespace visionarm
