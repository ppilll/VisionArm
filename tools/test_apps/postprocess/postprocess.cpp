#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef VISIONARM_WITH_RKNN
#include "rknn_api.h"
#endif

namespace visionarm {

constexpr int kBranchCount = 3;
#ifdef VISIONARM_WITH_RKNN
constexpr int kOutputCount = 9;
#endif
constexpr int kBoxSides = 4;

// This postprocessor is intentionally specialized for one business target:
//   - class channel 1 is the target;
//   - class channel 0 is ignored as noise;
//   - only one final box is required;
//   - therefore NMS and all-candidate decoding are unnecessary.

enum class CoordinateSpace {
    MODEL_INPUT,
    ORIGINAL_FRAME,
};

enum class TargetState {
    NO_TARGET,
    DETECTED,
    INVALID,
};

struct Detection {
    float x1 = 0.0F;
    float y1 = 0.0F;
    float x2 = 0.0F;
    float y2 = 0.0F;
    float confidence = 0.0F;
    int class_id = -1;
    CoordinateSpace space = CoordinateSpace::MODEL_INPUT;
};

struct PreprocessTransform {
    int source_width = 0;
    int source_height = 0;
    int model_width = 0;
    int model_height = 0;

    // Forward transform:
    // model_x = source_x * scale_x + pad_left
    // model_y = source_y * scale_y + pad_top
    float scale_x = 0.0F;
    float scale_y = 0.0F;
    float uniform_scale = 0.0F;

    int pad_left = 0;
    int pad_top = 0;
    int pad_right = 0;
    int pad_bottom = 0;
    bool letterbox = true;
};

struct TargetObservation {
    TargetState state = TargetState::NO_TARGET;
    bool valid = false;

    float confidence = 0.0F;
    int class_id = -1;

    float x1 = 0.0F;
    float y1 = 0.0F;
    float x2 = 0.0F;
    float y2 = 0.0F;

    float center_x = 0.0F;
    float center_y = 0.0F;
    float width = 0.0F;
    float height = 0.0F;
    float area = 0.0F;

    int source_width = 0;
    int source_height = 0;
};

struct TargetError {
    TargetState state = TargetState::NO_TARGET;
    bool valid = false;

    float dx_px = 0.0F;
    float dy_px = 0.0F;
    float error_x_normalized = 0.0F;
    float error_y_normalized = 0.0F;
    float confidence = 0.0F;
};

struct DecoderConfig {
    int model_width = 960;
    int model_height = 960;
    int class_count = 2;
    int target_class_id = 1;
    int dfl_bins = 16;

    float confidence_threshold = 0.25F;

    // Rockchip's optimized YOLOv8 class outputs normally expose confidence
    // values. Set true only if model inspection proves these values are logits.
    bool class_scores_are_logits = false;
};

struct PostprocessConfig {
    DecoderConfig decoder;
    float minimum_box_area_px = 4.0F;
};

struct QuantizedTensorView {
    const int8_t* data = nullptr;
    int batch = 1;
    int channels = 0;
    int height = 0;
    int width = 0;
    int32_t zero_point = 0;
    float scale = 0.0F;
    std::size_t element_count = 0;
    int tensor_index = -1;
    std::string tensor_name;
};

struct YoloBranchView {
    QuantizedTensorView box;
    QuantizedTensorView classes;
    int stride = 0;
    int branch_index = -1;
};

struct BestGridLocation {
    int branch_index = -1;
    int grid_x = -1;
    int grid_y = -1;
    int stride = 0;
    float confidence = 0.0F;
};

struct PostprocessResult {
    std::optional<BestGridLocation> selected_grid;
    std::optional<Detection> model_detection;
    std::optional<Detection> original_detection;
    std::optional<TargetObservation> target;
    TargetError error;
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
        throw std::invalid_argument("tensor scale must be finite and positive");
    }

    return static_cast<float>(static_cast<int32_t>(value) - zero_point) * scale;
}

[[nodiscard]] int8_t QuantizeInt8ForTest(
    float value,
    int32_t zero_point,
    float scale) {

    const float quantized = value / scale + static_cast<float>(zero_point);
    const float clipped = std::clamp(quantized, -128.0F, 127.0F);
    return static_cast<int8_t>(std::lround(clipped));
}

[[nodiscard]] std::size_t RequiredElements(
    int batch,
    int channels,
    int height,
    int width) {

    if (batch <= 0 || channels <= 0 || height <= 0 || width <= 0) {
        throw std::invalid_argument("tensor dimensions must be positive");
    }

    return static_cast<std::size_t>(batch) *
           static_cast<std::size_t>(channels) *
           static_cast<std::size_t>(height) *
           static_cast<std::size_t>(width);
}

[[nodiscard]] std::size_t NchwIndex(
    int channel,
    int y,
    int x,
    int height,
    int width) {

    if (channel < 0 || y < 0 || x < 0 || height <= 0 || width <= 0 ||
        y >= height || x >= width) {
        throw std::out_of_range("invalid NCHW index");
    }

    return static_cast<std::size_t>(channel) *
               static_cast<std::size_t>(height) *
               static_cast<std::size_t>(width) +
           static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x);
}

[[nodiscard]] float DetectionArea(const Detection& detection) noexcept {
    return std::max(0.0F, detection.x2 - detection.x1) *
           std::max(0.0F, detection.y2 - detection.y1);
}

[[nodiscard]] PreprocessTransform MakeCenteredLetterboxTransform(
    int source_width,
    int source_height,
    int model_width,
    int model_height) {

    if (source_width <= 0 || source_height <= 0 ||
        model_width <= 0 || model_height <= 0) {
        throw std::invalid_argument("invalid image dimensions");
    }

    const float scale = std::min(
        static_cast<float>(model_width) / static_cast<float>(source_width),
        static_cast<float>(model_height) / static_cast<float>(source_height));

    const int resized_width = static_cast<int>(
        std::lround(static_cast<float>(source_width) * scale));
    const int resized_height = static_cast<int>(
        std::lround(static_cast<float>(source_height) * scale));

    const int remaining_width = model_width - resized_width;
    const int remaining_height = model_height - resized_height;
    const int pad_left = remaining_width / 2;
    const int pad_top = remaining_height / 2;

    return PreprocessTransform{
        source_width,
        source_height,
        model_width,
        model_height,
        scale,
        scale,
        scale,
        pad_left,
        pad_top,
        remaining_width - pad_left,
        remaining_height - pad_top,
        true,
    };
}

// -----------------------------------------------------------------------------
// Specialized YOLOv8 Top-1 decoder
// -----------------------------------------------------------------------------

class YoloV8TargetTop1Decoder final {
public:
    explicit YoloV8TargetTop1Decoder(DecoderConfig config)
        : config_(std::move(config)) {
        ValidateConfig();
    }

    [[nodiscard]] std::optional<BestGridLocation> FindBestTargetGrid(
        const std::array<YoloBranchView, kBranchCount>& branches) const {

        std::optional<BestGridLocation> global_best;

        for (const YoloBranchView& branch : branches) {
            ValidateBranch(branch);
            const std::optional<BestGridLocation> branch_best =
                FindBranchMaximum(branch);

            if (!branch_best.has_value()) {
                continue;
            }

            if (!global_best.has_value() ||
                IsBetterLocation(*branch_best, *global_best)) {
                global_best = branch_best;
            }
        }

        if (!global_best.has_value() ||
            global_best->confidence < config_.confidence_threshold) {
            return std::nullopt;
        }

        return global_best;
    }

    [[nodiscard]] Detection DecodeSelectedGrid(
        const std::array<YoloBranchView, kBranchCount>& branches,
        const BestGridLocation& selected) const {

        if (selected.branch_index < 0 ||
            selected.branch_index >= kBranchCount) {
            throw std::out_of_range("selected branch index is invalid");
        }

        const YoloBranchView& branch =
            branches[static_cast<std::size_t>(selected.branch_index)];
        ValidateBranch(branch);

        if (selected.grid_x < 0 || selected.grid_x >= branch.box.width ||
            selected.grid_y < 0 || selected.grid_y >= branch.box.height ||
            selected.stride != branch.stride) {
            throw std::invalid_argument("selected grid does not match branch");
        }

        const float left = DecodeDflDistance(
            branch.box, 0, selected.grid_y, selected.grid_x);
        const float top = DecodeDflDistance(
            branch.box, 1, selected.grid_y, selected.grid_x);
        const float right = DecodeDflDistance(
            branch.box, 2, selected.grid_y, selected.grid_x);
        const float bottom = DecodeDflDistance(
            branch.box, 3, selected.grid_y, selected.grid_x);

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
            throw std::runtime_error("selected DFL box is invalid");
        }

        return detection;
    }

private:
    void ValidateConfig() const {
        if (config_.model_width <= 0 || config_.model_height <= 0) {
            throw std::invalid_argument("model dimensions must be positive");
        }
        if (config_.class_count <= 0 || config_.target_class_id < 0 ||
            config_.target_class_id >= config_.class_count) {
            throw std::invalid_argument("target class configuration is invalid");
        }
        if (config_.dfl_bins <= 1) {
            throw std::invalid_argument("dfl_bins must be greater than one");
        }
        if (!IsFinite(config_.confidence_threshold) ||
            config_.confidence_threshold < 0.0F ||
            config_.confidence_threshold > 1.0F) {
            throw std::invalid_argument(
                "confidence_threshold must be in [0, 1]");
        }
    }

    void ValidateTensor(
        const QuantizedTensorView& tensor,
        int expected_channels,
        int expected_height,
        int expected_width,
        const char* role) const {

        if (tensor.data == nullptr) {
            throw std::invalid_argument(std::string(role) + " tensor is null");
        }
        if (tensor.batch != 1 || tensor.channels != expected_channels ||
            tensor.height != expected_height ||
            tensor.width != expected_width) {
            throw std::invalid_argument(std::string(role) +
                                        " tensor shape does not match contract");
        }
        if (!IsFinite(tensor.scale) || tensor.scale <= 0.0F) {
            throw std::invalid_argument(std::string(role) +
                                        " tensor scale is invalid");
        }
        const std::size_t required = RequiredElements(
            tensor.batch, tensor.channels, tensor.height, tensor.width);
        if (tensor.element_count < required) {
            throw std::invalid_argument(std::string(role) +
                                        " tensor buffer is too small");
        }
    }

    void ValidateBranch(const YoloBranchView& branch) const {
        if (branch.branch_index < 0 || branch.branch_index >= kBranchCount) {
            throw std::invalid_argument("invalid branch index");
        }
        if (branch.stride <= 0 ||
            config_.model_width % branch.stride != 0 ||
            config_.model_height % branch.stride != 0) {
            throw std::invalid_argument("invalid branch stride");
        }

        const int expected_width = config_.model_width / branch.stride;
        const int expected_height = config_.model_height / branch.stride;

        ValidateTensor(
            branch.box,
            kBoxSides * config_.dfl_bins,
            expected_height,
            expected_width,
            "box");
        ValidateTensor(
            branch.classes,
            config_.class_count,
            expected_height,
            expected_width,
            "class");
    }

    [[nodiscard]] float ConvertClassValue(
        int8_t raw_value,
        const QuantizedTensorView& tensor) const {

        float score = DequantizeInt8(
            raw_value, tensor.zero_point, tensor.scale);
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

        // Affine dequantization has positive scale, and sigmoid is monotonic.
        // Therefore the argmax can be found directly in INT8 space. Only the
        // branch maximum is converted to float.
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
        const int grid_y = static_cast<int>(best_offset /
                                             static_cast<std::size_t>(width));
        const int grid_x = static_cast<int>(best_offset %
                                             static_cast<std::size_t>(width));

        return BestGridLocation{
            branch.branch_index,
            grid_x,
            grid_y,
            branch.stride,
            score,
        };
    }

    [[nodiscard]] static bool IsBetterLocation(
        const BestGridLocation& candidate,
        const BestGridLocation& current) noexcept {

        constexpr float epsilon = 1.0e-7F;
        if (candidate.confidence > current.confidence + epsilon) {
            return true;
        }
        if (std::fabs(candidate.confidence - current.confidence) <= epsilon) {
            // Deterministic tie-break: prefer the finer feature map.
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

    [[nodiscard]] float DecodeDflDistance(
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
            const int8_t raw = box.data[NchwIndex(
                channel, y, x, box.height, box.width)];
            maximum_raw = std::max(maximum_raw, raw);
        }

        float denominator = 0.0F;
        float numerator = 0.0F;

        for (int bin = 0; bin < config_.dfl_bins; ++bin) {
            const int channel = side * config_.dfl_bins + bin;
            const int8_t raw = box.data[NchwIndex(
                channel, y, x, box.height, box.width)];

            // Dequantized logit difference:
            // ((q-zp) - (qmax-zp)) * scale == (q-qmax) * scale.
            // The zero point cancels, so no per-bin dequantization is needed.
            const float shifted_logit =
                static_cast<float>(
                    static_cast<int32_t>(raw) -
                    static_cast<int32_t>(maximum_raw)) *
                box.scale;
            const float weight = std::exp(shifted_logit);
            denominator += weight;
            numerator += weight * static_cast<float>(bin);
        }

        if (!IsFinite(denominator) || denominator <= 0.0F ||
            !IsFinite(numerator)) {
            throw std::runtime_error("DFL softmax produced an invalid value");
        }

        return numerator / denominator;
    }

    DecoderConfig config_;
};

// -----------------------------------------------------------------------------
// Coordinate mapping and target output
// -----------------------------------------------------------------------------

class CoordinateMapper final {
public:
    [[nodiscard]] static std::optional<Detection> ToOriginal(
        const Detection& input,
        const PreprocessTransform& transform,
        float minimum_area_px) {

        ValidateTransform(transform);

        if (input.space != CoordinateSpace::MODEL_INPUT ||
            !IsFinite(input) || !IsFinite(minimum_area_px) ||
            minimum_area_px < 0.0F) {
            return std::nullopt;
        }

        Detection output = input;

        if (transform.letterbox) {
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
            output.x1 = input.x1 / transform.scale_x;
            output.y1 = input.y1 / transform.scale_y;
            output.x2 = input.x2 / transform.scale_x;
            output.y2 = input.y2 / transform.scale_y;
        }

        const float maximum_x =
            static_cast<float>(transform.source_width - 1);
        const float maximum_y =
            static_cast<float>(transform.source_height - 1);

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

private:
    static void ValidateTransform(const PreprocessTransform& transform) {
        if (transform.source_width <= 0 || transform.source_height <= 0 ||
            transform.model_width <= 0 || transform.model_height <= 0) {
            throw std::invalid_argument("transform dimensions are invalid");
        }

        if (transform.letterbox) {
            if (!IsFinite(transform.uniform_scale) ||
                transform.uniform_scale <= 0.0F) {
                throw std::invalid_argument(
                    "letterbox uniform_scale is invalid");
            }
        } else if (!IsFinite(transform.scale_x) ||
                   !IsFinite(transform.scale_y) ||
                   transform.scale_x <= 0.0F ||
                   transform.scale_y <= 0.0F) {
            throw std::invalid_argument("stretch scales are invalid");
        }
    }
};

[[nodiscard]] TargetObservation MakeTargetObservation(
    const Detection& detection,
    int source_width,
    int source_height) {

    TargetObservation observation;

    if (detection.space != CoordinateSpace::ORIGINAL_FRAME ||
        !IsFinite(detection) || detection.x2 <= detection.x1 ||
        detection.y2 <= detection.y1 || source_width <= 0 ||
        source_height <= 0) {
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

[[nodiscard]] TargetError CalculateTargetError(
    const std::optional<TargetObservation>& target) {

    TargetError error;

    if (!target.has_value()) {
        error.state = TargetState::NO_TARGET;
        return error;
    }

    error.state = target->state;
    error.confidence = target->confidence;

    if (!target->valid || target->state != TargetState::DETECTED ||
        target->source_width <= 0 || target->source_height <= 0) {
        return error;
    }

    const float frame_center_x =
        static_cast<float>(target->source_width) * 0.5F;
    const float frame_center_y =
        static_cast<float>(target->source_height) * 0.5F;

    error.dx_px = target->center_x - frame_center_x;
    error.dy_px = target->center_y - frame_center_y;
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

class YoloV8TargetTop1Postprocessor final {
public:
    explicit YoloV8TargetTop1Postprocessor(PostprocessConfig config)
        : config_(std::move(config)), decoder_(config_.decoder) {

        if (!IsFinite(config_.minimum_box_area_px) ||
            config_.minimum_box_area_px < 0.0F) {
            throw std::invalid_argument(
                "minimum_box_area_px must be finite and non-negative");
        }
    }

    [[nodiscard]] PostprocessResult Process(
        const std::array<YoloBranchView, kBranchCount>& branches,
        const PreprocessTransform& transform) const {

        if (transform.model_width != config_.decoder.model_width ||
            transform.model_height != config_.decoder.model_height) {
            throw std::invalid_argument(
                "preprocess transform and decoder model sizes differ");
        }

        PostprocessResult result;
        result.selected_grid = decoder_.FindBestTargetGrid(branches);

        if (!result.selected_grid.has_value()) {
            result.error = CalculateTargetError(result.target);
            return result;
        }

        result.model_detection =
            decoder_.DecodeSelectedGrid(branches, *result.selected_grid);

        result.original_detection = CoordinateMapper::ToOriginal(
            *result.model_detection,
            transform,
            config_.minimum_box_area_px);

        if (!result.original_detection.has_value()) {
            TargetObservation invalid;
            invalid.state = TargetState::INVALID;
            invalid.confidence = result.model_detection->confidence;
            invalid.class_id = result.model_detection->class_id;
            result.target = invalid;
            result.error = CalculateTargetError(result.target);
            return result;
        }

        result.target = MakeTargetObservation(
            *result.original_detection,
            transform.source_width,
            transform.source_height);
        result.error = CalculateTargetError(result.target);
        return result;
    }

private:
    PostprocessConfig config_;
    YoloV8TargetTop1Decoder decoder_;
};

// -----------------------------------------------------------------------------
// Optional RKNN adapter
// -----------------------------------------------------------------------------

#ifdef VISIONARM_WITH_RKNN

[[nodiscard]] QuantizedTensorView MakeRknnTensorView(
    const rknn_output& output,
    const rknn_tensor_attr& attribute) {

    if (output.buf == nullptr) {
        throw std::invalid_argument("RKNN output buffer is null");
    }
    if (output.want_float != 0) {
        throw std::invalid_argument(
            "RKNN output must use want_float=0 for INT8 decoder");
    }
    if (attribute.n_dims != 4 || attribute.dims[0] != 1) {
        throw std::invalid_argument("RKNN output must be a 4-D batch-1 tensor");
    }
    if (attribute.fmt != RKNN_TENSOR_NCHW) {
        throw std::invalid_argument("RKNN output format must be NCHW");
    }
    if (attribute.type != RKNN_TENSOR_INT8) {
        throw std::invalid_argument("RKNN output type must be INT8");
    }
    if (attribute.qnt_type != RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC) {
        throw std::invalid_argument(
            "RKNN output quantization must be affine asymmetric");
    }
    if (output.size < attribute.n_elems * sizeof(int8_t)) {
        throw std::invalid_argument("RKNN output buffer size is too small");
    }

    return QuantizedTensorView{
        static_cast<const int8_t*>(output.buf),
        static_cast<int>(attribute.dims[0]),
        static_cast<int>(attribute.dims[1]),
        static_cast<int>(attribute.dims[2]),
        static_cast<int>(attribute.dims[3]),
        attribute.zp,
        attribute.scale,
        attribute.n_elems,
        static_cast<int>(attribute.index),
        attribute.name,
    };
}

[[nodiscard]] std::array<YoloBranchView, kBranchCount> BuildRknnBranches(
    const std::array<rknn_output, kOutputCount>& outputs,
    const std::array<rknn_tensor_attr, kOutputCount>& attributes,
    int model_width,
    int model_height) {

    std::array<YoloBranchView, kBranchCount> branches{};

    for (int branch_index = 0; branch_index < kBranchCount; ++branch_index) {
        const int base = branch_index * 3;

        // output[base + 2] is score_sum. It is deliberately ignored because
        // this specialized path scans only class[target_class_id].
        const QuantizedTensorView box = MakeRknnTensorView(
            outputs[static_cast<std::size_t>(base)],
            attributes[static_cast<std::size_t>(base)]);
        const QuantizedTensorView classes = MakeRknnTensorView(
            outputs[static_cast<std::size_t>(base + 1)],
            attributes[static_cast<std::size_t>(base + 1)]);

        if (box.height <= 0 || box.width <= 0 ||
            model_height % box.height != 0 ||
            model_width % box.width != 0) {
            throw std::invalid_argument(
                "cannot derive stride from RKNN output shape");
        }

        const int stride_y = model_height / box.height;
        const int stride_x = model_width / box.width;
        if (stride_x != stride_y) {
            throw std::invalid_argument("x/y strides differ");
        }

        branches[static_cast<std::size_t>(branch_index)] = YoloBranchView{
            box,
            classes,
            stride_x,
            branch_index,
        };
    }

    return branches;
}

[[nodiscard]] PostprocessResult PostprocessRknnOutputsTop1(
    const std::array<rknn_output, kOutputCount>& outputs,
    const std::array<rknn_tensor_attr, kOutputCount>& attributes,
    const PreprocessTransform& transform,
    const PostprocessConfig& config) {

    const auto branches = BuildRknnBranches(
        outputs,
        attributes,
        config.decoder.model_width,
        config.decoder.model_height);

    return YoloV8TargetTop1Postprocessor(config).Process(branches, transform);
}

#endif  // VISIONARM_WITH_RKNN

// -----------------------------------------------------------------------------
// Synthetic tests
// -----------------------------------------------------------------------------

struct OwnedQuantizedTensor {
    int channels = 0;
    int height = 0;
    int width = 0;
    int32_t zero_point = 0;
    float scale = 0.0F;
    int tensor_index = -1;
    std::string tensor_name;
    std::vector<int8_t> storage;

    OwnedQuantizedTensor(
        int channels_value,
        int height_value,
        int width_value,
        int32_t zero_point_value,
        float scale_value,
        int index_value,
        std::string name_value)
        : channels(channels_value),
          height(height_value),
          width(width_value),
          zero_point(zero_point_value),
          scale(scale_value),
          tensor_index(index_value),
          tensor_name(std::move(name_value)),
          storage(
              RequiredElements(1, channels_value, height_value, width_value),
              QuantizeInt8ForTest(
                  0.0F, zero_point_value, scale_value)) {}

    [[nodiscard]] QuantizedTensorView View() const {
        return QuantizedTensorView{
            storage.data(),
            1,
            channels,
            height,
            width,
            zero_point,
            scale,
            storage.size(),
            tensor_index,
            tensor_name,
        };
    }

    void Set(int channel, int y, int x, float value) {
        storage[NchwIndex(channel, y, x, height, width)] =
            QuantizeInt8ForTest(value, zero_point, scale);
    }
};

struct OwnedBranch {
    OwnedQuantizedTensor box;
    OwnedQuantizedTensor classes;
    int stride = 0;
    int branch_index = -1;

    OwnedBranch(
        int height,
        int width,
        int stride_value,
        int box_channels,
        int class_count,
        int base_index,
        int branch_index_value)
        : box(
              box_channels,
              height,
              width,
              0,
              0.125F,
              base_index,
              "box"),
          classes(
              class_count,
              height,
              width,
              -128,
              1.0F / 128.0F,
              base_index + 1,
              "class"),
          stride(stride_value),
          branch_index(branch_index_value) {}

    [[nodiscard]] YoloBranchView View() const {
        return YoloBranchView{
            box.View(), classes.View(), stride, branch_index};
    }
};

void SetDflPeak(
    OwnedQuantizedTensor& box,
    int side,
    int dfl_bins,
    int y,
    int x,
    int peak_bin) {

    for (int bin = 0; bin < dfl_bins; ++bin) {
        const float logit = bin == peak_bin ? 8.0F : -8.0F;
        box.Set(side * dfl_bins + bin, y, x, logit);
    }
}

[[nodiscard]] bool NearlyEqual(
    float lhs,
    float rhs,
    float tolerance) noexcept {
    return std::fabs(lhs - rhs) <= tolerance;
}

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error("self-test failed: " + message);
    }
}

void PrintDetection(const Detection& detection, const char* label) {
    std::cout << label
              << " class=" << detection.class_id
              << " confidence=" << std::fixed << std::setprecision(4)
              << detection.confidence
              << " xyxy=[" << detection.x1 << ", " << detection.y1
              << ", " << detection.x2 << ", " << detection.y2 << "]\n";
}

[[nodiscard]] PostprocessConfig MakeTestConfig() {
    PostprocessConfig config;
    config.decoder.model_width = 960;
    config.decoder.model_height = 960;
    config.decoder.class_count = 2;
    config.decoder.target_class_id = 1;
    config.decoder.dfl_bins = 16;
    config.decoder.confidence_threshold = 0.25F;
    config.decoder.class_scores_are_logits = false;
    config.minimum_box_area_px = 4.0F;
    return config;
}

int RunSyntheticSelfTest() {
    const PostprocessConfig config = MakeTestConfig();

    OwnedBranch branch0(120, 120, 8, 64, 2, 0, 0);
    OwnedBranch branch1(60, 60, 16, 64, 2, 3, 1);
    OwnedBranch branch2(30, 30, 32, 64, 2, 6, 2);

    // Noise class 0 is deliberately higher than all target scores. It must not
    // influence selection because only class channel 1 is inspected.
    branch0.classes.Set(0, 10, 10, 0.99F);
    branch1.classes.Set(0, 20, 20, 0.98F);
    branch2.classes.Set(0, 5, 5, 0.97F);

    // Target class 1 has candidates on P3 and P4. P4 is globally higher and
    // must be the only location decoded with DFL.
    branch0.classes.Set(1, 60, 60, 0.88F);
    branch1.classes.Set(1, 30, 30, 0.93F);
    branch2.classes.Set(1, 15, 15, 0.20F);

    SetDflPeak(branch0.box, 0, 16, 60, 60, 10);
    SetDflPeak(branch0.box, 1, 16, 60, 60, 8);
    SetDflPeak(branch0.box, 2, 16, 60, 60, 10);
    SetDflPeak(branch0.box, 3, 16, 60, 60, 8);

    SetDflPeak(branch1.box, 0, 16, 30, 30, 5);
    SetDflPeak(branch1.box, 1, 16, 30, 30, 4);
    SetDflPeak(branch1.box, 2, 16, 30, 30, 5);
    SetDflPeak(branch1.box, 3, 16, 30, 30, 4);

    const std::array<YoloBranchView, kBranchCount> branches{
        branch0.View(), branch1.View(), branch2.View()};

    const PreprocessTransform transform =
        MakeCenteredLetterboxTransform(1920, 1080, 960, 960);

    const YoloV8TargetTop1Postprocessor postprocessor(config);
    const PostprocessResult result =
        postprocessor.Process(branches, transform);

    Require(result.selected_grid.has_value(), "target grid missing");
    Require(result.selected_grid->branch_index == 1,
            "global Top-1 should come from P4");
    Require(result.selected_grid->grid_x == 30 &&
                result.selected_grid->grid_y == 30,
            "wrong P4 grid selected");
    Require(result.model_detection.has_value(), "model box missing");
    Require(result.original_detection.has_value(), "original box missing");
    Require(result.target.has_value(), "target observation missing");
    Require(result.target->valid, "target observation invalid");
    Require(result.target->class_id == 1,
            "selected class must always be target class 1");
    Require(NearlyEqual(result.target->confidence, 0.93F, 0.02F),
            "target confidence mismatch");
    Require(result.error.valid, "target error invalid");

    std::cout << "selected branch=P" << result.selected_grid->branch_index + 3
              << " stride=" << result.selected_grid->stride
              << " grid=[" << result.selected_grid->grid_x << ", "
              << result.selected_grid->grid_y << "]\n";
    PrintDetection(*result.model_detection, "model");
    PrintDetection(*result.original_detection, "original");

    // No-target test: class 0 can be arbitrarily high, but class 1 remains
    // below threshold. The decoder must return no box.
    OwnedBranch no_target0(120, 120, 8, 64, 2, 0, 0);
    OwnedBranch no_target1(60, 60, 16, 64, 2, 3, 1);
    OwnedBranch no_target2(30, 30, 32, 64, 2, 6, 2);
    no_target0.classes.Set(0, 40, 40, 0.99F);
    no_target1.classes.Set(0, 20, 20, 0.99F);
    no_target2.classes.Set(0, 10, 10, 0.99F);
    no_target0.classes.Set(1, 40, 40, 0.10F);
    no_target1.classes.Set(1, 20, 20, 0.15F);
    no_target2.classes.Set(1, 10, 10, 0.20F);

    const std::array<YoloBranchView, kBranchCount> no_target_branches{
        no_target0.View(), no_target1.View(), no_target2.View()};
    const PostprocessResult no_target_result =
        postprocessor.Process(no_target_branches, transform);

    Require(!no_target_result.selected_grid.has_value(),
            "low target scores must produce NO_TARGET");
    Require(!no_target_result.model_detection.has_value(),
            "NO_TARGET must not decode DFL");
    Require(!no_target_result.target.has_value(),
            "NO_TARGET must not create an observation");
    Require(no_target_result.error.state == TargetState::NO_TARGET,
            "NO_TARGET state mismatch");

    std::cout << "Top-1 target-only postprocess self-test: PASS\n";
    return EXIT_SUCCESS;
}

}  // namespace visionarm

int main(int argc, char** argv) {
    try {
        if (argc > 1 && std::string(argv[1]) == "--help") {
            std::cout
                << "VisionArm YOLOv8 RKNN target-only Top-1 postprocess test\n"
                << "Usage: " << argv[0] << " [--self-test]\n"
                << "Default action: run synthetic INT8 tensor self-tests.\n";
            return EXIT_SUCCESS;
        }

        return visionarm::RunSyntheticSelfTest();
    } catch (const std::exception& exception) {
        std::cerr << "fatal: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
}