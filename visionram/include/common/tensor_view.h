#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace visionarm {

enum class TensorDataType {
    UNKNOWN,
    INT8,
    UINT8,
    FLOAT16,
    FLOAT32,
};

enum class TensorLayout {
    UNKNOWN,
    NCHW,
    NHWC,
};

enum class TensorQuantization {
    NONE,
    AFFINE_ASYMMETRIC,
    OTHER,
};

struct TensorView {
    uint32_t index = 0;
    std::string name;

    const void* data = nullptr;
    std::size_t bytes = 0;
    std::vector<uint32_t> dims;

    TensorDataType data_type = TensorDataType::UNKNOWN;
    TensorLayout layout = TensorLayout::UNKNOWN;
    TensorQuantization quantization = TensorQuantization::NONE;

    int32_t zero_point = 0;
    float scale = 0.0F;
};

}  // namespace visionarm
