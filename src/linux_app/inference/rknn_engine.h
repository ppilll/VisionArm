#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rknn_api.h"

struct InputTensor {
    uint32_t index = 0;
    void* data = nullptr;
    uint32_t size = 0;
    rknn_tensor_type type = RKNN_TENSOR_UINT8;
    rknn_tensor_format format = RKNN_TENSOR_NHWC;
};

struct OutputTensor {
    uint32_t index = 0;
    std::string name;
    std::vector<uint32_t> dims;
    rknn_tensor_type type{};
    rknn_tensor_format format{};
    rknn_tensor_qnt_type qnt_type{};
    int32_t zero_point = 0;
    float scale = 0.0f;
    std::vector<uint8_t> data;
};

struct ModelInfo {
    std::string api_version;
    std::string driver_version;

    std::vector<rknn_tensor_attr> inputs;
    std::vector<rknn_tensor_attr> outputs;
};

class RknnEngine {
public:
    RknnEngine() = default;
    ~RknnEngine();

    RknnEngine(const RknnEngine&) = delete;
    RknnEngine& operator=(const RknnEngine&) = delete;

    bool Init(const std::string& model_path);

    bool Infer(
        const InputTensor& input,
        std::vector<OutputTensor>* outputs);

    const ModelInfo& GetModelInfo() const {
        return model_info_;
    }

    void Shutdown();

private:
    bool QueryModel();
    static bool CheckRet(const char* api, int ret);

    rknn_context ctx_ = 0;
    bool initialized_ = false;
    ModelInfo model_info_;
};
