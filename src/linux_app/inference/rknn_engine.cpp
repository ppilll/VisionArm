#include "rknn_engine.h"

#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <utility>

namespace {

std::vector<uint8_t> ReadFile(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary | std::ios::ate);
    if (!ifs) {
        return {};
    }

    const std::streamoff end = ifs.tellg();
    if (end <= 0 ||
        static_cast<uint64_t>(end) >
            static_cast<uint64_t>(std::numeric_limits<uint32_t>::max())) {
        return {};
    }

    std::vector<uint8_t> data(static_cast<size_t>(end));
    ifs.seekg(0, std::ios::beg);

    if (!ifs.read(
            reinterpret_cast<char*>(data.data()),
            static_cast<std::streamsize>(data.size()))) {
        return {};
    }

    return data;
}

uint32_t TensorTypeBytes(rknn_tensor_type type) {
    switch (type) {
        case RKNN_TENSOR_FLOAT32:
        case RKNN_TENSOR_INT32:
        case RKNN_TENSOR_UINT32:
            return 4;
        case RKNN_TENSOR_FLOAT16:
        case RKNN_TENSOR_INT16:
        case RKNN_TENSOR_UINT16:
            return 2;
        case RKNN_TENSOR_INT64:
            return 8;
        case RKNN_TENSOR_INT8:
        case RKNN_TENSOR_UINT8:
        case RKNN_TENSOR_BOOL:
            return 1;
        default:
            return 0;
    }
}

void DumpTensorAttr(
    std::ostream& os,
    const char* kind,
    const rknn_tensor_attr& attr) {

    os << kind << '[' << attr.index << ']'
       << " name=" << attr.name
       << " dims=[";

    for (uint32_t i = 0; i < attr.n_dims; ++i) {
        if (i != 0) {
            os << ", ";
        }
        os << attr.dims[i];
    }

    os << ']'
       << " n_elems=" << attr.n_elems
       << " size=" << attr.size
       << " size_with_stride=" << attr.size_with_stride
       << " w_stride=" << attr.w_stride
       << " fmt=" << get_format_string(attr.fmt)
       << " type=" << get_type_string(attr.type)
       << " qnt=" << get_qnt_type_string(attr.qnt_type)
       << " zp=" << attr.zp
       << " scale=" << attr.scale;

    if (attr.qnt_type == RKNN_TENSOR_QNT_DFP) {
        os << " fl=" << static_cast<int>(attr.fl);
    }

    os << '\n';
}

void DumpImageInputSummary(const rknn_tensor_attr& attr) {
    if (attr.n_dims != 4) {
        std::cout
            << "Input tensor is not a four-dimensional image tensor; "
            << "n_dims=" << attr.n_dims << '\n';
        return;
    }

    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t channels = 0;

    if (attr.fmt == RKNN_TENSOR_NCHW) {
        channels = attr.dims[1];
        height = attr.dims[2];
        width = attr.dims[3];
    } else if (attr.fmt == RKNN_TENSOR_NHWC) {
        height = attr.dims[1];
        width = attr.dims[2];
        channels = attr.dims[3];
    } else {
        std::cout
            << "Input tensor format is not NCHW or NHWC: "
            << get_format_string(attr.fmt) << '\n';
        return;
    }

    std::cout
        << "image input summary: width=" << width
        << ", height=" << height
        << ", channels=" << channels
        << ", model_fmt=" << get_format_string(attr.fmt)
        << ", model_type=" << get_type_string(attr.type)
        << '\n';
}

}  // namespace

RknnEngine::~RknnEngine() {
    Shutdown();
}

bool RknnEngine::CheckRet(const char* api, int ret) {
    if (ret == RKNN_SUCC) {
        return true;
    }

    std::cerr << api << " failed, ret=" << ret << '\n';
    return false;
}

bool RknnEngine::Init(const std::string& model_path) {
    Shutdown();

    std::vector<uint8_t> model = ReadFile(model_path);
    if (model.empty()) {
        std::cerr << "Failed to read RKNN model: " << model_path << '\n';
        return false;
    }

    if (model.size() > std::numeric_limits<uint32_t>::max()) {
        std::cerr << "RKNN model is too large: " << model.size() << " bytes\n";
        return false;
    }

    int ret = rknn_init(
        &ctx_,
        model.data(),
        static_cast<uint32_t>(model.size()),
        0,
        nullptr);

    if (!CheckRet("rknn_init", ret)) {
        ctx_ = 0;
        return false;
    }

    initialized_ = true;

    if (!QueryModel()) {
        Shutdown();
        return false;
    }

    return true;
}

bool RknnEngine::QueryModel() {
    if (!initialized_) {
        std::cerr << "QueryModel called before initialization\n";
        return false;
    }

    rknn_sdk_version version{};
    int ret = rknn_query(
        ctx_,
        RKNN_QUERY_SDK_VERSION,
        &version,
        sizeof(version));

    if (!CheckRet("RKNN_QUERY_SDK_VERSION", ret)) {
        return false;
    }

    model_info_.api_version = version.api_version;
    model_info_.driver_version = version.drv_version;

    rknn_input_output_num io_num{};
    ret = rknn_query(
        ctx_,
        RKNN_QUERY_IN_OUT_NUM,
        &io_num,
        sizeof(io_num));

    if (!CheckRet("RKNN_QUERY_IN_OUT_NUM", ret)) {
        return false;
    }

    if (io_num.n_input == 0 || io_num.n_output == 0) {
        std::cerr
            << "Invalid model I/O count: inputs=" << io_num.n_input
            << ", outputs=" << io_num.n_output << '\n';
        return false;
    }

    model_info_.inputs.clear();
    model_info_.outputs.clear();
    model_info_.inputs.reserve(io_num.n_input);
    model_info_.outputs.reserve(io_num.n_output);

    std::cout << "========== RKNN MODEL CONTRACT ==========" << '\n';
    std::cout << "RKNN API version: " << model_info_.api_version << '\n';
    std::cout << "RKNN driver version: " << model_info_.driver_version << '\n';
    std::cout << "input count: " << io_num.n_input << '\n';
    std::cout << "output count: " << io_num.n_output << '\n';

    for (uint32_t i = 0; i < io_num.n_input; ++i) {
        rknn_tensor_attr attr{};
        attr.index = i;

        ret = rknn_query(
            ctx_,
            RKNN_QUERY_INPUT_ATTR,
            &attr,
            sizeof(attr));

        if (!CheckRet("RKNN_QUERY_INPUT_ATTR", ret)) {
            return false;
        }

        model_info_.inputs.push_back(attr);
        DumpTensorAttr(std::cout, "input", attr);
    }

    for (uint32_t i = 0; i < io_num.n_output; ++i) {
        rknn_tensor_attr attr{};
        attr.index = i;

        ret = rknn_query(
            ctx_,
            RKNN_QUERY_OUTPUT_ATTR,
            &attr,
            sizeof(attr));

        if (!CheckRet("RKNN_QUERY_OUTPUT_ATTR", ret)) {
            return false;
        }

        model_info_.outputs.push_back(attr);
        DumpTensorAttr(std::cout, "output", attr);
    }

    if (model_info_.inputs.size() == 1) {
        DumpImageInputSummary(model_info_.inputs.front());
    }

    std::cout << "=========================================" << '\n';
    return true;
}

bool RknnEngine::Infer(
    const InputTensor& input,
    std::vector<OutputTensor>* outputs) {

    if (!initialized_) {
        std::cerr << "Infer called before Init\n";
        return false;
    }

    if (outputs == nullptr) {
        std::cerr << "Infer received a null outputs pointer\n";
        return false;
    }

    outputs->clear();

    if (input.data == nullptr) {
        std::cerr << "Infer received a null input buffer\n";
        return false;
    }

    if (model_info_.inputs.size() != 1) {
        std::cerr
            << "This implementation expects exactly one model input; model has "
            << model_info_.inputs.size() << '\n';
        return false;
    }

    if (input.index >= model_info_.inputs.size()) {
        std::cerr
            << "Input index out of range: " << input.index
            << ", input count=" << model_info_.inputs.size() << '\n';
        return false;
    }

    const uint32_t element_bytes = TensorTypeBytes(input.type);
    if (element_bytes == 0) {
        std::cerr
            << "Unsupported host input type: "
            << static_cast<int>(input.type) << '\n';
        return false;
    }

    const rknn_tensor_attr& expected = model_info_.inputs[input.index];
    const uint64_t expected_bytes =
        static_cast<uint64_t>(expected.n_elems) * element_bytes;

    if (expected_bytes > std::numeric_limits<uint32_t>::max()) {
        std::cerr << "Expected input byte size exceeds uint32_t\n";
        return false;
    }

    if (input.size != static_cast<uint32_t>(expected_bytes)) {
        std::cerr
            << "Input size mismatch: expected=" << expected_bytes
            << " bytes for host type=" << get_type_string(input.type)
            << ", actual=" << input.size << " bytes\n";
        return false;
    }

    rknn_input rknn_in{};
    rknn_in.index = input.index;
    rknn_in.buf = input.data;
    rknn_in.size = input.size;
    rknn_in.type = input.type;
    rknn_in.fmt = input.format;
    rknn_in.pass_through = 0;

    int ret = rknn_inputs_set(ctx_, 1, &rknn_in);
    if (!CheckRet("rknn_inputs_set", ret)) {
        return false;
    }

    ret = rknn_run(ctx_, nullptr);
    if (!CheckRet("rknn_run", ret)) {
        return false;
    }

    const uint32_t output_count =
        static_cast<uint32_t>(model_info_.outputs.size());

    std::vector<rknn_output> raw_outputs(output_count);
    for (uint32_t i = 0; i < output_count; ++i) {
        raw_outputs[i].index = i;
        raw_outputs[i].want_float = 0;
        raw_outputs[i].is_prealloc = 0;
        raw_outputs[i].buf = nullptr;
        raw_outputs[i].size = 0;
    }

    ret = rknn_outputs_get(
        ctx_,
        output_count,
        raw_outputs.data(),
        nullptr);

    if (!CheckRet("rknn_outputs_get", ret)) {
        return false;
    }

    struct OutputGuard {
        rknn_context ctx = 0;
        std::vector<rknn_output>* outputs = nullptr;

        ~OutputGuard() {
            if (outputs == nullptr || outputs->empty()) {
                return;
            }

            const int release_ret = rknn_outputs_release(
                ctx,
                static_cast<uint32_t>(outputs->size()),
                outputs->data());

            if (release_ret != RKNN_SUCC) {
                std::cerr
                    << "rknn_outputs_release failed, ret="
                    << release_ret << '\n';
            }
        }
    } guard;

    guard.ctx = ctx_;
    guard.outputs = &raw_outputs;

    outputs->reserve(output_count);

    for (uint32_t i = 0; i < output_count; ++i) {
        const rknn_tensor_attr& attr = model_info_.outputs[i];
        const rknn_output& raw = raw_outputs[i];

        if (raw.buf == nullptr) {
            std::cerr << "Output " << i << " has a null buffer\n";
            outputs->clear();
            return false;
        }

        OutputTensor out;
        out.index = i;
        out.name = attr.name;
        out.type = attr.type;
        out.format = attr.fmt;
        out.qnt_type = attr.qnt_type;
        out.zero_point = attr.zp;
        out.scale = attr.scale;
        out.dims.reserve(attr.n_dims);

        for (uint32_t d = 0; d < attr.n_dims; ++d) {
            out.dims.push_back(attr.dims[d]);
        }

        const uint8_t* begin =
            static_cast<const uint8_t*>(raw.buf);
        out.data.assign(begin, begin + raw.size);

        outputs->push_back(std::move(out));
    }

    return true;
}

void RknnEngine::Shutdown() {
    if (initialized_) {
        const int ret = rknn_destroy(ctx_);
        if (ret != RKNN_SUCC) {
            std::cerr << "rknn_destroy failed, ret=" << ret << '\n';
        }
    }

    ctx_ = 0;
    initialized_ = false;
    model_info_ = {};
}
