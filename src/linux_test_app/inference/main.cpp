#include "rknn_engine.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include <opencv2/opencv.hpp>

namespace {

struct ImageShape {
    int width = 0;
    int height = 0;
    int channels = 0;
};

struct LetterboxInfo {
    float scale = 1.0f;
    int pad_x = 0;
    int pad_y = 0;
    int resized_width = 0;
    int resized_height = 0;
};

bool GetImageShape(
    const rknn_tensor_attr& attr,
    ImageShape* shape) {

    if (shape == nullptr || attr.n_dims != 4) {
        return false;
    }

    if (attr.fmt == RKNN_TENSOR_NCHW) {
        shape->channels = static_cast<int>(attr.dims[1]);
        shape->height = static_cast<int>(attr.dims[2]);
        shape->width = static_cast<int>(attr.dims[3]);
        return true;
    }

    if (attr.fmt == RKNN_TENSOR_NHWC) {
        shape->height = static_cast<int>(attr.dims[1]);
        shape->width = static_cast<int>(attr.dims[2]);
        shape->channels = static_cast<int>(attr.dims[3]);
        return true;
    }

    return false;
}

bool LetterboxBgrToRgb(
    const cv::Mat& source_bgr,
    int model_width,
    int model_height,
    cv::Mat* destination_rgb,
    LetterboxInfo* info) {

    if (source_bgr.empty() ||
        destination_rgb == nullptr ||
        info == nullptr ||
        model_width <= 0 ||
        model_height <= 0) {
        return false;
    }

    cv::Mat source_rgb;
    cv::cvtColor(source_bgr, source_rgb, cv::COLOR_BGR2RGB);

    const float scale = std::min(
        static_cast<float>(model_width) / source_rgb.cols,
        static_cast<float>(model_height) / source_rgb.rows);

    const int resized_width = std::max(
        1,
        static_cast<int>(std::round(source_rgb.cols * scale)));

    const int resized_height = std::max(
        1,
        static_cast<int>(std::round(source_rgb.rows * scale)));

    cv::Mat resized_rgb;
    cv::resize(
        source_rgb,
        resized_rgb,
        cv::Size(resized_width, resized_height),
        0.0,
        0.0,
        cv::INTER_LINEAR);

    const int pad_x = (model_width - resized_width) / 2;
    const int pad_y = (model_height - resized_height) / 2;

    *destination_rgb = cv::Mat(
        model_height,
        model_width,
        CV_8UC3,
        cv::Scalar(114, 114, 114));

    const cv::Rect roi(
        pad_x,
        pad_y,
        resized_width,
        resized_height);

    resized_rgb.copyTo((*destination_rgb)(roi));

    info->scale = scale;
    info->pad_x = pad_x;
    info->pad_y = pad_y;
    info->resized_width = resized_width;
    info->resized_height = resized_height;
    return true;
}

bool EnsureDirectory(const std::string& path) {
    if (path.empty()) {
        return false;
    }

    if (::mkdir(path.c_str(), 0755) == 0) {
        return true;
    }

    return errno == EEXIST;
}

std::string SanitizeFileName(const std::string& value) {
    std::string result;
    result.reserve(value.size());

    for (char ch : value) {
        const bool safe =
            (ch >= 'a' && ch <= 'z') ||
            (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9') ||
            ch == '-' || ch == '_';

        result.push_back(safe ? ch : '_');
    }

    if (result.empty()) {
        result = "unnamed";
    }

    return result;
}

std::string DimsToString(const std::vector<uint32_t>& dims) {
    std::ostringstream os;
    os << '[';
    for (size_t i = 0; i < dims.size(); ++i) {
        if (i != 0) {
            os << ", ";
        }
        os << dims[i];
    }
    os << ']';
    return os.str();
}

void WriteTensorAttr(
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
       << " fmt=" << get_format_string(attr.fmt)
       << " type=" << get_type_string(attr.type)
       << " qnt=" << get_qnt_type_string(attr.qnt_type)
       << " zp=" << attr.zp
       << " scale=" << attr.scale
       << '\n';
}

bool SaveContract(
    const std::string& path,
    const ModelInfo& model_info,
    const ImageShape& image_shape,
    const LetterboxInfo* letterbox,
    const cv::Size* original_size) {

    std::ofstream os(path);
    if (!os) {
        return false;
    }

    os << "[V3 RKNN TENSOR CONTRACT]\n";
    os << "RKNN API version: " << model_info.api_version << '\n';
    os << "RKNN driver version: " << model_info.driver_version << '\n';
    os << "task: obtain from training command/data, expected detect\n";
    os << "quantization: inspect output qnt/type below; expected INT8 affine\n";
    os << "input count: " << model_info.inputs.size() << '\n';
    os << "output count: " << model_info.outputs.size() << '\n';
    os << "model input width: " << image_shape.width << '\n';
    os << "model input height: " << image_shape.height << '\n';
    os << "model input channels: " << image_shape.channels << '\n';
    os << "host input color: RGB\n";
    os << "host input type: UINT8\n";
    os << "host input layout: NHWC\n";
    os << "host input range: 0..255\n";
    os << "preprocess: aspect-ratio-preserving letterbox\n";
    os << "padding color: 114,114,114\n";
    os << "crop: no\n";
    os << "raw output coordinate format: raw feature maps, not xyxy\n";
    os << "NMS: not performed by this program\n";
    os << "class count: obtain from data.yaml\n";
    os << "ping-pong class id: obtain from data.yaml names order\n";

    if (letterbox != nullptr && original_size != nullptr) {
        os << "test image original width: " << original_size->width << '\n';
        os << "test image original height: " << original_size->height << '\n';
        os << "letterbox scale: " << letterbox->scale << '\n';
        os << "letterbox pad_x: " << letterbox->pad_x << '\n';
        os << "letterbox pad_y: " << letterbox->pad_y << '\n';
        os << "letterbox resized width: "
           << letterbox->resized_width << '\n';
        os << "letterbox resized height: "
           << letterbox->resized_height << '\n';
    }

    os << "\n[INPUT TENSORS]\n";
    for (const auto& attr : model_info.inputs) {
        WriteTensorAttr(os, "input", attr);
    }

    os << "\n[OUTPUT TENSORS]\n";
    for (const auto& attr : model_info.outputs) {
        WriteTensorAttr(os, "output", attr);
    }

    return true;
}

bool SaveOutputs(
    const std::string& output_dir,
    const std::vector<OutputTensor>& outputs) {

    for (const OutputTensor& output : outputs) {
        std::ostringstream filename;
        filename
            << output_dir
            << "/output_"
            << output.index
            << '_'
            << SanitizeFileName(output.name)
            << ".bin";

        std::ofstream os(filename.str(), std::ios::binary);
        if (!os) {
            std::cerr
                << "Failed to open output file: "
                << filename.str() << '\n';
            return false;
        }

        os.write(
            reinterpret_cast<const char*>(output.data.data()),
            static_cast<std::streamsize>(output.data.size()));

        if (!os) {
            std::cerr
                << "Failed to write output file: "
                << filename.str() << '\n';
            return false;
        }

        std::cout
            << "saved " << filename.str()
            << " bytes=" << output.data.size()
            << " dims=" << DimsToString(output.dims)
            << " type=" << get_type_string(output.type)
            << " qnt=" << get_qnt_type_string(output.qnt_type)
            << " zp=" << output.zero_point
            << " scale=" << output.scale
            << '\n';
    }

    return true;
}

void PrintUsage(const char* program) {
    std::cerr
        << "Usage:\n"
        << "  " << program << " model.rknn\n"
        << "      Load the model and print its tensor contract.\n\n"
        << "  " << program
        << " model.rknn image.jpg [output_dir]\n"
        << "      Letterbox the image, run one inference, and dump raw outputs.\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2 && argc != 3 && argc != 4) {
        PrintUsage(argv[0]);
        return 1;
    }

    const std::string model_path = argv[1];

    RknnEngine engine;
    if (!engine.Init(model_path)) {
        std::cerr << "Failed to initialize RKNN engine\n";
        return 1;
    }

    const ModelInfo& model_info = engine.GetModelInfo();
    if (model_info.inputs.size() != 1) {
        std::cerr
            << "This demo expects one input tensor; model has "
            << model_info.inputs.size() << '\n';
        return 1;
    }

    ImageShape image_shape;
    if (!GetImageShape(model_info.inputs[0], &image_shape)) {
        std::cerr
            << "Cannot interpret input[0] as a four-dimensional "
            << "NCHW/NHWC image tensor\n";
        return 1;
    }

    if (image_shape.channels != 3) {
        std::cerr
            << "This demo expects a three-channel image; model has "
            << image_shape.channels << " channels\n";
        return 1;
    }

    if (argc == 2) {
        std::cout
            << "Model inspection completed. No image inference requested.\n";
        return 0;
    }

    const std::string image_path = argv[2];
    const std::string output_dir =
        argc == 4 ? argv[3] : "rknn_dump";

    if (!EnsureDirectory(output_dir)) {
        std::cerr
            << "Failed to create/access output directory: "
            << output_dir << ", error=" << std::strerror(errno) << '\n';
        return 1;
    }

    cv::Mat source_bgr = cv::imread(image_path, cv::IMREAD_COLOR);
    if (source_bgr.empty()) {
        std::cerr << "Failed to read image: " << image_path << '\n';
        return 1;
    }

    cv::Mat input_rgb;
    LetterboxInfo letterbox;
    if (!LetterboxBgrToRgb(
            source_bgr,
            image_shape.width,
            image_shape.height,
            &input_rgb,
            &letterbox)) {
        std::cerr << "Image preprocessing failed\n";
        return 1;
    }

    if (!input_rgb.isContinuous()) {
        input_rgb = input_rgb.clone();
    }

    const uint64_t input_bytes =
        static_cast<uint64_t>(input_rgb.total()) * input_rgb.elemSize();

    if (input_bytes > std::numeric_limits<uint32_t>::max()) {
        std::cerr << "Input image buffer is too large\n";
        return 1;
    }

    InputTensor input;
    input.index = 0;
    input.data = input_rgb.data;
    input.size = static_cast<uint32_t>(input_bytes);
    input.type = RKNN_TENSOR_UINT8;
    input.format = RKNN_TENSOR_NHWC;

    std::cout
        << "preprocess: original="
        << source_bgr.cols << 'x' << source_bgr.rows
        << ", model="
        << image_shape.width << 'x' << image_shape.height
        << ", scale=" << letterbox.scale
        << ", pad_x=" << letterbox.pad_x
        << ", pad_y=" << letterbox.pad_y
        << ", host_color=RGB"
        << ", host_type=UINT8"
        << ", host_fmt=NHWC\n";

    std::vector<OutputTensor> outputs;
    if (!engine.Infer(input, &outputs)) {
        std::cerr << "RKNN inference failed\n";
        return 1;
    }

    std::cout
        << "RKNN inference succeeded; raw output count="
        << outputs.size() << '\n';

    if (!SaveOutputs(output_dir, outputs)) {
        return 1;
    }

    cv::Mat preview_bgr;
    cv::cvtColor(input_rgb, preview_bgr, cv::COLOR_RGB2BGR);
    const std::string preview_path = output_dir + "/letterbox_preview.png";
    if (!cv::imwrite(preview_path, preview_bgr)) {
        std::cerr << "Failed to save preview: " << preview_path << '\n';
        return 1;
    }

    const cv::Size original_size(source_bgr.cols, source_bgr.rows);
    const std::string contract_path = output_dir + "/tensor_contract.txt";
    if (!SaveContract(
            contract_path,
            model_info,
            image_shape,
            &letterbox,
            &original_size)) {
        std::cerr << "Failed to save contract: " << contract_path << '\n';
        return 1;
    }

    std::cout << "saved " << preview_path << '\n';
    std::cout << "saved " << contract_path << '\n';
    std::cout
        << "This program only dumps raw RKNN tensors. "
        << "It does not perform YOLOv8 DFL decoding or NMS.\n";

    return 0;
}
