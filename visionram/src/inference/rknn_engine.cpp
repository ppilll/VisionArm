#include "inference/rknn_engine.h"

#include "common/monotonic_clock.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace visionarm {
namespace {

[[nodiscard]] std::vector<uint8_t> ReadFile(const std::string& path) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream) {
        throw std::runtime_error("failed to open RKNN model: " + path);
    }
    const std::streamoff end = stream.tellg();
    if (end <= 0 || static_cast<uint64_t>(end) >
            static_cast<uint64_t>(std::numeric_limits<uint32_t>::max())) {
        throw std::runtime_error("invalid RKNN model size");
    }
    std::vector<uint8_t> data(static_cast<std::size_t>(end));
    stream.seekg(0, std::ios::beg);
    if (!stream.read(reinterpret_cast<char*>(data.data()),
                     static_cast<std::streamsize>(data.size()))) {
        throw std::runtime_error("failed to read RKNN model: " + path);
    }
    return data;
}

[[nodiscard]] uint32_t TensorTypeBytes(rknn_tensor_type type) noexcept {
    switch (type) {
        case RKNN_TENSOR_FLOAT32:
        case RKNN_TENSOR_INT32:
        case RKNN_TENSOR_UINT32:
            return 4U;
        case RKNN_TENSOR_FLOAT16:
        case RKNN_TENSOR_INT16:
        case RKNN_TENSOR_UINT16:
            return 2U;
        case RKNN_TENSOR_INT64:
            return 8U;
        case RKNN_TENSOR_INT8:
        case RKNN_TENSOR_UINT8:
        case RKNN_TENSOR_BOOL:
            return 1U;
        default:
            return 0U;
    }
}

[[nodiscard]] TensorDataType ConvertType(rknn_tensor_type type) noexcept {
    switch (type) {
        case RKNN_TENSOR_INT8: return TensorDataType::INT8;
        case RKNN_TENSOR_UINT8: return TensorDataType::UINT8;
        case RKNN_TENSOR_FLOAT16: return TensorDataType::FLOAT16;
        case RKNN_TENSOR_FLOAT32: return TensorDataType::FLOAT32;
        default: return TensorDataType::UNKNOWN;
    }
}

[[nodiscard]] TensorLayout ConvertLayout(rknn_tensor_format format) noexcept {
    switch (format) {
        case RKNN_TENSOR_NCHW: return TensorLayout::NCHW;
        case RKNN_TENSOR_NHWC: return TensorLayout::NHWC;
        default: return TensorLayout::UNKNOWN;
    }
}

[[nodiscard]] TensorQuantization ConvertQuantization(
    rknn_tensor_qnt_type type) noexcept {
    switch (type) {
        case RKNN_TENSOR_QNT_NONE:
            return TensorQuantization::NONE;
        case RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC:
            return TensorQuantization::AFFINE_ASYMMETRIC;
        default:
            return TensorQuantization::OTHER;
    }
}

[[nodiscard]] ImageShape ShapeFromAttr(const rknn_tensor_attr& attr) {
    if (attr.n_dims != 4U) {
        throw std::runtime_error("RKNN image input must be four-dimensional");
    }
    ImageShape shape;
    if (attr.fmt == RKNN_TENSOR_NCHW) {
        shape.batch = attr.dims[0];
        shape.channels = attr.dims[1];
        shape.height = attr.dims[2];
        shape.width = attr.dims[3];
    } else if (attr.fmt == RKNN_TENSOR_NHWC) {
        shape.batch = attr.dims[0];
        shape.height = attr.dims[1];
        shape.width = attr.dims[2];
        shape.channels = attr.dims[3];
    } else {
        throw std::runtime_error("unsupported RKNN image tensor format");
    }
    return shape;
}

[[nodiscard]] uint32_t CheckedImageBytes(
    const ImageShape& shape,
    uint32_t element_bytes) {
    const uint64_t bytes = static_cast<uint64_t>(shape.batch) *
        shape.width * shape.height * shape.channels * element_bytes;
    if (bytes == 0U || bytes > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error("invalid RKNN input allocation size");
    }
    return static_cast<uint32_t>(bytes);
}

void ResetTiming(RknnRunTiming* timing) noexcept {
    if (timing != nullptr) {
        *timing = RknnRunTiming{};
    }
}

}  // namespace

const char* RknnIoModeName(RknnIoMode mode) noexcept {
    switch (mode) {
        case RknnIoMode::INPUTS_SET_PREALLOC_OUTPUT:
            return "inputs_set_prealloc_output";
        case RknnIoMode::BOUND_HOST_IO:
            return "bound_host_io";
        case RknnIoMode::BOUND_NATIVE_INPUT_LOGICAL_OUTPUT:
            return "bound_native_input_logical_output";
    }
    return "unknown";
}

RknnEngine::~RknnEngine() {
    Shutdown();
}

void RknnEngine::CheckRknn(const char* operation, int result) {
    if (result != RKNN_SUCC) {
        throw std::runtime_error(
            std::string(operation) + " failed, ret=" +
            std::to_string(result));
    }
}

void RknnEngine::Initialize(const RknnEngineConfig& config) {
    Shutdown();
    if (config.model_path.empty() || config.input_slot_count == 0U ||
        config.output_slot_count == 0U) {
        throw std::invalid_argument("invalid RKNN engine configuration");
    }
    if (TensorTypeBytes(config.host_input_type) == 0U ||
        (config.host_input_format != RKNN_TENSOR_NHWC &&
         config.host_input_format != RKNN_TENSOR_NCHW)) {
        throw std::invalid_argument("unsupported RKNN host input contract");
    }

    std::vector<uint8_t> model = ReadFile(config.model_path);
    CheckRknn("rknn_init", rknn_init(
        &context_, model.data(), static_cast<uint32_t>(model.size()), 0,
        nullptr));

    initialized_ = true;
    config_ = config;
    try {
        QueryModel();
        QueryNativeInput();
        ParseInputShape();
        ConfigureInputContract();
        AllocateSlots();
    } catch (...) {
        Shutdown();
        throw;
    }
}

void RknnEngine::QueryModel() {
    rknn_sdk_version version{};
    CheckRknn("RKNN_QUERY_SDK_VERSION", rknn_query(
        context_, RKNN_QUERY_SDK_VERSION, &version, sizeof(version)));
    model_info_.api_version = version.api_version;
    model_info_.driver_version = version.drv_version;

    rknn_input_output_num counts{};
    CheckRknn("RKNN_QUERY_IN_OUT_NUM", rknn_query(
        context_, RKNN_QUERY_IN_OUT_NUM, &counts, sizeof(counts)));
    if (counts.n_input != 1U || counts.n_output == 0U) {
        throw std::runtime_error(
            "pipeline requires one input and at least one output");
    }

    model_info_.input_attributes.clear();
    model_info_.output_attributes.clear();
    model_info_.input_attributes.reserve(counts.n_input);
    model_info_.output_attributes.reserve(counts.n_output);

    for (uint32_t index = 0; index < counts.n_input; ++index) {
        rknn_tensor_attr attr{};
        attr.index = index;
        CheckRknn("RKNN_QUERY_INPUT_ATTR", rknn_query(
            context_, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr)));
        model_info_.input_attributes.push_back(attr);
    }
    for (uint32_t index = 0; index < counts.n_output; ++index) {
        rknn_tensor_attr attr{};
        attr.index = index;
        CheckRknn("RKNN_QUERY_OUTPUT_ATTR", rknn_query(
            context_, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)));
        model_info_.output_attributes.push_back(attr);
    }
}

void RknnEngine::QueryNativeInput() {
    rknn_tensor_attr attr{};
    attr.index = 0U;

    // Command value 10 is RKNN_QUERY_NATIVE_NHWC_INPUT_ATTR in current
    // RKNPU2 headers. Older runtimes may reject it; command 8 is the general
    // native input query and is used as a fallback.
    int result = rknn_query(
        context_, static_cast<rknn_query_cmd>(10), &attr, sizeof(attr));
    if (result != RKNN_SUCC) {
        attr = {};
        attr.index = 0U;
        result = rknn_query(
            context_, RKNN_QUERY_NATIVE_INPUT_ATTR, &attr, sizeof(attr));
    }

    model_info_.native_input_query_succeeded = result == RKNN_SUCC;
    if (result == RKNN_SUCC) {
        model_info_.native_input_attribute = attr;
    }
}

void RknnEngine::ParseInputShape() {
    input_shape_ = ShapeFromAttr(model_info_.input_attributes.front());
    if (input_shape_.batch != 1U || input_shape_.channels != 3U ||
        input_shape_.width == 0U || input_shape_.height == 0U) {
        throw std::runtime_error(
            "pipeline requires batch=1 and three-channel image input");
    }
}

void RknnEngine::ConfigureInputContract() {
    selected_output_attrs_ = model_info_.output_attributes;
    for (rknn_tensor_attr& output_attr : selected_output_attrs_) {
        output_attr.pass_through = 0U;
    }
    native_direct_input_supported_ = false;

    const uint32_t host_element_bytes =
        TensorTypeBytes(config_.host_input_type);
    const uint32_t host_bytes =
        CheckedImageBytes(input_shape_, host_element_bytes);

    if (config_.io_mode == RknnIoMode::BOUND_NATIVE_INPUT_LOGICAL_OUTPUT) {
        if (!model_info_.native_input_query_succeeded) {
            throw std::runtime_error("native RKNN input attr query failed");
        }
        selected_input_attr_ = model_info_.native_input_attribute;
        ImageShape native_shape = ShapeFromAttr(selected_input_attr_);
        native_direct_input_supported_ =
            selected_input_attr_.fmt == RKNN_TENSOR_NHWC &&
            selected_input_attr_.type == RKNN_TENSOR_UINT8 &&
            native_shape.batch == input_shape_.batch &&
            native_shape.width == input_shape_.width &&
            native_shape.height == input_shape_.height &&
            native_shape.channels == input_shape_.channels;
        if (!native_direct_input_supported_) {
            throw std::runtime_error(
                "native input is not RGA-compatible NHWC UINT8 RGB");
        }
        selected_input_attr_.pass_through = 1U;
        input_allocation_bytes_ = std::max(
            selected_input_attr_.size,
            selected_input_attr_.size_with_stride);
        const uint32_t width_stride =
            selected_input_attr_.w_stride != 0U
                ? selected_input_attr_.w_stride
                : input_shape_.width;
        input_row_stride_bytes_ = width_stride * input_shape_.channels;
        input_memory_layout_ = ModelInputMemoryLayout::RGB_UINT8_NHWC;
        return;
    }

    selected_input_attr_ = model_info_.input_attributes.front();
    selected_input_attr_.type = config_.host_input_type;
    selected_input_attr_.fmt = config_.host_input_format;
    selected_input_attr_.pass_through = 0U;
    selected_input_attr_.w_stride = input_shape_.width;
    selected_input_attr_.h_stride = input_shape_.height;
    selected_input_attr_.size = host_bytes;
    selected_input_attr_.size_with_stride = host_bytes;
    input_allocation_bytes_ = host_bytes;

    if (config_.host_input_format != RKNN_TENSOR_NHWC) {
        input_row_stride_bytes_ = input_shape_.width * host_element_bytes;
        input_memory_layout_ = ModelInputMemoryLayout::OPAQUE_NATIVE;
    } else {
        input_row_stride_bytes_ =
            input_shape_.width * input_shape_.channels * host_element_bytes;
        input_memory_layout_ =
            config_.host_input_type == RKNN_TENSOR_UINT8
                ? ModelInputMemoryLayout::RGB_UINT8_NHWC
                : ModelInputMemoryLayout::RGB_INT8_NHWC;
    }
}

void RknnEngine::AllocateSlots() {
    if (input_allocation_bytes_ == 0U) {
        throw std::runtime_error("RKNN input allocation size is zero");
    }

    input_slots_.resize(config_.input_slot_count);
    for (std::size_t index = 0; index < input_slots_.size(); ++index) {
        InputSlot& slot = input_slots_[index];

        void* input_address = nullptr;
        int input_fd = -1;
        std::size_t input_capacity = 0U;
        std::size_t input_offset = 0U;

        if (config_.input_dma_heap_path.empty()) {
            slot.memory = rknn_create_mem(context_, input_allocation_bytes_);
            if (slot.memory == nullptr || slot.memory->virt_addr == nullptr ||
                slot.memory->fd < 0 ||
                slot.memory->size < input_allocation_bytes_) {
                throw std::runtime_error(
                    "failed to allocate RKNN input slot");
            }
            if (slot.memory->offset < 0) {
                throw std::runtime_error(
                    "RKNN input slot has negative DMA offset");
            }
            input_address = slot.memory->virt_addr;
            input_fd = slot.memory->fd;
            input_capacity = slot.memory->size;
            input_offset = static_cast<std::size_t>(slot.memory->offset);
        } else {
            slot.external_buffer = DmaHeapBuffer::Allocate(
                config_.input_dma_heap_path,
                input_allocation_bytes_);
            if (slot.external_buffer.size() >
                static_cast<std::size_t>(
                    std::numeric_limits<uint32_t>::max())) {
                throw std::runtime_error(
                    "external RKNN input DMA-BUF exceeds RKNN size limit");
            }

            slot.memory = rknn_create_mem_from_fd(
                context_,
                slot.external_buffer.fd(),
                slot.external_buffer.address(),
                input_allocation_bytes_,
                0);
            if (slot.memory == nullptr) {
                throw std::runtime_error(
                    "rknn_create_mem_from_fd failed for input slot " +
                    std::to_string(index));
            }

            input_address = slot.external_buffer.address();
            input_fd = slot.external_buffer.fd();
            input_capacity = slot.external_buffer.size();
            input_offset = 0U;
        }

        slot.view.slot_index = index;
        slot.view.cpu_address = input_address;
        slot.view.dma_fd = input_fd;
        slot.view.dma_offset = input_offset;
        slot.view.capacity_bytes = input_capacity;
        slot.view.width = static_cast<int>(input_shape_.width);
        slot.view.height = static_cast<int>(input_shape_.height);
        slot.view.channels = static_cast<int>(input_shape_.channels);
        slot.view.memory_layout = input_memory_layout_;
        slot.view.row_stride_bytes = input_row_stride_bytes_;
    }

    output_slots_.resize(config_.output_slot_count);
    for (OutputSlot& slot : output_slots_) {
        const std::size_t count = selected_output_attrs_.size();
        slot.memories.resize(count, nullptr);
        slot.descriptors.resize(count);
        slot.views.resize(count);
        slot.dma_fds.resize(count, -1);

        for (std::size_t index = 0; index < count; ++index) {
            const rknn_tensor_attr& attr = selected_output_attrs_[index];
            const uint32_t allocation_size =
                std::max(attr.size, attr.size_with_stride);
            if (allocation_size == 0U) {
                throw std::runtime_error("RKNN output has zero size");
            }
            rknn_tensor_mem* memory =
                rknn_create_mem(context_, allocation_size);
            if (memory == nullptr || memory->virt_addr == nullptr ||
                memory->size < allocation_size) {
                throw std::runtime_error("failed to allocate RKNN output slot");
            }
            slot.memories[index] = memory;
            slot.dma_fds[index] = memory->fd;

            rknn_output& descriptor = slot.descriptors[index];
            descriptor.index = static_cast<uint32_t>(index);
            descriptor.want_float = 0U;
            descriptor.is_prealloc = 1U;
            descriptor.buf = memory->virt_addr;
            descriptor.size = memory->size;

            TensorView& view = slot.views[index];
            view.index = attr.index;
            view.name = attr.name;
            view.data = memory->virt_addr;
            view.bytes = attr.size;
            view.data_type = ConvertType(attr.type);
            view.layout = ConvertLayout(attr.fmt);
            view.quantization = ConvertQuantization(attr.qnt_type);
            view.zero_point = attr.zp;
            view.scale = attr.scale;
            view.dims.reserve(attr.n_dims);
            for (uint32_t dimension = 0;
                 dimension < attr.n_dims; ++dimension) {
                view.dims.push_back(attr.dims[dimension]);
            }
        }
    }
}

const ModelInputBufferView* RknnEngine::input_buffer(
    std::size_t slot_index) const noexcept {
    if (!initialized_ || slot_index >= input_slots_.size()) {
        return nullptr;
    }
    return &input_slots_[slot_index].view;
}

const std::vector<TensorView>* RknnEngine::output_views(
    std::size_t slot_index) const noexcept {
    if (!initialized_ || slot_index >= output_slots_.size()) {
        return nullptr;
    }
    return &output_slots_[slot_index].views;
}

const std::vector<int>* RknnEngine::output_dma_fds(
    std::size_t slot_index) const noexcept {
    if (!initialized_ || slot_index >= output_slots_.size()) {
        return nullptr;
    }
    return &output_slots_[slot_index].dma_fds;
}

bool RknnEngine::RunInputsSet(
    InputSlot& input_slot,
    OutputSlot& output_slot,
    RknnRunTiming* timing) noexcept {

    const int64_t submit_start = MonotonicNowNs();
    rknn_input input{};
    input.index = 0U;
    input.buf = input_slot.view.cpu_address;
    input.size = input_allocation_bytes_;
    input.type = config_.host_input_type;
    input.fmt = config_.host_input_format;
    input.pass_through = 0U;
    int result = rknn_inputs_set(context_, 1U, &input);
    if (timing != nullptr) {
        timing->input_submit_ns = MonotonicNowNs() - submit_start;
    }
    if (result != RKNN_SUCC) {
        std::cerr << "rknn_inputs_set failed, ret=" << result << '\n';
        return false;
    }

    const int64_t run_start = MonotonicNowNs();
    result = rknn_run(context_, nullptr);
    if (timing != nullptr) {
        timing->run_ns = MonotonicNowNs() - run_start;
    }
    if (result != RKNN_SUCC) {
        std::cerr << "rknn_run failed, ret=" << result << '\n';
        return false;
    }

    for (std::size_t index = 0; index < output_slot.descriptors.size(); ++index) {
        rknn_output& descriptor = output_slot.descriptors[index];
        descriptor.index = static_cast<uint32_t>(index);
        descriptor.want_float = 0U;
        descriptor.is_prealloc = 1U;
        descriptor.buf = output_slot.memories[index]->virt_addr;
        descriptor.size = output_slot.memories[index]->size;
    }

    const int64_t get_start = MonotonicNowNs();
    result = rknn_outputs_get(
        context_, static_cast<uint32_t>(output_slot.descriptors.size()),
        output_slot.descriptors.data(), nullptr);
    if (timing != nullptr) {
        timing->output_get_ns = MonotonicNowNs() - get_start;
    }
    if (result != RKNN_SUCC) {
        std::cerr << "rknn_outputs_get failed, ret=" << result << '\n';
        return false;
    }

    for (std::size_t index = 0; index < output_slot.views.size(); ++index) {
        const uint32_t logical_bytes =
            selected_output_attrs_[index].size;
        const uint32_t capacity =
            output_slot.memories[index]->size;

        if (logical_bytes == 0U || logical_bytes > capacity) {
            std::cerr
                << "invalid RKNN logical output size, index="
                << index
                << ", logical_bytes=" << logical_bytes
                << ", capacity=" << capacity
                << '\n';
            return false;
        }

        // rknn_output::size is the preallocated buffer capacity. It is not
        // an actual-bytes-returned field. Hash and postprocess only the
        // logical output tensor bytes.
        output_slot.views[index].bytes = logical_bytes;
    }

    const int64_t release_start = MonotonicNowNs();
    result = rknn_outputs_release(
        context_, static_cast<uint32_t>(output_slot.descriptors.size()),
        output_slot.descriptors.data());
    if (timing != nullptr) {
        timing->output_release_ns = MonotonicNowNs() - release_start;
    }
    if (result != RKNN_SUCC) {
        std::cerr << "rknn_outputs_release failed, ret=" << result << '\n';
        return false;
    }
    return true;
}

bool RknnEngine::BindIo(
    InputSlot& input_slot,
    OutputSlot& output_slot,
    RknnRunTiming* timing) noexcept {

    int result = RKNN_SUCC;
    if (bound_input_memory_ != input_slot.memory) {
        const int64_t input_start = MonotonicNowNs();
        result = rknn_set_io_mem(
            context_, input_slot.memory, &selected_input_attr_);
        if (timing != nullptr) {
            timing->input_submit_ns = MonotonicNowNs() - input_start;
        }
        if (result != RKNN_SUCC) {
            bound_input_memory_ = nullptr;
            std::cerr << "rknn_set_io_mem(input) failed, ret="
                      << result << '\n';
            return false;
        }
        bound_input_memory_ = input_slot.memory;
    }

    if (bound_output_slot_ != &output_slot) {
        const int64_t output_start = MonotonicNowNs();
        for (std::size_t index = 0;
             index < output_slot.memories.size();
             ++index) {
            result = rknn_set_io_mem(
                context_, output_slot.memories[index],
                &selected_output_attrs_[index]);
            if (result != RKNN_SUCC) {
                bound_output_slot_ = nullptr;
                std::cerr << "rknn_set_io_mem(output " << index
                          << ") failed, ret=" << result << '\n';
                return false;
            }
        }
        if (timing != nullptr) {
            timing->output_bind_ns = MonotonicNowNs() - output_start;
        }
        bound_output_slot_ = &output_slot;
    }
    return true;
}

bool RknnEngine::RunBoundIo(
    InputSlot& input_slot,
    OutputSlot& output_slot,
    RknnRunTiming* timing) noexcept {
    if (!BindIo(input_slot, output_slot, timing)) {
        return false;
    }
    const int64_t run_start = MonotonicNowNs();
    const int result = rknn_run(context_, nullptr);
    if (timing != nullptr) {
        timing->run_ns = MonotonicNowNs() - run_start;
    }
    if (result != RKNN_SUCC) {
        std::cerr << "rknn_run(bound IO) failed, ret=" << result << '\n';
        return false;
    }
    return true;
}

bool RknnEngine::Run(
    std::size_t input_slot_index,
    std::size_t output_slot_index,
    RknnRunTiming* timing) noexcept {
    ResetTiming(timing);
    if (!initialized_ || input_slot_index >= input_slots_.size() ||
        output_slot_index >= output_slots_.size()) {
        return false;
    }

    const int64_t total_start = MonotonicNowNs();
    bool ok = false;
    if (config_.io_mode == RknnIoMode::INPUTS_SET_PREALLOC_OUTPUT) {
        ok = RunInputsSet(
            input_slots_[input_slot_index],
            output_slots_[output_slot_index], timing);
    } else {
        ok = RunBoundIo(
            input_slots_[input_slot_index],
            output_slots_[output_slot_index], timing);
    }
    if (timing != nullptr) {
        timing->total_ns = MonotonicNowNs() - total_start;
    }
    return ok;
}

void RknnEngine::DestroySlots() noexcept {
    bound_input_memory_ = nullptr;
    bound_output_slot_ = nullptr;
    if (context_ == 0) {
        input_slots_.clear();
        output_slots_.clear();
        return;
    }
    for (OutputSlot& slot : output_slots_) {
        for (rknn_tensor_mem*& memory : slot.memories) {
            if (memory != nullptr) {
                const int result = rknn_destroy_mem(context_, memory);
                if (result != RKNN_SUCC) {
                    std::cerr << "rknn_destroy_mem(output) failed, ret="
                              << result << '\n';
                }
                memory = nullptr;
            }
        }
    }
    for (InputSlot& slot : input_slots_) {
        if (slot.memory != nullptr) {
            const int result = rknn_destroy_mem(context_, slot.memory);
            if (result != RKNN_SUCC) {
                std::cerr << "rknn_destroy_mem(input) failed, ret="
                          << result << '\n';
            }
            slot.memory = nullptr;
        }
        // rknn_destroy_mem() destroys only the RKNN wrapper for memory created
        // with rknn_create_mem_from_fd(). The dma-buf mmap/fd remain owned by
        // DmaHeapBuffer and are released after the wrapper is gone.
        slot.external_buffer.Reset();
    }
    input_slots_.clear();
    output_slots_.clear();
}

void RknnEngine::Shutdown() noexcept {
    DestroySlots();
    if (initialized_ && context_ != 0) {
        const int result = rknn_destroy(context_);
        if (result != RKNN_SUCC) {
            std::cerr << "rknn_destroy failed, ret=" << result << '\n';
        }
    }
    context_ = 0;
    initialized_ = false;
    native_direct_input_supported_ = false;
    config_ = {};
    model_info_ = {};
    input_shape_ = {};
    selected_input_attr_ = {};
    selected_output_attrs_.clear();
    input_allocation_bytes_ = 0U;
    input_row_stride_bytes_ = 0U;
    input_memory_layout_ = ModelInputMemoryLayout::RGB_UINT8_NHWC;
}

}  // namespace visionarm
