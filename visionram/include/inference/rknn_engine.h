#pragma once

#include "common/pipeline_types.h"
#include "common/tensor_view.h"
#include "memory/dma_heap_buffer.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "rknn_api.h"

namespace visionarm {

enum class RknnIoMode {
    // Existing verified baseline. Input is submitted with rknn_inputs_set();
    // outputs are obtained with preallocated rknn_outputs_get().
    INPUTS_SET_PREALLOC_OUTPUT,

    // Input/output rknn_tensor_mem objects are selected with rknn_set_io_mem.
    // The input attr uses pass_through=0 and RGB UINT8 NHWC, so Runtime may
    // still perform conversion into its native tensor layout.
    BOUND_HOST_IO,

    // Input is bound with the native input attr and pass_through=1. Outputs
    // remain bound with the logical model output attrs because the existing
    // Top-1 postprocessor consumes INT8 affine NCHW tensors. This mode is
    // accepted only when native input is NHWC UINT8 RGB.
    BOUND_NATIVE_INPUT_LOGICAL_OUTPUT,
};

[[nodiscard]] const char* RknnIoModeName(RknnIoMode mode) noexcept;

struct RknnRunTiming {
    int64_t input_submit_ns = 0;
    int64_t output_bind_ns = 0;
    int64_t run_ns = 0;
    int64_t output_get_ns = 0;
    int64_t output_release_ns = 0;
    int64_t total_ns = 0;
};

struct RknnModelInfo {
    std::string api_version;
    std::string driver_version;
    std::vector<rknn_tensor_attr> input_attributes;
    std::vector<rknn_tensor_attr> output_attributes;

    bool native_input_query_succeeded = false;
    rknn_tensor_attr native_input_attribute{};
};

struct RknnEngineConfig {
    std::string model_path;
    std::size_t input_slot_count = 2;
    std::size_t output_slot_count = 2;

    rknn_tensor_type host_input_type = RKNN_TENSOR_UINT8;
    rknn_tensor_format host_input_format = RKNN_TENSOR_NHWC;
    RknnIoMode io_mode = RknnIoMode::INPUTS_SET_PREALLOC_OUTPUT;

    // Empty: use rknn_create_mem() for input slots.
    // Non-empty: allocate every input slot from this Linux dma-heap and wrap
    // it with rknn_create_mem_from_fd(). For RK3588 RGA2 color fill, use a
    // heap whose complete backing store is below 4 GiB, normally:
    // /dev/dma_heap/system-uncached-dma32
    std::string input_dma_heap_path;
};

class RknnEngine final {
public:
    RknnEngine() = default;
    ~RknnEngine();

    RknnEngine(const RknnEngine&) = delete;
    RknnEngine& operator=(const RknnEngine&) = delete;

    void Initialize(const RknnEngineConfig& config);
    void Shutdown() noexcept;

    [[nodiscard]] bool initialized() const noexcept { return initialized_; }
    [[nodiscard]] RknnIoMode io_mode() const noexcept { return config_.io_mode; }
    [[nodiscard]] const RknnModelInfo& model_info() const noexcept {
        return model_info_;
    }
    [[nodiscard]] const ImageShape& input_shape() const noexcept {
        return input_shape_;
    }
    [[nodiscard]] bool native_direct_input_supported() const noexcept {
        return native_direct_input_supported_;
    }
    [[nodiscard]] bool input_uses_external_dma_heap() const noexcept {
        return !config_.input_dma_heap_path.empty();
    }
    [[nodiscard]] const std::string& input_dma_heap_path() const noexcept {
        return config_.input_dma_heap_path;
    }

    [[nodiscard]] std::size_t input_slot_count() const noexcept {
        return input_slots_.size();
    }
    [[nodiscard]] std::size_t output_slot_count() const noexcept {
        return output_slots_.size();
    }

    [[nodiscard]] const ModelInputBufferView* input_buffer(
        std::size_t slot_index) const noexcept;
    [[nodiscard]] const std::vector<TensorView>* output_views(
        std::size_t slot_index) const noexcept;
    [[nodiscard]] const std::vector<int>* output_dma_fds(
        std::size_t slot_index) const noexcept;

    [[nodiscard]] bool Run(
        std::size_t input_slot_index,
        std::size_t output_slot_index,
        RknnRunTiming* timing = nullptr) noexcept;

private:
    struct InputSlot {
        // rknn_tensor_mem is either allocated by RKNN or is a wrapper around
        // external_buffer. The external DMA-BUF must outlive the RKNN wrapper.
        DmaHeapBuffer external_buffer;
        rknn_tensor_mem* memory = nullptr;
        ModelInputBufferView view;
    };

    struct OutputSlot {
        std::vector<rknn_tensor_mem*> memories;
        std::vector<rknn_output> descriptors;
        std::vector<TensorView> views;
        std::vector<int> dma_fds;
    };

    void QueryModel();
    void QueryNativeInput();
    void ParseInputShape();
    void ConfigureInputContract();
    void AllocateSlots();
    void DestroySlots() noexcept;

    [[nodiscard]] bool BindIo(
        InputSlot& input_slot,
        OutputSlot& output_slot,
        RknnRunTiming* timing) noexcept;
    [[nodiscard]] bool RunInputsSet(
        InputSlot& input_slot,
        OutputSlot& output_slot,
        RknnRunTiming* timing) noexcept;
    [[nodiscard]] bool RunBoundIo(
        InputSlot& input_slot,
        OutputSlot& output_slot,
        RknnRunTiming* timing) noexcept;

    static void CheckRknn(const char* operation, int result);

    rknn_context context_ = 0;
    bool initialized_ = false;
    bool native_direct_input_supported_ = false;

    RknnEngineConfig config_;
    RknnModelInfo model_info_;
    ImageShape input_shape_;

    rknn_tensor_attr selected_input_attr_{};
    std::vector<rknn_tensor_attr> selected_output_attrs_;
    uint32_t input_allocation_bytes_ = 0U;
    uint32_t input_row_stride_bytes_ = 0U;
    ModelInputMemoryLayout input_memory_layout_ =
        ModelInputMemoryLayout::RGB_UINT8_NHWC;

    std::vector<InputSlot> input_slots_;
    std::vector<OutputSlot> output_slots_;

    // rknn_set_io_mem bindings are context state. Avoid repeating ten bind
    // calls when the same input/output slot pair is reused. Alternating slots
    // still rebinds and the benchmark measures that cost explicitly.
    rknn_tensor_mem* bound_input_memory_ = nullptr;
    OutputSlot* bound_output_slot_ = nullptr;
};

}  // namespace visionarm
