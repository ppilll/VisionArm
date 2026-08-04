#include "preprocess/rga_letterbox_preprocessor.h"

#include "preprocess/letterbox_geometry.h"

#include <limits>
#include <linux/videodev2.h>
#include <stdexcept>

namespace visionarm {
namespace {

[[nodiscard]] bool FitsInt(std::size_t value) noexcept {
    return value <= static_cast<std::size_t>(
        std::numeric_limits<int>::max());
}

[[nodiscard]] int PaddingColor(uint8_t value) noexcept {
    // imfill specifies the color from high to low as R, G, B, A.
    return static_cast<int>(value) |
           (static_cast<int>(value) << 8) |
           (static_cast<int>(value) << 16);
}

[[nodiscard]] bool RgaSucceeded(IM_STATUS status) noexcept {
    return status == IM_STATUS_SUCCESS || status == IM_STATUS_NOERROR;
}

}  // namespace

RgaLetterboxPreprocessor::RgaLetterboxPreprocessor(
    RgaLetterboxConfig config)
    : config_(config),
      source_buffers_(config.max_source_buffers),
      destination_buffers_(config.max_destination_slots) {

    if (config_.model_width <= 0 || config_.model_height <= 0 ||
        config_.max_source_buffers == 0U ||
        config_.max_destination_slots == 0U ||
        (config_.resize_policy.stretch_matching_source_aspect_ratio &&
         (config_.resize_policy.source_aspect_width <= 0 ||
          config_.resize_policy.source_aspect_height <= 0))) {
        throw std::invalid_argument("invalid RGA letterbox configuration");
    }
}

RgaLetterboxPreprocessor::~RgaLetterboxPreprocessor() {
    ReleaseAll();
}

bool RgaLetterboxPreprocessor::Validate(
    const CaptureFrameView& source,
    const ModelInputBufferView& destination) const noexcept {

    if (source.width <= 0 || source.height <= 0 ||
        (source.width % 2) != 0 || (source.height % 2) != 0 ||
        source.pixel_format != V4L2_PIX_FMT_NV12 ||
        source.plane_count != 1U ||
        source.buffer_index >= source_buffers_.size()) {
        return false;
    }

    const CapturePlaneView& plane = source.planes[0];
    const uint32_t source_stride =
        plane.stride != 0U
            ? plane.stride
            : static_cast<uint32_t>(source.width);
    const std::size_t source_required =
        static_cast<std::size_t>(source_stride) *
        static_cast<std::size_t>(source.height) * 3U / 2U;

    if (plane.dma_fd < 0 || plane.data_offset != 0U ||
        source_stride < static_cast<uint32_t>(source.width) ||
        plane.allocation_length < source_required ||
        !FitsInt(plane.allocation_length)) {
        return false;
    }

    if (destination.slot_index >= destination_buffers_.size() ||
        destination.dma_fd < 0 || destination.dma_offset != 0U ||
        destination.width != config_.model_width ||
        destination.height != config_.model_height ||
        destination.channels != 3 ||
        destination.memory_layout !=
            ModelInputMemoryLayout::RGB_UINT8_NHWC) {
        return false;
    }

    const uint32_t row_stride =
        destination.row_stride_bytes != 0U
            ? destination.row_stride_bytes
            : static_cast<uint32_t>(destination.width * 3);
    if ((row_stride % 3U) != 0U ||
        row_stride < static_cast<uint32_t>(destination.width * 3)) {
        return false;
    }

    const std::size_t destination_required =
        static_cast<std::size_t>(row_stride) *
        static_cast<std::size_t>(destination.height);
    return destination.capacity_bytes >= destination_required &&
           FitsInt(destination.capacity_bytes);
}

void RgaLetterboxPreprocessor::ReleaseImported(
    ImportedBuffer* imported) noexcept {
    if (imported == nullptr) {
        return;
    }
    if (imported->handle != 0U) {
        (void)releasebuffer_handle(imported->handle);
    }
    *imported = ImportedBuffer{};
}

bool RgaLetterboxPreprocessor::EnsureSourceImported(
    const CaptureFrameView& source,
    ImportedBuffer** imported) const noexcept {

    ImportedBuffer& entry = source_buffers_[source.buffer_index];
    const CapturePlaneView& plane = source.planes[0];
    const int width_stride = static_cast<int>(
        plane.stride != 0U
            ? plane.stride
            : static_cast<uint32_t>(source.width));

    const bool matches =
        entry.handle != 0U &&
        entry.fd == plane.dma_fd &&
        entry.size == plane.allocation_length &&
        entry.width == source.width &&
        entry.height == source.height &&
        entry.width_stride == width_stride &&
        entry.height_stride == source.height &&
        entry.format == RK_FORMAT_YCbCr_420_SP;
    if (matches) {
        *imported = &entry;
        return true;
    }

    ReleaseImported(&entry);
    const rga_buffer_handle_t handle = importbuffer_fd(
        plane.dma_fd,
        static_cast<int>(plane.allocation_length));
    if (handle == 0U) {
        return false;
    }

    entry.fd = plane.dma_fd;
    entry.size = plane.allocation_length;
    entry.width = source.width;
    entry.height = source.height;
    entry.width_stride = width_stride;
    entry.height_stride = source.height;
    entry.format = RK_FORMAT_YCbCr_420_SP;
    entry.handle = handle;
    ++snapshot_.source_imports;
    *imported = &entry;
    return true;
}

bool RgaLetterboxPreprocessor::EnsureDestinationImported(
    const ModelInputBufferView& destination,
    ImportedBuffer** imported) const noexcept {

    ImportedBuffer& entry = destination_buffers_[destination.slot_index];
    const uint32_t row_stride =
        destination.row_stride_bytes != 0U
            ? destination.row_stride_bytes
            : static_cast<uint32_t>(destination.width * 3);
    const int width_stride = static_cast<int>(row_stride / 3U);

    const bool matches =
        entry.handle != 0U &&
        entry.fd == destination.dma_fd &&
        entry.size == destination.capacity_bytes &&
        entry.width == destination.width &&
        entry.height == destination.height &&
        entry.width_stride == width_stride &&
        entry.height_stride == destination.height &&
        entry.format == RK_FORMAT_RGB_888;
    if (matches) {
        *imported = &entry;
        return true;
    }

    ReleaseImported(&entry);
    const rga_buffer_handle_t handle = importbuffer_fd(
        destination.dma_fd,
        static_cast<int>(destination.capacity_bytes));
    if (handle == 0U) {
        return false;
    }

    entry.fd = destination.dma_fd;
    entry.size = destination.capacity_bytes;
    entry.width = destination.width;
    entry.height = destination.height;
    entry.width_stride = width_stride;
    entry.height_stride = destination.height;
    entry.format = RK_FORMAT_RGB_888;
    entry.handle = handle;
    ++snapshot_.destination_imports;
    *imported = &entry;
    return true;
}

bool RgaLetterboxPreprocessor::Process(
    const CaptureFrameView& source,
    const ModelInputBufferView& destination,
    PreprocessTransform* transform) const {

    ++snapshot_.process_calls;
    if (transform == nullptr || !Validate(source, destination)) {
        ++snapshot_.validation_failures;
        return false;
    }

    LetterboxGeometry geometry;
    PreprocessTransform computed_transform;
    if (!ComputeModelResizeGeometry(
            source.width,
            source.height,
            destination.width,
            destination.height,
            config_.resize_policy,
            &geometry,
            &computed_transform)) {
        ++snapshot_.validation_failures;
        return false;
    }

    ImportedBuffer* source_import = nullptr;
    ImportedBuffer* destination_import = nullptr;
    if (!EnsureSourceImported(source, &source_import) ||
        !EnsureDestinationImported(destination, &destination_import)) {
        ++snapshot_.import_failures;
        return false;
    }

    rga_buffer_t source_buffer = wrapbuffer_handle(
        source_import->handle,
        source_import->width,
        source_import->height,
        source_import->format,
        source_import->width_stride,
        source_import->height_stride);
    rga_buffer_t destination_buffer = wrapbuffer_handle(
        destination_import->handle,
        destination_import->width,
        destination_import->height,
        destination_import->format,
        destination_import->width_stride,
        destination_import->height_stride);

    imsetColorSpace(
        &source_buffer,
        static_cast<IM_COLOR_SPACE_MODE>(config_.color_space_mode));

    const bool has_padding =
        geometry.pad_left != 0 || geometry.pad_top != 0 ||
        geometry.pad_right != 0 || geometry.pad_bottom != 0;
    if (has_padding) {
        const im_rect full_destination{
            0,
            0,
            destination.width,
            destination.height,
        };
        const IM_STATUS fill_status = imfill_t(
            destination_buffer,
            full_destination,
            PaddingColor(config_.padding_value),
            1);
        if (!RgaSucceeded(fill_status)) {
            ++snapshot_.fill_failures;
            return false;
        }
        ++snapshot_.fill_operations;
    }

    const im_rect source_rect{0, 0, source.width, source.height};
    const im_rect destination_rect{
        geometry.pad_left,
        geometry.pad_top,
        geometry.resized_width,
        geometry.resized_height,
    };
    const im_rect empty_rect{};
    rga_buffer_t empty_buffer{};
    im_opt_t options{};

    const IM_STATUS process_status = improcess(
        source_buffer,
        destination_buffer,
        empty_buffer,
        source_rect,
        destination_rect,
        empty_rect,
        -1,
        nullptr,
        &options,
        IM_SYNC);
    if (!RgaSucceeded(process_status)) {
        ++snapshot_.process_failures;
        return false;
    }

    *transform = computed_transform;
    ++snapshot_.process_successes;
    if (computed_transform.letterbox) {
        ++snapshot_.letterbox_successes;
    } else {
        ++snapshot_.direct_resize_successes;
    }
    return true;
}

RgaPreprocessorSnapshot RgaLetterboxPreprocessor::snapshot() const noexcept {
    return snapshot_;
}

void RgaLetterboxPreprocessor::ReleaseAll() noexcept {
    for (ImportedBuffer& imported : source_buffers_) {
        ReleaseImported(&imported);
    }
    for (ImportedBuffer& imported : destination_buffers_) {
        ReleaseImported(&imported);
    }
}

}  // namespace visionarm
