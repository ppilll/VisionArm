#include "preprocess/nv12_letterbox_preprocessor.h"

#include "preprocess/letterbox_geometry.h"

#include <cstddef>
#include <cstdint>
#include <linux/videodev2.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace visionarm {
namespace {

struct Nv12Planes {
    const uint8_t* y = nullptr;
    const uint8_t* uv = nullptr;
    std::size_t y_stride = 0;
    std::size_t uv_stride = 0;
};

[[nodiscard]] Nv12Planes ResolveNv12Planes(const FramePacket& frame) {
    if (frame.width <= 0 || frame.height <= 0 ||
        (frame.width % 2) != 0 || (frame.height % 2) != 0 ||
        frame.plane_count == 0U ||
        frame.plane_count > kMaxFramePlanes) {
        throw std::invalid_argument("invalid NV12 frame packet");
    }

    const bool supported =
        frame.pixel_format == V4L2_PIX_FMT_NV12
#ifdef V4L2_PIX_FMT_NV12M
        || frame.pixel_format == V4L2_PIX_FMT_NV12M
#endif
        ;
    if (!supported) {
        throw std::invalid_argument("preprocessor requires NV12 or NV12M");
    }

    if (frame.plane_count == 1U) {
        const FramePlane& plane = frame.planes[0];
        if (plane.mapped_address == nullptr) {
            throw std::invalid_argument("NV12 plane has no CPU mapping");
        }

        const std::size_t y_stride = plane.stride != 0U
            ? plane.stride
            : static_cast<std::size_t>(frame.width);
        const std::size_t uv_stride = y_stride;
        const std::size_t y_bytes =
            y_stride * static_cast<std::size_t>(frame.height);
        const std::size_t uv_bytes =
            uv_stride * static_cast<std::size_t>(frame.height / 2);

        const std::size_t available =
            plane.allocation_length > plane.data_offset
                ? plane.allocation_length - plane.data_offset
                : 0U;
        if (y_bytes + uv_bytes > available) {
            throw std::invalid_argument(
                "single-plane NV12 allocation is smaller than negotiated layout");
        }

        const auto* y = static_cast<const uint8_t*>(plane.mapped_address);
        return Nv12Planes{
            y,
            y + y_bytes,
            y_stride,
            uv_stride,
        };
    }

    const FramePlane& y_plane = frame.planes[0];
    const FramePlane& uv_plane = frame.planes[1];
    if (y_plane.mapped_address == nullptr ||
        uv_plane.mapped_address == nullptr) {
        throw std::invalid_argument("NV12M plane has no CPU mapping");
    }

    const std::size_t y_stride = y_plane.stride != 0U
        ? y_plane.stride
        : static_cast<std::size_t>(frame.width);
    const std::size_t uv_stride = uv_plane.stride != 0U
        ? uv_plane.stride
        : static_cast<std::size_t>(frame.width);

    const std::size_t y_required =
        y_stride * static_cast<std::size_t>(frame.height);
    const std::size_t uv_required =
        uv_stride * static_cast<std::size_t>(frame.height / 2);

    const std::size_t y_available =
        y_plane.allocation_length > y_plane.data_offset
            ? y_plane.allocation_length - y_plane.data_offset
            : 0U;
    const std::size_t uv_available =
        uv_plane.allocation_length > uv_plane.data_offset
            ? uv_plane.allocation_length - uv_plane.data_offset
            : 0U;

    if (y_required > y_available || uv_required > uv_available) {
        throw std::invalid_argument(
            "multi-plane NV12 allocation is smaller than negotiated layout");
    }

    return Nv12Planes{
        static_cast<const uint8_t*>(y_plane.mapped_address),
        static_cast<const uint8_t*>(uv_plane.mapped_address),
        y_stride,
        uv_stride,
    };
}

}  // namespace

Nv12LetterboxPreprocessor::Nv12LetterboxPreprocessor(
    Nv12LetterboxConfig config)
    : config_(config) {

    if (config_.model_width <= 0 || config_.model_height <= 0 ||
        (config_.resize_policy.stretch_matching_source_aspect_ratio &&
         (config_.resize_policy.source_aspect_width <= 0 ||
          config_.resize_policy.source_aspect_height <= 0))) {
        throw std::invalid_argument("invalid NV12 resize configuration");
    }
}

bool Nv12LetterboxPreprocessor::Process(
    const FramePacket& frame,
    const ModelInputBufferView& destination,
    PreprocessTransform* transform) const {

    if (transform == nullptr || destination.cpu_address == nullptr ||
        destination.width != config_.model_width ||
        destination.height != config_.model_height ||
        destination.channels != 3 ||
        destination.memory_layout !=
            ModelInputMemoryLayout::RGB_UINT8_NHWC) {
        return false;
    }

    const uint32_t row_stride = destination.row_stride_bytes != 0U
        ? destination.row_stride_bytes
        : static_cast<uint32_t>(destination.width * destination.channels);

    const std::size_t required_bytes =
        static_cast<std::size_t>(row_stride) *
        static_cast<std::size_t>(destination.height);
    if (required_bytes > destination.capacity_bytes) {
        return false;
    }

    try {
        const Nv12Planes planes = ResolveNv12Planes(frame);

        cv::Mat y_plane(
            frame.height,
            frame.width,
            CV_8UC1,
            const_cast<uint8_t*>(planes.y),
            planes.y_stride);

        cv::Mat uv_plane(
            frame.height / 2,
            frame.width / 2,
            CV_8UC2,
            const_cast<uint8_t*>(planes.uv),
            planes.uv_stride);

        cv::Mat source_rgb;
        cv::cvtColorTwoPlane(
            y_plane,
            uv_plane,
            source_rgb,
            cv::COLOR_YUV2RGB_NV12);

        LetterboxGeometry geometry;
        PreprocessTransform computed_transform;
        if (!ComputeModelResizeGeometry(
                frame.width,
                frame.height,
                config_.model_width,
                config_.model_height,
                config_.resize_policy,
                &geometry,
                &computed_transform)) {
            return false;
        }

        cv::Mat model_rgb(
            destination.height,
            destination.width,
            CV_8UC3,
            destination.cpu_address,
            row_stride);

        if (!computed_transform.letterbox) {
            cv::resize(
                source_rgb,
                model_rgb,
                cv::Size(destination.width, destination.height),
                0.0,
                0.0,
                cv::INTER_LINEAR);
        } else {
            cv::Mat resized_rgb;
            cv::resize(
                source_rgb,
                resized_rgb,
                cv::Size(geometry.resized_width, geometry.resized_height),
                0.0,
                0.0,
                cv::INTER_LINEAR);

            model_rgb.setTo(cv::Scalar(
                config_.padding_value,
                config_.padding_value,
                config_.padding_value));
            resized_rgb.copyTo(model_rgb(cv::Rect(
                geometry.pad_left,
                geometry.pad_top,
                geometry.resized_width,
                geometry.resized_height)));
        }

        *transform = computed_transform;
        return true;
    } catch (const cv::Exception&) {
        return false;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace visionarm
