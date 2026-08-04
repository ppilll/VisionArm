#pragma once

#include "common/pipeline_types.h"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace visionarm {

enum class CaptureConsumer : uint8_t {
    VIDEO_ENCODER = 0,
    INFERENCE = 1,
};

using CaptureConsumerMask = uint32_t;

constexpr CaptureConsumerMask CaptureConsumerBit(
    CaptureConsumer consumer) noexcept {
    return CaptureConsumerMask{1U} << static_cast<uint8_t>(consumer);
}

constexpr CaptureConsumerMask kVideoEncoderConsumer =
    CaptureConsumerBit(CaptureConsumer::VIDEO_ENCODER);
constexpr CaptureConsumerMask kInferenceConsumer =
    CaptureConsumerBit(CaptureConsumer::INFERENCE);
constexpr CaptureConsumerMask kAllCaptureConsumers =
    kVideoEncoderConsumer | kInferenceConsumer;

constexpr bool IsValidCaptureConsumerMask(
    CaptureConsumerMask mask) noexcept {
    return (mask & ~kAllCaptureConsumers) == 0U;
}

constexpr bool HasCaptureConsumer(
    CaptureConsumerMask mask,
    CaptureConsumer consumer) noexcept {
    return (mask & CaptureConsumerBit(consumer)) != 0U;
}

constexpr uint32_t CaptureConsumerCount(
    CaptureConsumerMask mask) noexcept {
    mask &= kAllCaptureConsumers;
    uint32_t count = 0U;
    while (mask != 0U) {
        count += mask & 1U;
        mask >>= 1U;
    }
    return count;
}

struct CaptureBufferKey {
    uint64_t capture_session_id = 0U;
    uint64_t frame_id = 0U;
    uint32_t buffer_index = 0U;
};

constexpr bool operator==(
    const CaptureBufferKey& lhs,
    const CaptureBufferKey& rhs) noexcept {
    return lhs.capture_session_id == rhs.capture_session_id &&
           lhs.frame_id == rhs.frame_id &&
           lhs.buffer_index == rhs.buffer_index;
}

constexpr bool operator!=(
    const CaptureBufferKey& lhs,
    const CaptureBufferKey& rhs) noexcept {
    return !(lhs == rhs);
}

constexpr CaptureBufferKey MakeCaptureBufferKey(
    const CaptureFrameView& frame) noexcept {
    return CaptureBufferKey{
        frame.identity.capture_session_id,
        frame.identity.frame_id,
        frame.buffer_index,
    };
}

enum class FrameReleaseReason : uint8_t {
    COMPLETED = 0,
    SKIPPED_NO_RESOURCE,
    REPLACED_BY_NEWER_FRAME,
    BRANCH_REJECTED,
    PIPELINE_STOP,
    PROCESSING_ERROR,
    LEASE_ABANDONED,
};

enum class CaptureFrameValidationError : uint8_t {
    NONE = 0,
    ZERO_DIMENSION,
    INVALID_PLANE_COUNT,
    NULL_MAPPED_ADDRESS,
    INVALID_DMA_FD,
    ZERO_ALLOCATION_LENGTH,
    DATA_RANGE_OUT_OF_BOUNDS,
};

struct CaptureFrameValidation {
    bool ok = false;
    CaptureFrameValidationError error =
        CaptureFrameValidationError::INVALID_PLANE_COUNT;
    uint32_t plane_index = 0U;
};

inline CaptureFrameValidation ValidateCaptureFrameView(
    const CaptureFrameView& frame,
    bool require_dmabuf) noexcept {

    if (frame.width <= 0 || frame.height <= 0) {
        return {false, CaptureFrameValidationError::ZERO_DIMENSION, 0U};
    }
    if (frame.plane_count == 0U ||
        frame.plane_count > static_cast<uint32_t>(kMaxFramePlanes)) {
        return {
            false,
            CaptureFrameValidationError::INVALID_PLANE_COUNT,
            0U,
        };
    }

    for (uint32_t plane_index = 0U;
         plane_index < frame.plane_count;
         ++plane_index) {
        const CapturePlaneView& plane = frame.planes[plane_index];

        if (plane.mapped_address == nullptr) {
            return {
                false,
                CaptureFrameValidationError::NULL_MAPPED_ADDRESS,
                plane_index,
            };
        }
        if (require_dmabuf && plane.dma_fd < 0) {
            return {
                false,
                CaptureFrameValidationError::INVALID_DMA_FD,
                plane_index,
            };
        }
        if (plane.allocation_length == 0U) {
            return {
                false,
                CaptureFrameValidationError::ZERO_ALLOCATION_LENGTH,
                plane_index,
            };
        }
        if (plane.data_offset > plane.allocation_length ||
            plane.bytes_used >
                plane.allocation_length - plane.data_offset) {
            return {
                false,
                CaptureFrameValidationError::DATA_RANGE_OUT_OF_BOUNDS,
                plane_index,
            };
        }
    }

    return {true, CaptureFrameValidationError::NONE, 0U};
}

struct RequeueRequest {
    CaptureBufferKey key;
    FrameIdentity identity;
};

constexpr RequeueRequest MakeRequeueRequest(
    const CaptureFrameView& frame) noexcept {
    return RequeueRequest{MakeCaptureBufferKey(frame), frame.identity};
}

struct CaptureBufferBrokerSnapshot {
    uint64_t accepted_frames = 0U;
    uint64_t rejected_frames = 0U;
    uint64_t invalid_publish_requests = 0U;
    uint64_t duplicate_buffer_publish_attempts = 0U;
    uint64_t leases_created = 0U;
    uint64_t leases_released = 0U;
    uint64_t duplicate_release_attempts = 0U;
    uint64_t stale_release_attempts = 0U;
    uint64_t requeue_requests_emitted = 0U;
    uint64_t requeue_requests_popped = 0U;
    uint64_t notifier_failures = 0U;
    uint64_t outstanding_frames = 0U;
    uint64_t outstanding_leases = 0U;
    uint64_t pending_requeue_requests = 0U;
    uint64_t high_watermark_frames = 0U;
    uint64_t high_watermark_leases = 0U;
    bool closed = false;
};

class IFrameLease {
public:
    virtual ~IFrameLease() = default;

    IFrameLease(const IFrameLease&) = delete;
    IFrameLease& operator=(const IFrameLease&) = delete;

    [[nodiscard]] virtual bool valid() const noexcept = 0;
    [[nodiscard]] virtual CaptureConsumer consumer() const noexcept = 0;
    [[nodiscard]] virtual CaptureBufferKey key() const noexcept = 0;
    [[nodiscard]] virtual const CaptureFrameView& frame() const = 0;

    virtual bool Release(FrameReleaseReason reason) noexcept = 0;

protected:
    IFrameLease() = default;
};

using FrameLeasePtr = std::unique_ptr<IFrameLease>;

struct CaptureDispatch {
    CaptureBufferKey key;
    FrameLeasePtr video_encoder;
    FrameLeasePtr inference;

    CaptureDispatch() = default;
    CaptureDispatch(const CaptureDispatch&) = delete;
    CaptureDispatch& operator=(const CaptureDispatch&) = delete;
    CaptureDispatch(CaptureDispatch&&) noexcept = default;
    CaptureDispatch& operator=(CaptureDispatch&&) noexcept = default;

    [[nodiscard]] uint32_t lease_count() const noexcept {
        return static_cast<uint32_t>(video_encoder != nullptr) +
               static_cast<uint32_t>(inference != nullptr);
    }
};

class ICaptureBufferBroker {
public:
    virtual ~ICaptureBufferBroker() = default;

    ICaptureBufferBroker(const ICaptureBufferBroker&) = delete;
    ICaptureBufferBroker& operator=(const ICaptureBufferBroker&) = delete;

    virtual bool Publish(
        CaptureFrameView frame,
        CaptureConsumerMask consumers,
        CaptureDispatch* dispatch) = 0;

    virtual bool TryPopRequeue(RequeueRequest* request) noexcept = 0;

    // Blocks until a completion is available, or until Close() has been called
    // and every accepted frame has reached the completion-drained state.
    virtual bool WaitPopRequeue(RequeueRequest* request) noexcept = 0;

    virtual void Close() noexcept = 0;

    [[nodiscard]] virtual CaptureBufferBrokerSnapshot GetSnapshot()
        const noexcept = 0;

protected:
    ICaptureBufferBroker() = default;
};

}  // namespace visionarm
