#pragma once

#include "camera/capture_buffer_contract.h"

#include <cstddef>

namespace visionarm {

enum class CaptureResult {
    FRAME,
    TIMEOUT,
    DROPPED,
    WAKE,
    STOPPED,
};

class ICameraSource {
public:
    virtual ~ICameraSource() = default;

    virtual void Open() = 0;
    virtual void Start() = 0;

    virtual CaptureResult Capture(CaptureFrameView* frame) = 0;

    // Only the camera owner thread may call this function. The implementation
    // must validate the full lifecycle key, not only the reusable buffer index.
    virtual bool Requeue(const RequeueRequest& request) noexcept = 0;

    // Wakes a blocking Capture() without requesting shutdown. Used when a
    // broker completion becomes available and the owner must perform QBUF.
    virtual void Wake() noexcept = 0;

    virtual void RequestStop() noexcept = 0;
    virtual void Stop() noexcept = 0;

    [[nodiscard]] virtual std::size_t buffer_count() const noexcept = 0;
};

}  // namespace visionarm
