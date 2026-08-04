#pragma once

#include "camera/capture_buffer_contract.h"

#include <cstddef>
#include <functional>
#include <memory>

namespace visionarm {

struct CaptureBufferBrokerConfig {
    // Must equal or exceed the number of V4L2 buffers that may be DQBUF at
    // once. Buffer index values must be in [0, max_buffer_count).
    std::size_t max_buffer_count = 0U;

    bool require_dmabuf = true;

    // Called after a requeue completion is emitted and after Close(). The
    // callback must be non-blocking; exceptions are caught and counted.
    std::function<void()> requeue_ready_notifier;
};

class CaptureBufferBroker final : public ICaptureBufferBroker {
public:
    explicit CaptureBufferBroker(CaptureBufferBrokerConfig config);
    ~CaptureBufferBroker() override;

    CaptureBufferBroker(const CaptureBufferBroker&) = delete;
    CaptureBufferBroker& operator=(const CaptureBufferBroker&) = delete;

    bool Publish(
        CaptureFrameView frame,
        CaptureConsumerMask consumers,
        CaptureDispatch* dispatch) override;

    bool TryPopRequeue(RequeueRequest* request) noexcept override;
    bool WaitPopRequeue(RequeueRequest* request) noexcept override;
    void Close() noexcept override;

    [[nodiscard]] CaptureBufferBrokerSnapshot GetSnapshot()
        const noexcept override;

private:
    struct Core;
    class FrameLeaseImpl;
    std::shared_ptr<Core> core_;
};

}  // namespace visionarm
