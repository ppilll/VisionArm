#include "camera/capture_buffer_broker.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace visionarm {
namespace {

constexpr std::size_t ConsumerIndex(CaptureConsumer consumer) noexcept {
    return static_cast<std::size_t>(consumer);
}

}  // namespace

struct CaptureBufferBroker::Core final {
    struct Slot {
        bool occupied = false;
        bool completion_queued = false;
        CaptureBufferKey key;
        CaptureFrameView frame;
        CaptureConsumerMask requested_consumers = 0U;
        CaptureConsumerMask released_consumers = 0U;
        uint32_t remaining_leases = 0U;
        std::array<FrameReleaseReason, 2U> release_reasons{
            FrameReleaseReason::LEASE_ABANDONED,
            FrameReleaseReason::LEASE_ABANDONED,
        };
    };

    explicit Core(CaptureBufferBrokerConfig input_config)
        : config(std::move(input_config)),
          slots(config.max_buffer_count),
          completions(config.max_buffer_count) {}

    bool ReleaseLease(
        uint32_t buffer_index,
        CaptureBufferKey key,
        CaptureConsumer consumer,
        FrameReleaseReason reason) noexcept {

        bool notify_external = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (buffer_index >= slots.size()) {
                ++snapshot.stale_release_attempts;
                return false;
            }

            Slot& slot = slots[buffer_index];
            const CaptureConsumerMask bit = CaptureConsumerBit(consumer);
            if (!slot.occupied || slot.key != key ||
                (slot.requested_consumers & bit) == 0U) {
                ++snapshot.stale_release_attempts;
                return false;
            }
            if ((slot.released_consumers & bit) != 0U) {
                ++snapshot.duplicate_release_attempts;
                return false;
            }

            slot.released_consumers |= bit;
            slot.release_reasons[ConsumerIndex(consumer)] = reason;
            --slot.remaining_leases;
            ++snapshot.leases_released;
            --snapshot.outstanding_leases;

            if (slot.remaining_leases == 0U) {
                if (completion_size >= completions.size()) {
                    // This cannot occur while one slot remains occupied until
                    // its completion is popped. Keep the slot occupied rather
                    // than silently losing the only legal QBUF completion.
                    ++snapshot.stale_release_attempts;
                    return false;
                }

                completions[completion_tail] = RequeueRequest{
                    slot.key,
                    slot.frame.identity,
                };
                completion_tail = NextCompletion(completion_tail);
                ++completion_size;
                slot.completion_queued = true;
                ++snapshot.requeue_requests_emitted;
                snapshot.pending_requeue_requests = completion_size;
                notify_external = true;
                completion_cv.notify_all();
            }
        }

        if (notify_external) {
            NotifyExternal();
        }
        return true;
    }

    void RecordDuplicateRelease() noexcept {
        std::lock_guard<std::mutex> lock(mutex);
        ++snapshot.duplicate_release_attempts;
    }

    const CaptureFrameView& Frame(
        uint32_t buffer_index,
        CaptureBufferKey key) const {
        std::lock_guard<std::mutex> lock(mutex);
        if (buffer_index >= slots.size()) {
            throw std::logic_error("frame lease buffer index is invalid");
        }
        const Slot& slot = slots[buffer_index];
        if (!slot.occupied || slot.key != key) {
            throw std::logic_error("frame lease is stale");
        }
        return slot.frame;
    }

    bool PopLocked(RequeueRequest* request) noexcept {
        if (request == nullptr || completion_size == 0U) {
            return false;
        }

        std::optional<RequeueRequest>& entry = completions[completion_head];
        if (!entry.has_value()) {
            return false;
        }

        *request = *entry;
        entry.reset();
        completion_head = NextCompletion(completion_head);
        --completion_size;

        const uint32_t buffer_index = request->key.buffer_index;
        if (buffer_index < slots.size()) {
            Slot& slot = slots[buffer_index];
            if (slot.occupied && slot.completion_queued &&
                slot.key == request->key) {
                slot = {};
                --snapshot.outstanding_frames;
            }
        }

        ++snapshot.requeue_requests_popped;
        snapshot.pending_requeue_requests = completion_size;
        completion_cv.notify_all();
        return true;
    }

    std::size_t NextCompletion(std::size_t index) const noexcept {
        return (index + 1U) % completions.size();
    }

    void NotifyExternal() noexcept {
        if (!config.requeue_ready_notifier) {
            return;
        }
        try {
            config.requeue_ready_notifier();
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex);
            ++snapshot.notifier_failures;
        }
    }

    CaptureBufferBrokerConfig config;
    mutable std::mutex mutex;
    std::condition_variable completion_cv;
    std::vector<Slot> slots;
    std::vector<std::optional<RequeueRequest>> completions;
    std::size_t completion_head = 0U;
    std::size_t completion_tail = 0U;
    std::size_t completion_size = 0U;
    CaptureBufferBrokerSnapshot snapshot;
};

class CaptureBufferBroker::FrameLeaseImpl final : public IFrameLease {
public:
    FrameLeaseImpl(
        std::shared_ptr<CaptureBufferBroker::Core> core,
        uint32_t buffer_index,
        CaptureBufferKey key,
        CaptureConsumer consumer)
        : core_(std::move(core)),
          buffer_index_(buffer_index),
          key_(key),
          consumer_(consumer) {}

    ~FrameLeaseImpl() override {
        if (armed_.load(std::memory_order_acquire) &&
            !released_.load(std::memory_order_acquire)) {
            (void)Release(FrameReleaseReason::LEASE_ABANDONED);
        }
    }

    void Arm() noexcept {
        armed_.store(true, std::memory_order_release);
    }

    [[nodiscard]] bool valid() const noexcept override {
        return armed_.load(std::memory_order_acquire) &&
               !released_.load(std::memory_order_acquire);
    }

    [[nodiscard]] CaptureConsumer consumer() const noexcept override {
        return consumer_;
    }

    [[nodiscard]] CaptureBufferKey key() const noexcept override {
        return key_;
    }

    [[nodiscard]] const CaptureFrameView& frame() const override {
        if (!valid()) {
            throw std::logic_error("frame() called on an invalid lease");
        }
        return core_->Frame(buffer_index_, key_);
    }

    bool Release(FrameReleaseReason reason) noexcept override {
        if (!armed_.load(std::memory_order_acquire)) {
            return false;
        }

        bool expected = false;
        if (!released_.compare_exchange_strong(
                expected,
                true,
                std::memory_order_acq_rel,
                std::memory_order_acquire)) {
            core_->RecordDuplicateRelease();
            return false;
        }

        return core_->ReleaseLease(
            buffer_index_,
            key_,
            consumer_,
            reason);
    }

private:
    std::shared_ptr<CaptureBufferBroker::Core> core_;
    uint32_t buffer_index_ = 0U;
    CaptureBufferKey key_;
    CaptureConsumer consumer_ = CaptureConsumer::INFERENCE;
    std::atomic<bool> armed_{false};
    std::atomic<bool> released_{false};
};


CaptureBufferBroker::CaptureBufferBroker(CaptureBufferBrokerConfig config) {
    if (config.max_buffer_count == 0U) {
        throw std::invalid_argument(
            "CaptureBufferBroker max_buffer_count must be > 0");
    }
    core_ = std::make_shared<Core>(std::move(config));
}

CaptureBufferBroker::~CaptureBufferBroker() {
    Close();
}

bool CaptureBufferBroker::Publish(
    CaptureFrameView frame,
    CaptureConsumerMask consumers,
    CaptureDispatch* dispatch) {

    if (dispatch == nullptr) {
        std::lock_guard<std::mutex> lock(core_->mutex);
        ++core_->snapshot.rejected_frames;
        ++core_->snapshot.invalid_publish_requests;
        return false;
    }

    // Destroy any caller-owned previous dispatch before taking the core lock.
    *dispatch = CaptureDispatch{};

    const CaptureFrameValidation validation = ValidateCaptureFrameView(
        frame,
        core_->config.require_dmabuf);
    if (!validation.ok || consumers == 0U ||
        !IsValidCaptureConsumerMask(consumers) ||
        frame.buffer_index >= core_->slots.size()) {
        std::lock_guard<std::mutex> lock(core_->mutex);
        ++core_->snapshot.rejected_frames;
        ++core_->snapshot.invalid_publish_requests;
        return false;
    }

    const CaptureBufferKey key = MakeCaptureBufferKey(frame);
    std::unique_ptr<FrameLeaseImpl> video_lease;
    std::unique_ptr<FrameLeaseImpl> inference_lease;

    std::lock_guard<std::mutex> lock(core_->mutex);
    if (core_->snapshot.closed) {
        ++core_->snapshot.rejected_frames;
        return false;
    }

    Core::Slot& slot = core_->slots[frame.buffer_index];
    if (slot.occupied) {
        ++core_->snapshot.rejected_frames;
        ++core_->snapshot.duplicate_buffer_publish_attempts;
        return false;
    }

    try {
        if (HasCaptureConsumer(consumers, CaptureConsumer::VIDEO_ENCODER)) {
            video_lease = std::make_unique<FrameLeaseImpl>(
                core_,
                frame.buffer_index,
                key,
                CaptureConsumer::VIDEO_ENCODER);
        }
        if (HasCaptureConsumer(consumers, CaptureConsumer::INFERENCE)) {
            inference_lease = std::make_unique<FrameLeaseImpl>(
                core_,
                frame.buffer_index,
                key,
                CaptureConsumer::INFERENCE);
        }
    } catch (...) {
        ++core_->snapshot.rejected_frames;
        return false;
    }

    slot.occupied = true;
    slot.completion_queued = false;
    slot.key = key;
    slot.frame = frame;
    slot.requested_consumers = consumers;
    slot.released_consumers = 0U;
    slot.remaining_leases = CaptureConsumerCount(consumers);

    if (video_lease) {
        video_lease->Arm();
    }
    if (inference_lease) {
        inference_lease->Arm();
    }

    dispatch->key = key;
    dispatch->video_encoder = std::move(video_lease);
    dispatch->inference = std::move(inference_lease);

    ++core_->snapshot.accepted_frames;
    ++core_->snapshot.outstanding_frames;
    core_->snapshot.leases_created += dispatch->lease_count();
    core_->snapshot.outstanding_leases += dispatch->lease_count();
    core_->snapshot.high_watermark_frames = std::max(
        core_->snapshot.high_watermark_frames,
        core_->snapshot.outstanding_frames);
    core_->snapshot.high_watermark_leases = std::max(
        core_->snapshot.high_watermark_leases,
        core_->snapshot.outstanding_leases);
    return true;
}

bool CaptureBufferBroker::TryPopRequeue(
    RequeueRequest* request) noexcept {
    std::lock_guard<std::mutex> lock(core_->mutex);
    return core_->PopLocked(request);
}

bool CaptureBufferBroker::WaitPopRequeue(
    RequeueRequest* request) noexcept {
    if (request == nullptr) {
        return false;
    }

    std::unique_lock<std::mutex> lock(core_->mutex);
    core_->completion_cv.wait(lock, [this] {
        return core_->completion_size > 0U ||
               (core_->snapshot.closed &&
                core_->snapshot.outstanding_frames == 0U);
    });

    return core_->PopLocked(request);
}

void CaptureBufferBroker::Close() noexcept {
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(core_->mutex);
        if (!core_->snapshot.closed) {
            core_->snapshot.closed = true;
            changed = true;
        }
        core_->completion_cv.notify_all();
    }
    if (changed) {
        core_->NotifyExternal();
    }
}

CaptureBufferBrokerSnapshot CaptureBufferBroker::GetSnapshot()
    const noexcept {
    std::lock_guard<std::mutex> lock(core_->mutex);
    return core_->snapshot;
}

}  // namespace visionarm
