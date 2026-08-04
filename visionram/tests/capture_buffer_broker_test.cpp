#include "camera/capture_buffer_broker.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

namespace {

using visionarm::CaptureBufferBroker;
using visionarm::CaptureBufferBrokerConfig;
using visionarm::CaptureDispatch;
using visionarm::CaptureFrameView;
using visionarm::FrameReleaseReason;
using visionarm::RequeueRequest;

#define CHECK_TRUE(expr)                                                     \
    do {                                                                     \
        if (!(expr)) {                                                       \
            std::cerr << "CHECK failed at " << __FILE__ << ':' << __LINE__   \
                      << ": " #expr "\n";                                  \
            return false;                                                    \
        }                                                                    \
    } while (false)

CaptureFrameView MakeFrame(
    uint32_t buffer_index,
    uint64_t frame_id,
    std::array<uint8_t, 64U>* storage) {
    CaptureFrameView frame;
    frame.identity.capture_session_id = 7U;
    frame.identity.frame_id = frame_id;
    frame.identity.v4l2_sequence = static_cast<uint32_t>(frame_id);
    frame.buffer_index = buffer_index;
    frame.width = 4;
    frame.height = 4;
    frame.pixel_format = 1U;
    frame.plane_count = 1U;
    frame.planes[0].mapped_address = storage->data();
    frame.planes[0].dma_fd = -1;
    frame.planes[0].bytes_used = storage->size();
    frame.planes[0].allocation_length = storage->size();
    frame.planes[0].stride = 4U;
    frame.planes[0].size_image = static_cast<uint32_t>(storage->size());
    return frame;
}

bool TestSingleConsumer() {
    std::array<uint8_t, 64U> storage{};
    CaptureBufferBroker broker({2U, false, {}});
    CaptureDispatch dispatch;
    CHECK_TRUE(broker.Publish(
        MakeFrame(0U, 1U, &storage),
        visionarm::kInferenceConsumer,
        &dispatch));
    CHECK_TRUE(dispatch.inference != nullptr);
    CHECK_TRUE(dispatch.video_encoder == nullptr);
    CHECK_TRUE(dispatch.inference->valid());
    CHECK_TRUE(dispatch.inference->frame().buffer_index == 0U);
    CHECK_TRUE(!broker.TryPopRequeue(nullptr));
    CHECK_TRUE(dispatch.inference->Release(FrameReleaseReason::COMPLETED));
    CHECK_TRUE(!dispatch.inference->valid());

    RequeueRequest request;
    CHECK_TRUE(broker.TryPopRequeue(&request));
    CHECK_TRUE(request.key.buffer_index == 0U);
    CHECK_TRUE(!broker.TryPopRequeue(&request));

    const auto snapshot = broker.GetSnapshot();
    CHECK_TRUE(snapshot.accepted_frames == 1U);
    CHECK_TRUE(snapshot.leases_created == 1U);
    CHECK_TRUE(snapshot.leases_released == 1U);
    CHECK_TRUE(snapshot.outstanding_frames == 0U);
    CHECK_TRUE(snapshot.outstanding_leases == 0U);
    return true;
}

bool TestTwoConsumersArbitraryOrder() {
    std::array<uint8_t, 64U> storage{};
    CaptureBufferBroker broker({2U, false, {}});
    CaptureDispatch dispatch;
    CHECK_TRUE(broker.Publish(
        MakeFrame(1U, 2U, &storage),
        visionarm::kAllCaptureConsumers,
        &dispatch));
    CHECK_TRUE(dispatch.lease_count() == 2U);

    CHECK_TRUE(dispatch.inference->Release(
        FrameReleaseReason::SKIPPED_NO_RESOURCE));
    RequeueRequest request;
    CHECK_TRUE(!broker.TryPopRequeue(&request));
    CHECK_TRUE(dispatch.video_encoder->Release(FrameReleaseReason::COMPLETED));
    CHECK_TRUE(broker.TryPopRequeue(&request));
    CHECK_TRUE(request.key.frame_id == 2U);
    return true;
}

bool TestDestructorAbandonsLease() {
    std::array<uint8_t, 64U> storage{};
    CaptureBufferBroker broker({1U, false, {}});
    {
        CaptureDispatch dispatch;
        CHECK_TRUE(broker.Publish(
            MakeFrame(0U, 3U, &storage),
            visionarm::kInferenceConsumer,
            &dispatch));
    }

    RequeueRequest request;
    CHECK_TRUE(broker.TryPopRequeue(&request));
    CHECK_TRUE(broker.GetSnapshot().leases_released == 1U);
    return true;
}

bool TestDuplicateRelease() {
    std::array<uint8_t, 64U> storage{};
    CaptureBufferBroker broker({1U, false, {}});
    CaptureDispatch dispatch;
    CHECK_TRUE(broker.Publish(
        MakeFrame(0U, 4U, &storage),
        visionarm::kInferenceConsumer,
        &dispatch));
    CHECK_TRUE(dispatch.inference->Release(FrameReleaseReason::COMPLETED));
    CHECK_TRUE(!dispatch.inference->Release(FrameReleaseReason::COMPLETED));
    CHECK_TRUE(broker.GetSnapshot().duplicate_release_attempts == 1U);
    return true;
}

bool TestDuplicateBufferPublishRejectedUntilCompletionPopped() {
    std::array<uint8_t, 64U> storage_a{};
    std::array<uint8_t, 64U> storage_b{};
    CaptureBufferBroker broker({1U, false, {}});
    CaptureDispatch first;
    CaptureDispatch second;
    CHECK_TRUE(broker.Publish(
        MakeFrame(0U, 5U, &storage_a),
        visionarm::kInferenceConsumer,
        &first));
    CHECK_TRUE(!broker.Publish(
        MakeFrame(0U, 6U, &storage_b),
        visionarm::kInferenceConsumer,
        &second));
    CHECK_TRUE(first.inference->Release(FrameReleaseReason::COMPLETED));
    CHECK_TRUE(!broker.Publish(
        MakeFrame(0U, 6U, &storage_b),
        visionarm::kInferenceConsumer,
        &second));

    RequeueRequest request;
    CHECK_TRUE(broker.TryPopRequeue(&request));
    CHECK_TRUE(broker.Publish(
        MakeFrame(0U, 6U, &storage_b),
        visionarm::kInferenceConsumer,
        &second));
    CHECK_TRUE(second.inference->Release(FrameReleaseReason::COMPLETED));
    CHECK_TRUE(broker.TryPopRequeue(&request));
    return true;
}

bool TestCloseRejectsPublishAndWaitTerminates() {
    std::array<uint8_t, 64U> storage{};
    CaptureBufferBroker broker({1U, false, {}});
    broker.Close();
    CaptureDispatch dispatch;
    CHECK_TRUE(!broker.Publish(
        MakeFrame(0U, 7U, &storage),
        visionarm::kInferenceConsumer,
        &dispatch));
    RequeueRequest request;
    CHECK_TRUE(!broker.WaitPopRequeue(&request));
    return true;
}

bool TestWaitWakesOnFinalRelease() {
    std::array<uint8_t, 64U> storage{};
    CaptureBufferBroker broker({1U, false, {}});
    CaptureDispatch dispatch;
    CHECK_TRUE(broker.Publish(
        MakeFrame(0U, 8U, &storage),
        visionarm::kAllCaptureConsumers,
        &dispatch));

    auto waiter = std::async(std::launch::async, [&broker] {
        RequeueRequest request;
        return broker.WaitPopRequeue(&request) && request.key.frame_id == 8U;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    CHECK_TRUE(dispatch.video_encoder->Release(FrameReleaseReason::COMPLETED));
    CHECK_TRUE(waiter.wait_for(std::chrono::milliseconds(20)) ==
               std::future_status::timeout);
    CHECK_TRUE(dispatch.inference->Release(FrameReleaseReason::COMPLETED));
    CHECK_TRUE(waiter.get());
    return true;
}

bool TestConcurrentReleaseAndNotifier() {
    std::array<uint8_t, 64U> storage{};
    std::atomic<uint32_t> notifications{0U};
    CaptureBufferBrokerConfig config;
    config.max_buffer_count = 1U;
    config.require_dmabuf = false;
    config.requeue_ready_notifier = [&notifications] {
        notifications.fetch_add(1U, std::memory_order_relaxed);
    };
    CaptureBufferBroker broker(std::move(config));
    CaptureDispatch dispatch;
    CHECK_TRUE(broker.Publish(
        MakeFrame(0U, 9U, &storage),
        visionarm::kAllCaptureConsumers,
        &dispatch));

    auto* video = dispatch.video_encoder.get();
    auto* inference = dispatch.inference.get();
    std::thread thread_a([video] {
        (void)video->Release(FrameReleaseReason::COMPLETED);
    });
    std::thread thread_b([inference] {
        (void)inference->Release(FrameReleaseReason::COMPLETED);
    });
    thread_a.join();
    thread_b.join();

    RequeueRequest request;
    CHECK_TRUE(broker.TryPopRequeue(&request));
    CHECK_TRUE(notifications.load(std::memory_order_relaxed) == 1U);
    return true;
}

bool TestInvalidPublish() {
    std::array<uint8_t, 64U> storage{};
    CaptureBufferBroker broker({1U, false, {}});
    CaptureDispatch dispatch;
    CHECK_TRUE(!broker.Publish(
        MakeFrame(0U, 10U, &storage),
        0U,
        &dispatch));
    CHECK_TRUE(!broker.Publish(
        MakeFrame(2U, 11U, &storage),
        visionarm::kInferenceConsumer,
        &dispatch));
    CHECK_TRUE(broker.GetSnapshot().invalid_publish_requests == 2U);
    return true;
}

}  // namespace

int main() {
    const bool ok =
        TestSingleConsumer() &&
        TestTwoConsumersArbitraryOrder() &&
        TestDestructorAbandonsLease() &&
        TestDuplicateRelease() &&
        TestDuplicateBufferPublishRejectedUntilCompletionPopped() &&
        TestCloseRejectsPublishAndWaitTerminates() &&
        TestWaitWakesOnFinalRelease() &&
        TestConcurrentReleaseAndNotifier() &&
        TestInvalidPublish();

    if (!ok) {
        return EXIT_FAILURE;
    }
    std::cout << "capture_buffer_broker_test PASSED\n";
    return EXIT_SUCCESS;
}
