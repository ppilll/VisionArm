#include "camera/capture_buffer_contract.h"

#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <type_traits>

namespace {

using visionarm::CaptureBufferKey;
using visionarm::CaptureConsumer;
using visionarm::CaptureConsumerBit;
using visionarm::CaptureConsumerCount;
using visionarm::CaptureFrameValidationError;
using visionarm::CaptureFrameView;
using visionarm::HasCaptureConsumer;
using visionarm::IsValidCaptureConsumerMask;
using visionarm::MakeCaptureBufferKey;
using visionarm::ValidateCaptureFrameView;

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

CaptureFrameView MakeValidFrame() {
    static uint8_t storage[4096]{};

    CaptureFrameView frame;
    frame.identity.capture_session_id = 7U;
    frame.identity.frame_id = 100U;
    frame.buffer_index = 3U;
    frame.width = 1280;
    frame.height = 720;
    frame.pixel_format = 0x3231564EU;  // NV12, no Linux header dependency.
    frame.plane_count = 1U;
    frame.planes[0].mapped_address = storage;
    frame.planes[0].dma_fd = 11;
    frame.planes[0].data_offset = 64U;
    frame.planes[0].bytes_used = 1024U;
    frame.planes[0].allocation_length = sizeof(storage);
    frame.planes[0].stride = 1280U;
    frame.planes[0].size_image = 1382400U;
    return frame;
}

void TestConsumerMask() {
    const auto video = CaptureConsumerBit(CaptureConsumer::VIDEO_ENCODER);
    const auto inference = CaptureConsumerBit(CaptureConsumer::INFERENCE);
    const auto both = video | inference;

    Require(IsValidCaptureConsumerMask(0U), "empty mask must be valid");
    Require(IsValidCaptureConsumerMask(both), "known mask must be valid");
    Require(!IsValidCaptureConsumerMask(1U << 8U),
            "unknown consumer bit must be invalid");
    Require(HasCaptureConsumer(both, CaptureConsumer::VIDEO_ENCODER),
            "video bit missing");
    Require(HasCaptureConsumer(both, CaptureConsumer::INFERENCE),
            "inference bit missing");
    Require(CaptureConsumerCount(both) == 2U,
            "consumer count must be two");
}

void TestBufferKeyIncludesGeneration() {
    CaptureFrameView first = MakeValidFrame();
    CaptureFrameView second = first;
    second.identity.frame_id = first.identity.frame_id + 1U;

    const CaptureBufferKey first_key = MakeCaptureBufferKey(first);
    const CaptureBufferKey second_key = MakeCaptureBufferKey(second);

    Require(first_key != second_key,
            "reused V4L2 index must have a different lifecycle key");
    Require(first_key.buffer_index == second_key.buffer_index,
            "test must reuse the same V4L2 buffer index");

    CaptureFrameView restarted = first;
    restarted.identity.capture_session_id += 1U;
    const CaptureBufferKey restarted_key = MakeCaptureBufferKey(restarted);
    Require(first_key != restarted_key,
            "camera restart must have a different lifecycle key");
}

void TestValidFrame() {
    const CaptureFrameView frame = MakeValidFrame();
    const auto validation = ValidateCaptureFrameView(frame, true);
    Require(validation.ok, "valid frame was rejected");
    Require(validation.error == CaptureFrameValidationError::NONE,
            "valid frame returned an error");
}

void TestInvalidPlaneCount() {
    CaptureFrameView frame = MakeValidFrame();
    frame.plane_count = 0U;

    const auto validation = ValidateCaptureFrameView(frame, true);
    Require(!validation.ok, "zero plane count was accepted");
    Require(validation.error ==
                CaptureFrameValidationError::INVALID_PLANE_COUNT,
            "wrong validation error for zero plane count");
}

void TestInvalidDataRange() {
    CaptureFrameView frame = MakeValidFrame();
    frame.planes[0].data_offset = 4000U;
    frame.planes[0].bytes_used = 128U;

    const auto validation = ValidateCaptureFrameView(frame, true);
    Require(!validation.ok, "out-of-bounds plane data was accepted");
    Require(validation.error ==
                CaptureFrameValidationError::DATA_RANGE_OUT_OF_BOUNDS,
            "wrong validation error for plane range");
}

void TestDmabufRequirement() {
    CaptureFrameView frame = MakeValidFrame();
    frame.planes[0].dma_fd = -1;

    Require(ValidateCaptureFrameView(frame, false).ok,
            "CPU-only frame should permit missing DMA-BUF fd");

    const auto validation = ValidateCaptureFrameView(frame, true);
    Require(!validation.ok, "required DMA-BUF fd was not enforced");
    Require(validation.error ==
                CaptureFrameValidationError::INVALID_DMA_FD,
            "wrong validation error for DMA-BUF fd");
}

}  // namespace

int main() {
    static_assert(
        std::is_same_v<visionarm::FramePacket,
                       visionarm::CaptureFrameView>,
        "R0 compatibility alias changed");
    static_assert(
        std::is_same_v<visionarm::FramePlane,
                       visionarm::CapturePlaneView>,
        "R0 plane compatibility alias changed");
    static_assert(!std::is_copy_constructible_v<visionarm::IFrameLease>,
                  "Frame leases must not be copyable");
    static_assert(!std::is_copy_assignable_v<visionarm::IFrameLease>,
                  "Frame leases must not be copy assignable");

    try {
        TestConsumerMask();
        TestBufferKeyIncludesGeneration();
        TestValidFrame();
        TestInvalidPlaneCount();
        TestInvalidDataRange();
        TestDmabufRequirement();
    } catch (const std::exception& error) {
        std::cerr << "capture_buffer_contract_test FAILED: "
                  << error.what() << '\n';
        return 1;
    }

    std::cout << "capture_buffer_contract_test PASSED\n";
    return 0;
}
