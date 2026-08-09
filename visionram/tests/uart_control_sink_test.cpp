#include "control/uart_control_sink.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

visionarm::ControlResult MakeResult(visionarm::TargetState state) {
    visionarm::ControlResult result;
    result.identity.capture_session_id = 0x1234U;
    result.identity.frame_id = 0x5678U;
    result.identity.v4l2_sequence = 77U;
    result.state = state;
    result.capture_timestamp_ns = 1'000'000'000LL;
    result.error.state = state;
    return result;
}

visionarm::ControlResult MakeValidDetected() {
    auto result = MakeResult(visionarm::TargetState::DETECTED);
    result.valid = true;
    result.error.valid = true;
    result.error.dx_px = 12.5F;
    result.error.dy_px = -12.5F;
    result.error.error_x_normalized = 0.5F;
    result.error.error_y_normalized = -0.5F;
    result.error.confidence = 0.5F;
    return result;
}

visionarm::uart::UartLink MakeStoppedLink() {
    visionarm::uart::UartLinkConfig config;
    config.device_path = "/dev/this-test-never-opens";
    return visionarm::uart::UartLink(config);
}

void TestSixStateMapping() {
    auto link = MakeStoppedLink();
    visionarm::UartControlSink sink(link);

    const visionarm::TargetState states[] = {
        visionarm::TargetState::NO_TARGET,
        visionarm::TargetState::CANDIDATE,
        visionarm::TargetState::DETECTED,
        visionarm::TargetState::LOST,
        visionarm::TargetState::INVALID,
        visionarm::TargetState::STALE,
    };

    for (uint8_t expected = 0U; expected < 6U; ++expected) {
        auto result = MakeResult(states[expected]);
        if (result.state == visionarm::TargetState::DETECTED) {
            result.valid = true;
            result.error.valid = true;
            result.error.confidence = 0.5F;
        }
        Require(!sink.Submit(result),
                "stopped UartLink must reject adapter submission");
        const auto snapshot = sink.Snapshot();
        Require(snapshot.latest_input.has_value(),
                "mapped input missing from snapshot");
        Require(snapshot.latest_input->target_state == expected,
                "TargetState mapping mismatch");
    }

    const auto metrics = sink.Snapshot().metrics;
    for (std::size_t index = 0U; index < metrics.state_counts.size(); ++index) {
        Require(metrics.state_counts[index] == 1U,
                "state counter mismatch");
    }
    Require(metrics.submissions == 6U, "submission counter mismatch");
    Require(metrics.rejected == 6U, "rejection propagation mismatch");
}

void TestDetectedMappingAndRounding() {
    auto link = MakeStoppedLink();
    visionarm::UartControlSink sink(link);

    auto result = MakeValidDetected();
    Require(!sink.Submit(result), "stopped link rejection expected");

    const auto snapshot = sink.Snapshot();
    Require(snapshot.latest_input.has_value(), "mapped DETECTED missing");
    const auto& input = *snapshot.latest_input;

    Require(input.valid, "valid DETECTED must remain valid");
    Require(input.source_capture_session_id == 0x1234U,
            "capture session mapping mismatch");
    Require(input.source_frame_id == 0x5678U,
            "frame id mapping mismatch");
    Require(input.source_v4l2_sequence == 77U,
            "v4l2 sequence mapping mismatch");
    Require(input.dx_px == 13, "positive half rounding mismatch");
    Require(input.dy_px == -13, "negative half rounding mismatch");
    Require(input.error_x_q15 == 16384, "positive Q15 rounding mismatch");
    Require(input.error_y_q15 == -16384, "negative Q15 rounding mismatch");
    Require(input.confidence_u16 == 32768U,
            "confidence rounding mismatch");
    Require(input.capture_monotonic_ns == 1'000'000'000ULL,
            "capture timestamp mapping mismatch");
    Require(snapshot.metrics.valid_inputs == 1U,
            "valid input metric mismatch");
}

void TestInvalidStateZeroing() {
    auto link = MakeStoppedLink();
    visionarm::UartControlSink sink(link);

    auto result = MakeValidDetected();
    result.state = visionarm::TargetState::LOST;
    result.error.state = visionarm::TargetState::LOST;

    Require(!sink.Submit(result), "stopped link rejection expected");
    const auto input = *sink.Snapshot().latest_input;
    Require(!input.valid, "LOST must be invalid on wire");
    Require(input.dx_px == 0 && input.dy_px == 0,
            "invalid state must zero pixel error");
    Require(input.error_x_q15 == 0 && input.error_y_q15 == 0,
            "invalid state must zero Q15 error");
    Require(input.confidence_u16 == 0U,
            "invalid state must zero confidence");
}

void TestNonfiniteInvalidation() {
    auto link = MakeStoppedLink();
    visionarm::UartControlSink sink(link);

    auto result = MakeValidDetected();
    result.error.dx_px = std::numeric_limits<float>::quiet_NaN();
    Require(!sink.Submit(result), "stopped link rejection expected");
    auto snapshot = sink.Snapshot();
    Require(!snapshot.latest_input->valid,
            "NaN must invalidate control");
    Require(snapshot.metrics.nonfinite_invalidations == 1U,
            "NaN invalidation metric mismatch");

    result = MakeValidDetected();
    result.error.confidence = std::numeric_limits<float>::infinity();
    Require(!sink.Submit(result), "stopped link rejection expected");
    snapshot = sink.Snapshot();
    Require(!snapshot.latest_input->valid,
            "infinity must invalidate control");
    Require(snapshot.metrics.nonfinite_invalidations == 2U,
            "infinity invalidation metric mismatch");
}

void TestSaturationAndEndpoints() {
    auto link = MakeStoppedLink();
    visionarm::UartControlSink sink(link);

    auto result = MakeValidDetected();
    result.error.dx_px = 1.0e9F;
    result.error.dy_px = -1.0e9F;
    result.error.error_x_normalized = 2.0F;
    result.error.error_y_normalized = -2.0F;
    result.error.confidence = 2.0F;

    Require(!sink.Submit(result), "stopped link rejection expected");
    auto input = *sink.Snapshot().latest_input;
    Require(input.dx_px == std::numeric_limits<int16_t>::max(),
            "positive int16 saturation mismatch");
    Require(input.dy_px == std::numeric_limits<int16_t>::min(),
            "negative int16 saturation mismatch");
    Require(input.error_x_q15 == 32767,
            "positive Q15 endpoint mismatch");
    Require(input.error_y_q15 == -32767,
            "negative Q15 endpoint mismatch");
    Require(input.confidence_u16 == 65535U,
            "confidence high endpoint mismatch");

    result = MakeValidDetected();
    result.error.confidence = -1.0F;
    Require(!sink.Submit(result), "stopped link rejection expected");
    input = *sink.Snapshot().latest_input;
    Require(input.confidence_u16 == 0U,
            "confidence low endpoint mismatch");
}

void TestIdentityTruncation() {
    auto link = MakeStoppedLink();
    visionarm::UartControlSink sink(link);

    auto result = MakeValidDetected();
    result.identity.capture_session_id = 0x1'0000'0001ULL;
    result.identity.frame_id = 0x2'0000'0002ULL;

    Require(!sink.Submit(result), "stopped link rejection expected");
    const auto snapshot = sink.Snapshot();
    Require(snapshot.latest_input->source_capture_session_id == 1U,
            "capture session low-word mapping mismatch");
    Require(snapshot.latest_input->source_frame_id == 2U,
            "frame id low-word mapping mismatch");
    Require(snapshot.metrics.identity_truncations == 2U,
            "identity truncation metric mismatch");
}

void TestInvalidTimestamp() {
    auto link = MakeStoppedLink();
    visionarm::UartControlSink sink(link);

    auto result = MakeValidDetected();
    result.capture_timestamp_ns = 0;

    Require(!sink.Submit(result), "stopped link rejection expected");
    const auto snapshot = sink.Snapshot();
    Require(!snapshot.latest_input->valid,
            "invalid timestamp must invalidate control");
    Require(snapshot.latest_input->capture_monotonic_ns == 0U,
            "invalid timestamp must map to zero");
    Require(snapshot.metrics.invalid_timestamp_invalidations == 1U,
            "invalid timestamp metric mismatch");
}

void TestUnknownEnumValue() {
    auto link = MakeStoppedLink();
    visionarm::UartControlSink sink(link);

    auto result = MakeValidDetected();
    result.state = static_cast<visionarm::TargetState>(255U);

    Require(!sink.Submit(result), "stopped link rejection expected");
    const auto snapshot = sink.Snapshot();
    Require(!snapshot.latest_input->valid,
            "unknown state must invalidate control");
    Require(snapshot.latest_input->target_state == 4U,
            "unknown state must map to protocol INVALID");
    Require(snapshot.metrics.invalid_state_invalidations == 1U,
            "unknown-state metric mismatch");
}

}  // namespace

int main() {
    try {
        TestSixStateMapping();
        TestDetectedMappingAndRounding();
        TestInvalidStateZeroing();
        TestNonfiniteInvalidation();
        TestSaturationAndEndpoints();
        TestIdentityTruncation();
        TestInvalidTimestamp();
        TestUnknownEnumValue();
        std::cout << "uart_control_sink_test PASSED\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "uart_control_sink_test FAILED: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
