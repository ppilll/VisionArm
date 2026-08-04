#include "control/control_sink.h"
#include "pipeline/target_state_machine.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

class RecordingPerceptionSink final : public visionarm::IPerceptionSink {
public:
    void Publish(visionarm::PerceptionPacket packet) override {
        latest = std::move(packet);
        ++count;
    }

    visionarm::PerceptionPacket latest;
    uint64_t count = 0U;
};

visionarm::PerceptionPacket DetectionPacket(
    uint64_t frame_id,
    int64_t capture_ns) {
    visionarm::PerceptionPacket packet;
    packet.identity.frame_id = frame_id;
    packet.identity.capture_timestamp_ns = capture_ns;

    visionarm::TargetObservation observation;
    observation.state = visionarm::TargetState::DETECTED;
    observation.valid = true;
    observation.confidence = 0.9F;
    observation.class_id = 0;
    observation.source_width = 1280;
    observation.source_height = 720;
    observation.center_x = 700.0F;
    observation.center_y = 300.0F;
    packet.result.target = observation;

    packet.result.error.state = visionarm::TargetState::DETECTED;
    packet.result.error.valid = true;
    packet.result.error.dx_px = 60.0F;
    packet.result.error.dy_px = -60.0F;
    packet.result.error.error_x_normalized = 0.09375F;
    packet.result.error.error_y_normalized = -0.1666667F;
    packet.result.error.confidence = 0.9F;
    return packet;
}

visionarm::PerceptionPacket NoTargetPacket(
    uint64_t frame_id,
    int64_t capture_ns) {
    visionarm::PerceptionPacket packet;
    packet.identity.frame_id = frame_id;
    packet.identity.capture_timestamp_ns = capture_ns;
    return packet;
}

visionarm::PerceptionPacket InvalidPacket(
    uint64_t frame_id,
    int64_t capture_ns) {
    visionarm::PerceptionPacket packet;
    packet.identity.frame_id = frame_id;
    packet.identity.capture_timestamp_ns = capture_ns;
    visionarm::TargetObservation observation;
    observation.state = visionarm::TargetState::INVALID;
    packet.result.target = observation;
    packet.result.error.state = visionarm::TargetState::INVALID;
    return packet;
}

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

}  // namespace

int main() {
    try {
        int64_t now_ns = 1'000'000'000LL;
        visionarm::MockControlSink control;
        RecordingPerceptionSink perception;
        visionarm::TargetStateMachineConfig config;
        config.acquire_hits = 2U;
        config.lost_misses = 2U;
        config.max_result_age_ns = 100'000'000LL;
        visionarm::TargetStateMachine machine(
            config,
            &control,
            &perception,
            [&now_ns] { return now_ns; });

        machine.Publish(DetectionPacket(1U, now_ns - 10'000'000LL));
        auto latest = control.Snapshot().latest;
        Require(latest.has_value(), "candidate result missing");
        Require(latest->state == visionarm::TargetState::CANDIDATE,
                "first hit must be CANDIDATE");
        Require(!latest->valid, "candidate must not produce valid control");
        Require(latest->error.dx_px == 0.0F,
                "candidate error must be sanitized");

        now_ns += 20'000'000LL;
        machine.Publish(DetectionPacket(2U, now_ns - 10'000'000LL));
        latest = control.Snapshot().latest;
        Require(latest->state == visionarm::TargetState::DETECTED,
                "second hit must acquire DETECTED");
        Require(latest->valid, "DETECTED must produce valid control");
        Require(latest->error.dx_px == 60.0F,
                "detected error must be current-frame error");

        now_ns += 20'000'000LL;
        machine.Publish(NoTargetPacket(3U, now_ns - 10'000'000LL));
        latest = control.Snapshot().latest;
        Require(latest->state == visionarm::TargetState::NO_TARGET,
                "single miss must not immediately become LOST");
        Require(!latest->valid && latest->error.dx_px == 0.0F,
                "miss must never reuse old valid control");

        now_ns += 20'000'000LL;
        machine.Publish(NoTargetPacket(4U, now_ns - 10'000'000LL));
        latest = control.Snapshot().latest;
        Require(latest->state == visionarm::TargetState::LOST,
                "lost_misses must transition to LOST");
        Require(!latest->valid, "LOST must be invalid control");

        now_ns += 20'000'000LL;
        machine.Publish(DetectionPacket(5U, now_ns - 10'000'000LL));
        latest = control.Snapshot().latest;
        Require(latest->state == visionarm::TargetState::CANDIDATE,
                "reacquisition must begin at CANDIDATE");

        now_ns += 20'000'000LL;
        machine.Publish(InvalidPacket(6U, now_ns - 10'000'000LL));
        latest = control.Snapshot().latest;
        Require(latest->state == visionarm::TargetState::INVALID,
                "invalid postprocess result must remain INVALID");
        Require(!latest->valid, "INVALID must be invalid control");

        now_ns += 200'000'000LL;
        machine.Publish(DetectionPacket(7U, now_ns - 150'000'000LL));
        latest = control.Snapshot().latest;
        Require(latest->state == visionarm::TargetState::STALE,
                "old result must become STALE");
        Require(!latest->valid && latest->error.dx_px == 0.0F,
                "STALE must not produce control");

        const auto state_snapshot = machine.Snapshot();
        Require(state_snapshot.processed_packets == 7U,
                "processed packet count mismatch");
        Require(state_snapshot.valid_controls == 1U,
                "valid control count mismatch");
        Require(perception.count == 7U,
                "finalized perception sink count mismatch");
        Require(perception.latest.result.error.state ==
                    visionarm::TargetState::STALE,
                "finalized perception state mismatch");

        std::cout << "target_state_machine_test PASSED\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "target_state_machine_test FAILED: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
