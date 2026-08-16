#include "metrics/v7_control_trace_recorder.h"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

class AcceptingSink final : public visionarm::IControlSink {
public:
    bool Submit(const visionarm::ControlResult&) noexcept override {
        ++count;
        return true;
    }

    uint64_t count = 0U;
};

}  // namespace

int main() {
    const std::string path = "/tmp/visionarm_v7_control_trace_test.csv";
    std::remove(path.c_str());

    AcceptingSink downstream;
    visionarm::V7ControlTraceStats stats;
    {
        visionarm::V7ControlTraceRecorder recorder(downstream, path);
        for (uint64_t i = 1U; i <= 3U; ++i) {
            visionarm::ControlResult result;
            result.identity.capture_session_id = 7U;
            result.identity.frame_id = i;
            result.identity.v4l2_sequence = static_cast<uint32_t>(100U + i);
            result.state = visionarm::TargetState::DETECTED;
            result.valid = true;
            result.error.valid = true;
            result.error.confidence = 0.9F;
            result.error.dx_px = static_cast<float>(i);
            result.error.dy_px = -static_cast<float>(i);
            result.error.error_x_normalized = 0.1F * static_cast<float>(i);
            result.error.error_y_normalized = -0.1F * static_cast<float>(i);
            result.capture_timestamp_ns = static_cast<int64_t>(1000U * i);
            result.generated_timestamp_ns = static_cast<int64_t>(1100U * i);
            result.age_ns = 100;
            assert(recorder.Submit(result));
        }
        recorder.Stop();
        stats = recorder.Snapshot();
    }

    assert(downstream.count == 3U);
    assert(stats.submissions == 3U);
    assert(stats.downstream_accepted == 3U);
    assert(stats.downstream_rejected == 0U);
    assert(stats.enqueued == 3U);
    assert(stats.dropped == 0U);
    assert(stats.written == 3U);
    assert(stats.write_errors == 0U);

    std::ifstream input(path);
    assert(input.good());
    std::string line;
    int line_count = 0;
    while (std::getline(input, line)) {
        ++line_count;
    }
    assert(line_count == 4);

    std::remove(path.c_str());
    return 0;
}
