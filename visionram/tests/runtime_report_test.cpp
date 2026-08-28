#include "report/runtime_report.h"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "runtime_report_test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool HasLine(const std::string& report, const std::string& line) {
    return report.find(line + '\n') != std::string::npos;
}

std::string Fixture(bool audio_enabled, bool split_topology = false) {
    return
        "module.camera.enabled=1\n"
        "module.inference.enabled=1\n"
        "module.video.enabled=1\n"
        "module.audio.enabled=" + std::string(audio_enabled ? "1\n" : "0\n") +
        "module.audio_encoder.enabled=0\n"
        "module.recorder.enabled=0\n"
        "module.network.enabled=0\n"
        "module.telemetry.enabled=0\n"
        "module.uart.enabled=0\n"
        "topology=" + std::string(
            split_topology ? "split_npu_postprocess\n"
                           : "fused_npu_postprocess\n") +
        "queue.completed.capacity=1\n"
        "queue.completed.high_watermark=0\n"
        "queue.completed.current_size=0\n"
        "queue.completed.pushed=0\n"
        "queue.completed.popped=0\n"
        "queue.completed.replaced_oldest=0\n"
        "queue.completed.stopped=1\n"
        "queue.captured.capacity=1\n"
        "queue.captured.high_watermark=1\n"
        "queue.captured.current_size=0\n"
        "queue.captured.pushed=10\n"
        "queue.captured.popped=10\n"
        "queue.captured.replaced_oldest=0\n"
        "queue.captured.stopped=1\n"
        "requested_duration_seconds=600\n"
        "observed_duration_seconds=600.030\n"
        "completed_requested_duration=1\n"
        "terminated_by_signal=0\n"
        "fatal_error=0\n"
        "graceful_shutdown_completed=1\n"
        "captured_frames=17996\n"
        "audio_timed_chunks=100\n"
        "audio_encode_packets_pushed=100\n"
        "audio_worker_last_error=\n"
        "camera_requested_width=1920\n"
        "latency.capture_to_result.p99_ms=86.305\n"
        "result=PASS\n";
}

std::string Render(std::string payload,
                   visionarm::report::ReportLevel level) {
    std::ostringstream output;
    std::string error;
    Require(visionarm::report::WriteRuntimeReport(
                payload, level, output, &error),
            "fixture serialization must succeed");
    return output.str();
}

void TestLevelsAndModuleFiltering() {
    const std::string summary = Render(
        Fixture(true), visionarm::report::ReportLevel::SUMMARY);
    const std::string performance = Render(
        Fixture(true), visionarm::report::ReportLevel::PERFORMANCE);
    const std::string diagnostic = Render(
        Fixture(true), visionarm::report::ReportLevel::DIAGNOSTIC);

    Require(HasLine(summary, "schema=visionarm.runtime_report.v1"),
            "schema must be present");
    Require(HasLine(summary, "report_level=summary"),
            "summary level must be explicit");
    Require(HasLine(summary, "result=PASS"),
            "summary must carry the final result");
    Require(HasLine(summary, "queue.captured.current_size=0"),
            "summary must expose queue drain state");
    Require(HasLine(summary, "queue.captured.stopped=1"),
            "summary must expose queue stop state");
    Require(!HasLine(summary, "queue.captured.capacity=1"),
            "summary must omit detailed queue capacity");
    Require(!HasLine(summary, "latency.capture_to_result.p99_ms=86.305"),
            "summary must omit performance latency");
    Require(HasLine(performance,
                    "latency.capture_to_result.p99_ms=86.305"),
            "performance must include latency");
    Require(!HasLine(performance, "camera_requested_width=1920"),
            "performance must omit diagnostic configuration");
    Require(HasLine(diagnostic, "camera_requested_width=1920"),
            "diagnostic must include configuration");
    Require(!HasLine(performance, "queue.completed.capacity=1"),
            "fused performance report must omit the unused completed queue");
    Require(!HasLine(diagnostic, "queue.completed.capacity=1"),
            "fused diagnostic report must omit the unused completed queue");

    const std::string split = Render(
        Fixture(true, true), visionarm::report::ReportLevel::PERFORMANCE);
    Require(HasLine(split, "queue.completed.capacity=1"),
            "split report must retain the active completed queue");

    const std::string audio_disabled = Render(
        Fixture(false), visionarm::report::ReportLevel::DIAGNOSTIC);
    Require(HasLine(audio_disabled, "module.audio.enabled=0"),
            "disabled module state must remain explicit");
    Require(!HasLine(audio_disabled, "audio_timed_chunks=100"),
            "disabled module detail must be omitted");
    Require(!HasLine(diagnostic, "audio_encode_packets_pushed=100"),
            "disabled audio encoder detail must be omitted");
}

void TestEscapingAndFaultFixture() {
    const std::string original = "disk=full\\path\nnext\titem";
    const std::string escaped = visionarm::report::EscapeReportValue(original);
    std::string decoded;
    std::string error;
    Require(visionarm::report::UnescapeReportValue(
                escaped, &decoded, &error),
            "escaped value must decode");
    Require(decoded == original, "report escaping must round-trip");

    const std::string payload =
        "module.camera.enabled=0\n"
        "fatal_error=1\n"
        "fatal_message=" + escaped + "\n"
        "completed_requested_duration=0\n"
        "terminated_by_signal=0\n"
        "graceful_shutdown_completed=0\n"
        "result=FAIL\n";
    const std::string report = Render(
        payload, visionarm::report::ReportLevel::SUMMARY);
    Require(HasLine(report, "fatal_message=" + escaped),
            "fault report must preserve escaped error text");
    Require(HasLine(report, "result=FAIL"),
            "fault report must be machine-readable");
}

void TestDuplicateRejected() {
    std::ostringstream output;
    std::string error;
    Require(!visionarm::report::WriteRuntimeReport(
                "result=PASS\nresult=FAIL\n",
                visionarm::report::ReportLevel::SUMMARY, output, &error),
            "duplicate keys must be rejected");
    Require(error.find("duplicate") != std::string::npos,
            "duplicate rejection must be explicit");
}

class FailingBuffer final : public std::streambuf {
protected:
    int_type overflow(int_type) override { return traits_type::eof(); }
};

void TestWriteFailureDetected() {
    FailingBuffer buffer;
    std::ostream output(&buffer);
    std::string error;
    Require(!visionarm::report::WriteRuntimeReport(
                "result=PASS\n", visionarm::report::ReportLevel::SUMMARY,
                output, &error),
            "stream failure must not report success");
}

}  // namespace

int main() {
    TestLevelsAndModuleFiltering();
    TestEscapingAndFaultFixture();
    TestDuplicateRejected();
    TestWriteFailureDetected();
    std::cout << "runtime_report_test=PASS\n";
    return EXIT_SUCCESS;
}
