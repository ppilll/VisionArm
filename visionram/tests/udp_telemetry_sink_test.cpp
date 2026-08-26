#include "telemetry/udp_telemetry_sink.h"

#include <arpa/inet.h>
#include <limits>
#include <netinet/in.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace {

void Check(bool condition) {
    if (!condition) {
        throw std::runtime_error("udp telemetry test check failed");
    }
}

}  // namespace

int main() {
    visionarm::UdpTelemetrySinkConfig config;
    config.host = "127.0.0.1";
    config.port = 5001U;
    config.control_backend = "mock";

    visionarm::TelemetryRuntimeStatus runtime;
    runtime.sample_monotonic_ns = 2'000'000'000LL;
    runtime.media_epoch_monotonic_ns = 1'000'000'000LL;
    runtime.pipeline_running = true;
    runtime.captured_frames = 60U;
    runtime.inference_results = 20U;
    runtime.video_frames_encoded = 60U;
    runtime.camera_fps = 30.0;
    runtime.inference_fps = 10.0;
    runtime.network_state = visionarm::TelemetryOutputState::RUNNING;
    runtime.recording_state = visionarm::TelemetryOutputState::FINALIZED;

    visionarm::ControlResult control;
    control.identity.frame_id = 42U;
    control.state = visionarm::TargetState::DETECTED;
    control.valid = true;
    control.observation.state = visionarm::TargetState::DETECTED;
    control.observation.valid = true;
    control.observation.confidence = 0.875F;
    control.observation.class_id = 0;
    control.observation.x1 = 10.0F;
    control.observation.y1 = 20.0F;
    control.observation.x2 = 30.0F;
    control.observation.y2 = 40.0F;
    control.observation.source_width = 1'920;
    control.observation.source_height = 1'080;
    control.error.valid = true;
    control.error.dx_px = 3.0F;
    control.error.dy_px = -4.0F;
    control.error.error_x_normalized = 0.1F;
    control.error.error_y_normalized = -0.2F;
    control.age_ns = 25'000'000LL;
    control.capture_timestamp_ns = 1'900'000'000LL;
    control.generated_timestamp_ns = 1'925'000'000LL;

    const std::string json = visionarm::BuildV8TelemetryJson(
        7U, 2'100'000'000LL, config, runtime, control);
    Check(json.size() < config.maximum_datagram_bytes);
    Check(json.find("\"schema\":\"visionarm.telemetry.v1\"") !=
           std::string::npos);
    Check(json.find("\"sequence\":7") != std::string::npos);
    Check(json.find("\"fps\":30.000") != std::string::npos);
    Check(json.find("\"state\":\"DETECTED\"") != std::string::npos);
    Check(json.find("\"capture_to_result_ms\":25.000") !=
           std::string::npos);
    Check(json.find("\"frame_id\":42") != std::string::npos);
    Check(json.find("\"capture_media_pts_ms\":900.000") !=
          std::string::npos);
    Check(json.find("\"result_staleness_ms\":175.000") !=
          std::string::npos);

    const std::string no_control = visionarm::BuildV8TelemetryJson(
        8U, 2'200'000'000LL, config, runtime, std::nullopt);
    Check(no_control.find("\"available\":false") != std::string::npos);
    Check(no_control.find("\"state\":\"NO_TARGET\"") !=
           std::string::npos);

    const int receiver = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    Check(receiver >= 0);
    sockaddr_in receiver_address{};
    receiver_address.sin_family = AF_INET;
    receiver_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    receiver_address.sin_port = 0;
    Check(::bind(receiver,
                  reinterpret_cast<const sockaddr*>(&receiver_address),
                  static_cast<socklen_t>(sizeof(receiver_address))) == 0);
    socklen_t address_length =
        static_cast<socklen_t>(sizeof(receiver_address));
    Check(::getsockname(
        receiver, reinterpret_cast<sockaddr*>(&receiver_address),
        &address_length) == 0);
    timeval timeout{};
    timeout.tv_sec = 2;
    Check(::setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO,
                        &timeout,
                        static_cast<socklen_t>(sizeof(timeout))) == 0);

    config.port = ntohs(receiver_address.sin_port);
    config.interval_ms = 20U;
    visionarm::UdpTelemetrySink sink(config);
    std::string start_error;
    Check(sink.Start(&start_error));
    Check(sink.UpdateControl(control));
    Check(sink.UpdateRuntime(runtime));
    char received[2'048]{};
    const ssize_t received_size =
        ::recv(receiver, received, sizeof(received), 0);
    Check(received_size > 0);
    const std::string received_json(
        received, static_cast<std::size_t>(received_size));
    Check(received_json.find("visionarm.telemetry.v1") !=
           std::string::npos);
    Check(sink.Stop());
    const auto snapshot = sink.Snapshot();
    Check(snapshot.started);
    Check(snapshot.stopped_cleanly);
    Check(!snapshot.running);
    Check(!snapshot.fatal_error);
    Check(snapshot.control_updates == 1U);
    Check(snapshot.runtime_updates == 1U);
    Check(snapshot.datagrams_sent >= 1U);
    Check(snapshot.send_failures == 0U);
    Check(::close(receiver) == 0);

    runtime.camera_fps = std::numeric_limits<double>::quiet_NaN();
    bool rejected_nonfinite = false;
    try {
        (void)visionarm::BuildV8TelemetryJson(
            9U, 2'300'000'000LL, config, runtime, control);
    } catch (const std::invalid_argument&) {
        rejected_nonfinite = true;
    }
    Check(rejected_nonfinite);
    return 0;
}
