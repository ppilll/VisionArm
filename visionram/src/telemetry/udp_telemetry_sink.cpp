#include "telemetry/udp_telemetry_sink.h"

#include "common/monotonic_clock.h"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <mutex>
#include <netdb.h>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>

namespace visionarm {
namespace {

[[nodiscard]] std::string EscapeJson(const std::string& value) {
    std::ostringstream output;
    for (const char raw_character : value) {
        const auto character = static_cast<unsigned char>(raw_character);
        switch (character) {
            case '"': output << "\\\""; break;
            case '\\': output << "\\\\"; break;
            case '\b': output << "\\b"; break;
            case '\f': output << "\\f"; break;
            case '\n': output << "\\n"; break;
            case '\r': output << "\\r"; break;
            case '\t': output << "\\t"; break;
            default:
                if (character < 0x20U) {
                    output << "\\u" << std::hex << std::setw(4)
                           << std::setfill('0')
                           << static_cast<unsigned int>(character)
                           << std::dec << std::setfill(' ');
                } else {
                    output << static_cast<char>(character);
                }
        }
    }
    return output.str();
}

void RequireFinite(double value, const char* name) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(
            std::string("non-finite telemetry field: ") + name);
    }
}

[[nodiscard]] const char* JsonBool(bool value) noexcept {
    return value ? "true" : "false";
}

}  // namespace

const char* TelemetryOutputStateName(TelemetryOutputState state) noexcept {
    switch (state) {
        case TelemetryOutputState::DISABLED: return "disabled";
        case TelemetryOutputState::STARTING: return "starting";
        case TelemetryOutputState::RUNNING: return "running";
        case TelemetryOutputState::FINALIZED: return "finalized";
        case TelemetryOutputState::FATAL: return "fatal";
    }
    return "unknown";
}

std::string BuildV8TelemetryJson(
    std::uint64_t sequence,
    std::int64_t sent_monotonic_ns,
    const UdpTelemetrySinkConfig& config,
    const TelemetryRuntimeStatus& runtime,
    const std::optional<ControlResult>& control) {
    RequireFinite(runtime.camera_fps, "camera_fps");
    RequireFinite(runtime.inference_fps, "inference_fps");

    ControlResult latest;
    const bool control_available = control.has_value();
    if (control_available) {
        latest = *control;
        RequireFinite(latest.observation.confidence, "target.confidence");
        RequireFinite(latest.observation.x1, "target.x1");
        RequireFinite(latest.observation.y1, "target.y1");
        RequireFinite(latest.observation.x2, "target.x2");
        RequireFinite(latest.observation.y2, "target.y2");
        RequireFinite(latest.error.dx_px, "control.dx_px");
        RequireFinite(latest.error.dy_px, "control.dy_px");
        RequireFinite(latest.error.error_x_normalized, "control.error_x");
        RequireFinite(latest.error.error_y_normalized, "control.error_y");
    }
    const double capture_media_pts_ms =
        control_available && runtime.media_epoch_monotonic_ns > 0 &&
        latest.capture_timestamp_ns >= runtime.media_epoch_monotonic_ns
            ? static_cast<double>(latest.capture_timestamp_ns -
                  runtime.media_epoch_monotonic_ns) / 1'000'000.0
            : 0.0;
    const double result_staleness_ms =
        control_available && latest.generated_timestamp_ns > 0 &&
        sent_monotonic_ns >= latest.generated_timestamp_ns
            ? static_cast<double>(sent_monotonic_ns -
                  latest.generated_timestamp_ns) / 1'000'000.0
            : 0.0;

    std::ostringstream json;
    json << std::fixed << std::setprecision(3)
         << "{\"schema\":\"visionarm.telemetry.v1\""
         << ",\"sequence\":" << sequence
         << ",\"sent_monotonic_ns\":" << sent_monotonic_ns
         << ",\"control_backend\":\""
         << EscapeJson(config.control_backend) << "\""
         << ",\"pipeline\":{\"sample_monotonic_ns\":"
         << runtime.sample_monotonic_ns
         << ",\"media_epoch_monotonic_ns\":"
         << runtime.media_epoch_monotonic_ns
         << ",\"running\":" << JsonBool(runtime.pipeline_running)
         << ",\"fatal_error\":"
         << JsonBool(runtime.pipeline_fatal_error) << '}'
         << ",\"camera\":{\"captured_frames\":"
         << runtime.captured_frames
         << ",\"fps\":" << runtime.camera_fps << '}'
         << ",\"inference\":{\"results\":"
         << runtime.inference_results
         << ",\"fps\":" << runtime.inference_fps << '}'
         << ",\"video\":{\"frames_encoded\":"
         << runtime.video_frames_encoded << '}'
         << ",\"network\":{\"state\":\""
         << TelemetryOutputStateName(runtime.network_state)
         << "\",\"video_access_units\":"
         << runtime.network_video_access_units
         << ",\"audio_packets\":" << runtime.network_audio_packets << '}'
         << ",\"recording\":{\"state\":\""
         << TelemetryOutputStateName(runtime.recording_state)
         << "\",\"video_samples\":"
         << runtime.recording_video_samples
         << ",\"audio_packets\":" << runtime.recording_audio_packets << '}'
         << ",\"target\":{\"available\":"
         << JsonBool(control_available)
         << ",\"state\":\"" << TargetStateName(latest.state) << "\""
         << ",\"valid\":" << JsonBool(latest.observation.valid)
         << ",\"confidence\":" << latest.observation.confidence
         << ",\"class_id\":" << latest.observation.class_id
         << ",\"bbox\":[" << latest.observation.x1 << ','
         << latest.observation.y1 << ',' << latest.observation.x2 << ','
         << latest.observation.y2 << ']'
         << ",\"source_width\":" << latest.observation.source_width
         << ",\"source_height\":" << latest.observation.source_height
         << '}'
         << ",\"control\":{\"valid\":" << JsonBool(latest.valid)
         << ",\"dx_px\":" << latest.error.dx_px
         << ",\"dy_px\":" << latest.error.dy_px
         << ",\"error_x_normalized\":"
         << latest.error.error_x_normalized
         << ",\"error_y_normalized\":"
         << latest.error.error_y_normalized
         << ",\"capture_to_result_ms\":"
         << static_cast<double>(latest.age_ns) / 1'000'000.0
         << ",\"capture_media_pts_ms\":" << capture_media_pts_ms
         << ",\"result_staleness_ms\":" << result_staleness_ms
         << ",\"capture_monotonic_ns\":"
         << latest.capture_timestamp_ns
         << ",\"generated_monotonic_ns\":"
         << latest.generated_timestamp_ns
         << ",\"frame_id\":" << latest.identity.frame_id
         << ",\"consecutive_hits\":" << latest.consecutive_hits
         << ",\"consecutive_misses\":" << latest.consecutive_misses
         << "}}";
    return json.str();
}

struct UdpTelemetrySink::Impl {
    explicit Impl(UdpTelemetrySinkConfig value) : config(std::move(value)) {}

    UdpTelemetrySinkConfig config;
    mutable std::mutex mutex;
    std::condition_variable wake;
    UdpTelemetrySinkSnapshot snapshot;
    std::optional<ControlResult> latest_control;
    std::optional<TelemetryRuntimeStatus> latest_runtime;
    bool stop_requested = false;
    int socket_fd = -1;
    sockaddr_storage destination{};
    socklen_t destination_length = 0;
    std::thread worker;

    void SetError(const std::string& error) noexcept {
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.fatal_error = true;
        snapshot.last_error = error;
    }

    void CloseSocket() noexcept {
        if (socket_fd >= 0) {
            (void)::close(socket_fd);
            socket_fd = -1;
        }
    }

    [[nodiscard]] bool OpenSocket(std::string* error) noexcept {
        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_protocol = IPPROTO_UDP;
        addrinfo* addresses = nullptr;
        const std::string service = std::to_string(config.port);
        const int resolve = ::getaddrinfo(
            config.host.c_str(), service.c_str(), &hints, &addresses);
        if (resolve != 0) {
            if (error != nullptr) {
                *error = std::string("telemetry getaddrinfo failed: ") +
                    gai_strerror(resolve);
            }
            return false;
        }

        bool opened = false;
        for (const addrinfo* address = addresses;
             address != nullptr; address = address->ai_next) {
            if (address->ai_addrlen > sizeof(destination)) {
                continue;
            }
            const int candidate = ::socket(
                address->ai_family, address->ai_socktype, address->ai_protocol);
            if (candidate < 0) {
                continue;
            }
            const int flags = ::fcntl(candidate, F_GETFL, 0);
            if (flags < 0 || ::fcntl(candidate, F_SETFL, flags | O_NONBLOCK) < 0 ||
                ::setsockopt(candidate, SOL_SOCKET, SO_SNDBUF,
                    &config.send_buffer_bytes,
                    static_cast<socklen_t>(sizeof(config.send_buffer_bytes))) < 0) {
                (void)::close(candidate);
                continue;
            }
            socket_fd = candidate;
            std::memcpy(&destination, address->ai_addr, address->ai_addrlen);
            destination_length = static_cast<socklen_t>(address->ai_addrlen);
            opened = true;
            break;
        }
        ::freeaddrinfo(addresses);
        if (!opened && error != nullptr) {
            *error = std::string("telemetry UDP socket open failed: ") +
                std::strerror(errno);
        }
        return opened;
    }

    void SendLatest() noexcept {
        TelemetryRuntimeStatus runtime;
        std::optional<ControlResult> control;
        std::uint64_t sequence = 0U;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!latest_runtime.has_value() || socket_fd < 0) {
                return;
            }
            runtime = *latest_runtime;
            control = latest_control;
            sequence = ++snapshot.datagrams_attempted;
            snapshot.final_sequence = sequence;
        }

        std::string payload;
        try {
            payload = BuildV8TelemetryJson(
                sequence, MonotonicNowNs(), config, runtime, control);
        } catch (const std::exception& exception) {
            std::lock_guard<std::mutex> lock(mutex);
            ++snapshot.serialization_failures;
            snapshot.fatal_error = true;
            snapshot.last_error = exception.what();
            return;
        }
        if (payload.size() > config.maximum_datagram_bytes) {
            std::lock_guard<std::mutex> lock(mutex);
            ++snapshot.oversized_datagrams;
            snapshot.fatal_error = true;
            snapshot.last_error = "telemetry JSON exceeds maximum datagram size";
            return;
        }

        const ssize_t sent = ::sendto(
            socket_fd, payload.data(), payload.size(), MSG_DONTWAIT,
            reinterpret_cast<const sockaddr*>(&destination),
            destination_length);
        std::lock_guard<std::mutex> lock(mutex);
        if (sent < 0 || static_cast<std::size_t>(sent) != payload.size()) {
            ++snapshot.send_failures;
            snapshot.fatal_error = true;
            snapshot.last_error = std::string("telemetry sendto failed: ") +
                std::strerror(errno);
            return;
        }
        ++snapshot.datagrams_sent;
        snapshot.bytes_sent += static_cast<std::uint64_t>(sent);
    }

    void WorkerMain() noexcept {
        const auto interval = std::chrono::milliseconds(
            static_cast<std::int64_t>(config.interval_ms));
        for (;;) {
            std::unique_lock<std::mutex> lock(mutex);
            const bool stopping = wake.wait_for(
                lock, interval, [this] { return stop_requested; });
            lock.unlock();
            if (stopping) {
                SendLatest();
                break;
            }
            SendLatest();
        }
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.running = false;
        snapshot.stopped_cleanly = true;
    }
};

UdpTelemetrySink::UdpTelemetrySink(UdpTelemetrySinkConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
    if (impl_->config.host.empty() || impl_->config.port == 0U ||
        impl_->config.interval_ms < 20U ||
        impl_->config.interval_ms > 60'000U ||
        impl_->config.send_buffer_bytes <= 0 ||
        impl_->config.maximum_datagram_bytes == 0U ||
        impl_->config.maximum_datagram_bytes > 65'507U ||
        impl_->config.control_backend.empty()) {
        throw std::invalid_argument("invalid UDP telemetry config");
    }
}

UdpTelemetrySink::~UdpTelemetrySink() {
    (void)Stop();
}

bool UdpTelemetrySink::Start(std::string* error) noexcept {
    if (impl_ == nullptr) {
        if (error != nullptr) *error = "telemetry implementation is missing";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->snapshot.started || impl_->worker.joinable()) {
            if (error != nullptr) *error = "telemetry supports one lifecycle";
            return false;
        }
    }
    if (!impl_->OpenSocket(error)) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.started = true;
        impl_->snapshot.running = true;
    }
    try {
        impl_->worker = std::thread(&Impl::WorkerMain, impl_.get());
    } catch (const std::exception& exception) {
        impl_->CloseSocket();
        impl_->SetError(exception.what());
        if (error != nullptr) *error = exception.what();
        return false;
    } catch (...) {
        impl_->CloseSocket();
        impl_->SetError("telemetry worker creation failed");
        if (error != nullptr) *error = "telemetry worker creation failed";
        return false;
    }
    return true;
}

bool UdpTelemetrySink::UpdateControl(const ControlResult& result) noexcept {
    if (impl_ == nullptr) return false;
    try {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->snapshot.running || impl_->stop_requested) return false;
        impl_->latest_control = result;
        ++impl_->snapshot.control_updates;
        return true;
    } catch (...) {
        return false;
    }
}

bool UdpTelemetrySink::UpdateRuntime(
    const TelemetryRuntimeStatus& status) noexcept {
    if (impl_ == nullptr || status.sample_monotonic_ns <= 0 ||
        !std::isfinite(status.camera_fps) || status.camera_fps < 0.0 ||
        !std::isfinite(status.inference_fps) || status.inference_fps < 0.0) {
        return false;
    }
    try {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->snapshot.running || impl_->stop_requested) return false;
        impl_->latest_runtime = status;
        ++impl_->snapshot.runtime_updates;
        return true;
    } catch (...) {
        return false;
    }
}

bool UdpTelemetrySink::Stop() noexcept {
    if (impl_ == nullptr) return false;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->stop_requested = true;
    }
    impl_->wake.notify_all();
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
    impl_->CloseSocket();
    const UdpTelemetrySinkSnapshot snapshot = Snapshot();
    return snapshot.started && snapshot.stopped_cleanly &&
        !snapshot.fatal_error && snapshot.datagrams_sent > 0U;
}

UdpTelemetrySinkSnapshot UdpTelemetrySink::Snapshot() const noexcept {
    if (impl_ == nullptr) return {};
    try {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        return impl_->snapshot;
    } catch (...) {
        return {};
    }
}

}  // namespace visionarm
