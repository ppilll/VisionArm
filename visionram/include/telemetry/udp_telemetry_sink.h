#pragma once

#include "control/control_sink.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace visionarm {

enum class TelemetryOutputState : std::uint8_t {
    DISABLED,
    STARTING,
    RUNNING,
    FINALIZED,
    FATAL,
};

[[nodiscard]] const char* TelemetryOutputStateName(
    TelemetryOutputState state) noexcept;

struct TelemetryRuntimeStatus {
    std::int64_t sample_monotonic_ns = 0;
    std::int64_t media_epoch_monotonic_ns = 0;
    bool pipeline_running = false;
    bool pipeline_fatal_error = false;

    std::uint64_t captured_frames = 0U;
    std::uint64_t inference_results = 0U;
    std::uint64_t video_frames_encoded = 0U;
    double camera_fps = 0.0;
    double inference_fps = 0.0;

    TelemetryOutputState network_state = TelemetryOutputState::DISABLED;
    TelemetryOutputState recording_state = TelemetryOutputState::DISABLED;
    std::uint64_t network_video_access_units = 0U;
    std::uint64_t network_audio_packets = 0U;
    std::uint64_t recording_video_samples = 0U;
    std::uint64_t recording_audio_packets = 0U;
};

struct UdpTelemetrySinkConfig {
    std::string host;
    std::uint16_t port = 0U;
    std::uint32_t interval_ms = 100U;
    std::int32_t send_buffer_bytes = 1 * 1'024 * 1'024;
    std::size_t maximum_datagram_bytes = 1'400U;
    std::string control_backend = "mock";
};

struct UdpTelemetrySinkSnapshot {
    bool started = false;
    bool running = false;
    bool stopped_cleanly = false;
    bool fatal_error = false;
    std::string last_error;

    std::uint64_t control_updates = 0U;
    std::uint64_t runtime_updates = 0U;
    std::uint64_t datagrams_attempted = 0U;
    std::uint64_t datagrams_sent = 0U;
    std::uint64_t bytes_sent = 0U;
    std::uint64_t send_failures = 0U;
    std::uint64_t serialization_failures = 0U;
    std::uint64_t oversized_datagrams = 0U;
    std::uint64_t final_sequence = 0U;
};

// Stable JSON wire-format helper used by the UDP worker and host unit test.
// It performs no I/O and throws if a numeric field is not finite.
[[nodiscard]] std::string BuildV8TelemetryJson(
    std::uint64_t sequence,
    std::int64_t sent_monotonic_ns,
    const UdpTelemetrySinkConfig& config,
    const TelemetryRuntimeStatus& runtime,
    const std::optional<ControlResult>& control);

// V8.5 latest-state UDP side telemetry.
//
// UpdateControl() and UpdateRuntime() only replace one owning in-memory
// snapshot. The integration supervisor samples LatestResultStore; this class
// is deliberately not an IControlSink and is never called by the inference
// control path. Neither update performs socket I/O or retains Camera
// FrameLease/DMA-BUF. One worker serializes the newest state at a bounded
// fixed rate.
class UdpTelemetrySink final {
public:
    explicit UdpTelemetrySink(UdpTelemetrySinkConfig config);
    ~UdpTelemetrySink();

    UdpTelemetrySink(const UdpTelemetrySink&) = delete;
    UdpTelemetrySink& operator=(const UdpTelemetrySink&) = delete;

    [[nodiscard]] bool Start(std::string* error = nullptr) noexcept;
    [[nodiscard]] bool UpdateControl(const ControlResult& result) noexcept;
    [[nodiscard]] bool UpdateRuntime(
        const TelemetryRuntimeStatus& status) noexcept;
    [[nodiscard]] bool Stop() noexcept;
    [[nodiscard]] UdpTelemetrySinkSnapshot Snapshot() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace visionarm
