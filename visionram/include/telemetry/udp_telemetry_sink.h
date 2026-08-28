#pragma once

// Transitional include path. New code uses observability/telemetry.h.
#include "observability/telemetry.h"

namespace visionarm {

// Kept only while the existing regression guardrail still names the former
// helper. The JSON schema and payload are unchanged.
[[nodiscard]] inline std::string BuildV8TelemetryJson(
    std::uint64_t sequence,
    std::int64_t sent_monotonic_ns,
    const UdpTelemetrySinkConfig& config,
    const TelemetryRuntimeStatus& runtime,
    const std::optional<ControlResult>& control) {
    return BuildTelemetryJson(sequence, sent_monotonic_ns, config, runtime,
                              control);
}

}  // namespace visionarm
