#pragma once

#include <cstdint>
#include <exception>
#include <limits>
#include <optional>

namespace visionarm::uart {

enum class HealthStatus { UNKNOWN, CONNECTING, HEALTHY, DEGRADED, UNHEALTHY };
enum class HealthReason {
    initial, opening, negotiating, ready, response_timeout, link_lost,
    open_failed, hello_rejected, startup_not_ready, peer_reboot, recovered,
    owner_stopped
};

inline const char* HealthStatusName(HealthStatus value) noexcept {
    switch (value) {
        case HealthStatus::UNKNOWN: return "UNKNOWN";
        case HealthStatus::CONNECTING: return "CONNECTING";
        case HealthStatus::HEALTHY: return "HEALTHY";
        case HealthStatus::DEGRADED: return "DEGRADED";
        case HealthStatus::UNHEALTHY: return "UNHEALTHY";
    }
    return "UNKNOWN";
}

inline const char* HealthReasonName(HealthReason value) noexcept {
    switch (value) {
        case HealthReason::initial: return "initial";
        case HealthReason::opening: return "opening";
        case HealthReason::negotiating: return "negotiating";
        case HealthReason::ready: return "ready";
        case HealthReason::response_timeout: return "response_timeout";
        case HealthReason::link_lost: return "link_lost";
        case HealthReason::open_failed: return "open_failed";
        case HealthReason::hello_rejected: return "hello_rejected";
        case HealthReason::startup_not_ready: return "startup_not_ready";
        case HealthReason::peer_reboot: return "peer_reboot";
        case HealthReason::recovered: return "recovered";
        case HealthReason::owner_stopped: return "owner_stopped";
    }
    return "initial";
}

struct UartHealthSample {
    HealthStatus status = HealthStatus::UNKNOWN;
    HealthReason reason = HealthReason::initial;
    uint64_t projection_revision = 0U;
    uint64_t link_epoch = 1U;
    std::optional<uint32_t> peer_boot_id;
};

inline bool SameHealthSample(const UartHealthSample& a,
                             const UartHealthSample& b) noexcept {
    return a.status == b.status && a.reason == b.reason &&
        a.projection_revision == b.projection_revision &&
        a.link_epoch == b.link_epoch && a.peer_boot_id == b.peer_boot_id;
}

struct UartHealthProjection {
    UartHealthSample current;
    std::optional<UartHealthSample> pending_issue;
};

// Only the UART owner mutates this record under its snapshot mutex. Readers
// copy both samples under that same mutex, then perform network I/O outside it.
// This is a bounded observation record, with no timeout/retry/wire policy.
class UartHealthRecord {
public:
    UartHealthProjection Read() const noexcept { return projection_; }

    void Record(HealthStatus status, HealthReason reason,
                std::optional<uint32_t> boot, bool new_session = false) noexcept {
        auto& current = projection_.current;
        const bool changed = status != current.status || reason != current.reason ||
            boot != current.peer_boot_id || new_session;
        if (!changed) return;
        Increment(current.projection_revision);
        if (new_session) Increment(current.link_epoch);
        current.status = status;
        current.reason = reason;
        current.peer_boot_id = boot;
        if (status == HealthStatus::DEGRADED || status == HealthStatus::UNHEALTHY) {
            auto& issue = projection_.pending_issue;
            if (!issue || Severity(status) >= Severity(issue->status)) {
                issue = current;
            } else {
                // Keep the worst reason, but cover the newest adverse revision
                // and its session/boot evidence. ACK of an older read cannot clear it.
                issue->projection_revision = current.projection_revision;
                issue->link_epoch = current.link_epoch;
                issue->peer_boot_id = current.peer_boot_id;
            }
        }
    }

    void AckThrough(uint64_t revision) noexcept {
        if (projection_.pending_issue &&
            projection_.pending_issue->projection_revision <= revision) {
            projection_.pending_issue.reset();
        }
    }

private:
    static unsigned Severity(HealthStatus status) noexcept {
        return status == HealthStatus::UNHEALTHY ? 2U : 1U;
    }
    static void Increment(uint64_t& value) noexcept {
        // Never wrap launch-scoped ordering. Exhaustion is a real owner fatal,
        // not a link-loss recovery trigger.
        if (value == std::numeric_limits<uint64_t>::max()) std::terminate();
        ++value;
    }
    UartHealthProjection projection_;
};

}  // namespace visionarm::uart
