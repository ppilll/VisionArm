#pragma once

#include "uart/protocol.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace visionarm::uart {

enum class LinkState : uint8_t {
    CLOSED = 0U,
    OPENING,
    NEGOTIATING,
    READY,
    DEGRADED,
    LINK_LOST,
    STOPPING,
    FAILED,
};

[[nodiscard]] const char* LinkStateName(LinkState state) noexcept;

struct UartLinkConfig {
    std::string device_path;
    uint32_t baud_rate = 115200U;
    bool exclusive = true;
    bool flush_on_open = true;

    uint64_t open_retry_ms = 1000U;
    uint64_t hello_period_ms = 500U;
    uint64_t heartbeat_period_ms = 200U;
    uint64_t response_timeout_ms = 150U;
    uint64_t reliable_timeout_ms = 200U;
    uint32_t reliable_max_retries = 3U;
    uint64_t parser_assembly_timeout_ms = 100U;
    uint64_t link_watchdog_timeout_ms = 1000U;
    uint64_t control_max_age_ms = 200U;
    uint32_t max_control_rate_hz = 30U;

    uint8_t linux_device_role = 1U;
    uint8_t expected_mcu_device_role = 2U;
    uint32_t capability_bits = 0U;
    uint16_t software_version_major = 5U;
    uint16_t software_version_minor = 3U;
    uint16_t software_version_patch = 0U;
};

// Protocol-domain control value accepted by the R5 UART module.
//
// All V4 pipeline interpretation and float/fixed-point conversion belongs to
// the future R6 adapter. The R5 link only applies transport-time freshness and
// encodes these already-normalized protocol-domain fields.
struct ControlUpdateInput {
    uint32_t source_capture_session_id = 0U;
    uint32_t source_frame_id = 0U;
    uint32_t source_v4l2_sequence = 0U;

    uint8_t target_state = 0U;
    bool valid = false;

    int16_t dx_px = 0;
    int16_t dy_px = 0;
    int16_t error_x_q15 = 0;
    int16_t error_y_q15 = 0;
    uint16_t confidence_u16 = 0U;

    // CLOCK_MONOTONIC timestamp of the source capture in nanoseconds.
    uint64_t capture_monotonic_ns = 0U;
};

struct UartMetrics {
    uint64_t open_attempts = 0U;
    uint64_t open_successes = 0U;
    uint64_t reopen_count = 0U;
    uint64_t poll_errors = 0U;
    uint64_t read_errors = 0U;
    uint64_t write_errors = 0U;

    uint64_t rx_bytes = 0U;
    uint64_t tx_bytes = 0U;
    uint64_t rx_valid_frames = 0U;
    uint64_t tx_frames = 0U;
    uint64_t message_decode_errors = 0U;
    uint64_t unexpected_responses = 0U;

    uint64_t wire_duplicates = 0U;
    uint64_t wire_gaps = 0U;
    uint64_t wire_old_sequences = 0U;

    uint64_t control_accepted = 0U;
    uint64_t control_overwritten = 0U;
    uint64_t control_encoded = 0U;
    uint64_t control_sent = 0U;
    uint64_t valid_control_sent = 0U;
    uint64_t invalid_control_sent = 0U;
    uint64_t control_invalidated_age = 0U;

    uint64_t hello_sent = 0U;
    uint64_t hello_ack_received = 0U;
    uint64_t heartbeat_sent = 0U;
    uint64_t status_received = 0U;
    uint64_t ping_sent = 0U;
    uint64_t pong_received = 0U;

    uint64_t reliable_requests = 0U;
    uint64_t reliable_sent = 0U;
    uint64_t reliable_retries = 0U;
    uint64_t reliable_acks = 0U;
    uint64_t reliable_nacks = 0U;
    uint64_t reliable_timeouts = 0U;
    uint64_t reliable_retry_exhausted = 0U;
    uint64_t reliable_failures = 0U;

    uint64_t remote_stop_requests = 0U;
    uint64_t clear_stop_requests = 0U;

    uint64_t response_windows_opened = 0U;
    uint64_t response_window_timeouts = 0U;

    uint64_t peer_boot_changes = 0U;
    uint64_t link_losses = 0U;
    uint64_t link_recoveries = 0U;

    ParserStats parser;
};

struct UartModuleSnapshot {
    LinkState state = LinkState::CLOSED;
    UartMetrics metrics;
    std::optional<Status> last_status;

    uint32_t local_boot_id = 0U;
    uint32_t peer_boot_id = 0U;
    bool peer_boot_id_valid = false;
    uint32_t next_wire_sequence = 0U;

    bool reliable_busy = false;
    bool response_window_open = false;
    std::string last_error;
};

class UartLink {
public:
    explicit UartLink(UartLinkConfig config);
    ~UartLink();

    UartLink(const UartLink&) = delete;
    UartLink& operator=(const UartLink&) = delete;

    [[nodiscard]] bool Start(std::string* error) noexcept;
    void Stop() noexcept;

    // Capacity-one latest-value submission. Success means accepted into the
    // bounded mailbox only; this call never performs serial I/O or waits for a
    // response from the MCU.
    [[nodiscard]] bool SubmitLatestControl(
        const ControlUpdateInput& input) noexcept;

    [[nodiscard]] bool RequestRemoteStop(uint16_t reason_code,
                                         uint32_t* transaction_id) noexcept;
    [[nodiscard]] bool RequestClearRemoteStop(
        uint32_t* transaction_id) noexcept;
    [[nodiscard]] bool RequestPing(uint32_t* ping_id) noexcept;

    [[nodiscard]] UartModuleSnapshot GetSnapshot() const noexcept;
    [[nodiscard]] UartMetrics GetMetrics() const noexcept;
    [[nodiscard]] LinkState GetLinkState() const noexcept;
    [[nodiscard]] bool IsRunning() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace visionarm::uart
