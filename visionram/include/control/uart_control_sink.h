#pragma once

#include "control/control_sink.h"
#include "control/uart_link.h"

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>

namespace visionarm {

struct UartControlSinkMetrics {
    uint64_t submissions = 0U;
    uint64_t accepted = 0U;
    uint64_t rejected = 0U;

    uint64_t valid_inputs = 0U;
    uint64_t invalid_inputs = 0U;
    uint64_t nonfinite_invalidations = 0U;
    uint64_t invalid_timestamp_invalidations = 0U;
    uint64_t invalid_state_invalidations = 0U;
    uint64_t identity_truncations = 0U;

    std::array<uint64_t, 6U> state_counts{};
};

struct UartControlSinkSnapshot {
    UartControlSinkMetrics metrics;
    std::optional<uart::ControlUpdateInput> latest_input;
};

// Adapter from the control domain to the UART public API.
//
// This class owns no thread, tty fd or UartLink lifetime. Submit() performs
// only bounded value conversion plus UartLink::SubmitLatestControl(); it never
// performs serial I/O and never waits for an MCU response.
class UartControlSink final : public IControlSink {
public:
    explicit UartControlSink(uart::UartLink& link) noexcept;

    bool Submit(const ControlResult& result) noexcept override;
    [[nodiscard]] UartControlSinkSnapshot Snapshot() const noexcept;

private:
    uart::UartLink* link_ = nullptr;
    mutable std::mutex mutex_;
    UartControlSinkSnapshot snapshot_;
};

}  // namespace visionarm
