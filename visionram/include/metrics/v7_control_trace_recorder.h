#pragma once

#include "control/control_sink.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>

namespace visionarm {

struct V7ControlTraceStats {
    uint64_t submissions = 0U;
    uint64_t downstream_accepted = 0U;
    uint64_t downstream_rejected = 0U;
    uint64_t enqueued = 0U;
    uint64_t dropped = 0U;
    uint64_t written = 0U;
    uint64_t write_errors = 0U;
};

/*
 * V7 test instrumentation decorator.
 *
 * The producer path performs only fixed-size record construction and one
 * bounded SPSC-ring enqueue after forwarding to the existing control sink.
 * CSV formatting and file I/O live exclusively on the writer thread.  Queue
 * full means trace drop, never control-path blocking.
 */
class V7ControlTraceRecorder final : public IControlSink {
public:
    V7ControlTraceRecorder(IControlSink& downstream, std::string csv_path);
    ~V7ControlTraceRecorder() override;

    V7ControlTraceRecorder(const V7ControlTraceRecorder&) = delete;
    V7ControlTraceRecorder& operator=(const V7ControlTraceRecorder&) = delete;

    bool Submit(const ControlResult& result) noexcept override;
    void Stop() noexcept;
    [[nodiscard]] V7ControlTraceStats Snapshot() const noexcept;

private:
    struct Record {
        uint64_t capture_session_id = 0U;
        uint64_t frame_id = 0U;
        uint32_t v4l2_sequence = 0U;
        uint8_t target_state = 0U;
        uint8_t valid = 0U;
        float confidence = 0.0F;
        float dx_px = 0.0F;
        float dy_px = 0.0F;
        float error_x = 0.0F;
        float error_y = 0.0F;
        int64_t capture_timestamp_ns = 0;
        int64_t generated_timestamp_ns = 0;
        int64_t age_ns = 0;
        uint32_t consecutive_hits = 0U;
        uint32_t consecutive_misses = 0U;
        uint8_t downstream_accepted = 0U;
    };

    static constexpr std::size_t kQueueCapacity = 4096U;

    void WriterMain() noexcept;
    bool TryEnqueue(const Record& record) noexcept;
    bool TryDequeue(Record* record) noexcept;
    void WriteRecord(const Record& record) noexcept;

    IControlSink* downstream_ = nullptr;
    std::ofstream output_;
    std::thread writer_;
    std::atomic<bool> stopping_{false};
    std::array<Record, kQueueCapacity> queue_{};
    std::atomic<uint64_t> write_sequence_{0U};
    std::atomic<uint64_t> read_sequence_{0U};

    std::atomic<uint64_t> submissions_{0U};
    std::atomic<uint64_t> downstream_accepted_{0U};
    std::atomic<uint64_t> downstream_rejected_{0U};
    std::atomic<uint64_t> enqueued_{0U};
    std::atomic<uint64_t> dropped_{0U};
    std::atomic<uint64_t> written_{0U};
    std::atomic<uint64_t> write_errors_{0U};
};

}  // namespace visionarm
