#include "metrics/v7_control_trace_recorder.h"

#include <chrono>
#include <iomanip>
#include <stdexcept>
#include <utility>

namespace visionarm {

V7ControlTraceRecorder::V7ControlTraceRecorder(
    IControlSink& downstream,
    std::string csv_path)
    : downstream_(&downstream),
      output_(std::move(csv_path), std::ios::out | std::ios::trunc) {
    if (!output_) {
        throw std::runtime_error("failed to open V7 control trace CSV");
    }

    output_
        << "capture_session_id,frame_id,v4l2_sequence,target_state,valid,"
        << "confidence,dx_px,dy_px,error_x,error_y,capture_timestamp_ns,"
        << "generated_timestamp_ns,age_ns,consecutive_hits,"
        << "consecutive_misses,downstream_accepted\n";
    output_.flush();
    if (!output_) {
        throw std::runtime_error("failed to initialize V7 control trace CSV");
    }

    writer_ = std::thread(&V7ControlTraceRecorder::WriterMain, this);
}

V7ControlTraceRecorder::~V7ControlTraceRecorder() {
    Stop();
}

bool V7ControlTraceRecorder::Submit(const ControlResult& result) noexcept {
    submissions_.fetch_add(1U, std::memory_order_relaxed);

    bool accepted = false;
    try {
        accepted = downstream_ != nullptr && downstream_->Submit(result);
    } catch (...) {
        accepted = false;
    }

    if (accepted) {
        downstream_accepted_.fetch_add(1U, std::memory_order_relaxed);
    } else {
        downstream_rejected_.fetch_add(1U, std::memory_order_relaxed);
    }

    Record record;
    record.capture_session_id = result.identity.capture_session_id;
    record.frame_id = result.identity.frame_id;
    record.v4l2_sequence = result.identity.v4l2_sequence;
    record.target_state = static_cast<uint8_t>(result.state);
    record.valid = result.valid ? 1U : 0U;
    record.confidence = result.error.confidence;
    record.dx_px = result.error.dx_px;
    record.dy_px = result.error.dy_px;
    record.error_x = result.error.error_x_normalized;
    record.error_y = result.error.error_y_normalized;
    record.capture_timestamp_ns = result.capture_timestamp_ns;
    record.generated_timestamp_ns = result.generated_timestamp_ns;
    record.age_ns = result.age_ns;
    record.consecutive_hits = result.consecutive_hits;
    record.consecutive_misses = result.consecutive_misses;
    record.downstream_accepted = accepted ? 1U : 0U;

    if (TryEnqueue(record)) {
        enqueued_.fetch_add(1U, std::memory_order_relaxed);
    } else {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
    }

    return accepted;
}

void V7ControlTraceRecorder::Stop() noexcept {
    const bool was_stopping =
        stopping_.exchange(true, std::memory_order_acq_rel);
    if (!was_stopping && writer_.joinable()) {
        writer_.join();
    } else if (writer_.joinable()) {
        writer_.join();
    }

    if (output_.is_open()) {
        output_.flush();
        if (!output_) {
            write_errors_.fetch_add(1U, std::memory_order_relaxed);
        }
        output_.close();
    }
}

V7ControlTraceStats V7ControlTraceRecorder::Snapshot() const noexcept {
    V7ControlTraceStats stats;
    stats.submissions = submissions_.load(std::memory_order_relaxed);
    stats.downstream_accepted =
        downstream_accepted_.load(std::memory_order_relaxed);
    stats.downstream_rejected =
        downstream_rejected_.load(std::memory_order_relaxed);
    stats.enqueued = enqueued_.load(std::memory_order_relaxed);
    stats.dropped = dropped_.load(std::memory_order_relaxed);
    stats.written = written_.load(std::memory_order_relaxed);
    stats.write_errors = write_errors_.load(std::memory_order_relaxed);
    return stats;
}

void V7ControlTraceRecorder::WriterMain() noexcept {
    for (;;) {
        Record record;
        bool did_work = false;

        while (TryDequeue(&record)) {
            did_work = true;
            WriteRecord(record);
        }

        if (stopping_.load(std::memory_order_acquire) &&
            read_sequence_.load(std::memory_order_acquire) ==
                write_sequence_.load(std::memory_order_acquire)) {
            break;
        }

        if (!did_work) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    output_.flush();
    if (!output_) {
        write_errors_.fetch_add(1U, std::memory_order_relaxed);
    }
}

bool V7ControlTraceRecorder::TryEnqueue(const Record& record) noexcept {
    if (stopping_.load(std::memory_order_acquire)) {
        return false;
    }

    const uint64_t write = write_sequence_.load(std::memory_order_relaxed);
    const uint64_t read = read_sequence_.load(std::memory_order_acquire);
    if ((write - read) >= static_cast<uint64_t>(kQueueCapacity)) {
        return false;
    }

    queue_[static_cast<std::size_t>(write % kQueueCapacity)] = record;
    write_sequence_.store(write + 1U, std::memory_order_release);
    return true;
}

bool V7ControlTraceRecorder::TryDequeue(Record* record) noexcept {
    if (record == nullptr) {
        return false;
    }

    const uint64_t read = read_sequence_.load(std::memory_order_relaxed);
    const uint64_t write = write_sequence_.load(std::memory_order_acquire);
    if (read == write) {
        return false;
    }

    *record = queue_[static_cast<std::size_t>(read % kQueueCapacity)];
    read_sequence_.store(read + 1U, std::memory_order_release);
    return true;
}

void V7ControlTraceRecorder::WriteRecord(const Record& record) noexcept {
    output_ << record.capture_session_id << ','
            << record.frame_id << ','
            << record.v4l2_sequence << ','
            << static_cast<unsigned int>(record.target_state) << ','
            << static_cast<unsigned int>(record.valid) << ','
            << std::setprecision(9) << record.confidence << ','
            << record.dx_px << ','
            << record.dy_px << ','
            << record.error_x << ','
            << record.error_y << ','
            << record.capture_timestamp_ns << ','
            << record.generated_timestamp_ns << ','
            << record.age_ns << ','
            << record.consecutive_hits << ','
            << record.consecutive_misses << ','
            << static_cast<unsigned int>(record.downstream_accepted)
            << '\n';

    if (output_) {
        written_.fetch_add(1U, std::memory_order_relaxed);
    } else {
        write_errors_.fetch_add(1U, std::memory_order_relaxed);
        output_.clear();
    }
}

}  // namespace visionarm
