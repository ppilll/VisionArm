#pragma once

#include "audio/audio_types.h"
#include "media/media_clock.h"
#include "pipeline/bounded_queue.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace visionarm {

struct AlsaCaptureConfig {
    std::string device = "hw:1,0";
    std::uint32_t sample_rate_hz = 48'000U;
    std::uint16_t channels = 2U;
    AudioSampleFormat sample_format = AudioSampleFormat::kS16LE;
    std::uint32_t period_frames = 1'024U;
    std::uint32_t buffer_frames = 4'096U;

    // V8.3 product path should not silently run with a different rate/channel/
    // period/buffer than the media pipeline was configured for.
    bool require_exact_hw_params = true;
};

struct AlsaCaptureInfo {
    AudioStreamFormat format;
    std::uint32_t period_frames = 0U;
    std::uint32_t buffer_frames = 0U;
    std::string device;
    std::string timestamp_type;
};

struct AlsaCaptureSnapshot {
    std::uint64_t chunks = 0U;
    std::uint64_t delivered_frames = 0U;
    std::uint64_t delivered_bytes = 0U;
    std::uint64_t xrun_count = 0U;
    std::uint64_t suspend_count = 0U;
    std::uint64_t recovery_count = 0U;
    std::uint64_t short_read_count = 0U;
    std::uint64_t eagain_count = 0U;
    std::uint64_t status_error_count = 0U;
};

// Synchronous ALSA PCM producer extracted from the frozen V2A test program.
//
// Threading contract:
// - Open(), Read() and Close() are intended to be called by the owning audio
//   capture thread.
// - Snapshot() may be called from another thread.
// - Read() owns no Camera/DMA-BUF resources. RawAudioChunk owns its PCM bytes.
//
// Timestamp contract:
// - CLOCK_MONOTONIC is mandatory for V8.3.
// - Read() does not assign media PTS. It exports sample-frame indices and ALSA
//   timing evidence for the later MediaClock stage.
class AlsaCapture final {
public:
    explicit AlsaCapture(AlsaCaptureConfig config);
    ~AlsaCapture();

    AlsaCapture(const AlsaCapture&) = delete;
    AlsaCapture& operator=(const AlsaCapture&) = delete;
    AlsaCapture(AlsaCapture&&) = delete;
    AlsaCapture& operator=(AlsaCapture&&) = delete;

    void Open();
    void Close() noexcept;

    [[nodiscard]] bool IsOpen() const noexcept;
    [[nodiscard]] const AlsaCaptureInfo& Info() const;

    // Blocks until one PCM chunk is available. XRUN/suspend recovery is
    // performed internally. The first chunk after recovery is marked with
    // timing.discontinuity_before=true.
    //
    // Returns false only if the blocking ALSA read was interrupted by EINTR;
    // the caller can check its own shutdown flag and call Read() again if the
    // interruption was unrelated to shutdown.
    bool Read(RawAudioChunk* chunk);

    [[nodiscard]] AlsaCaptureSnapshot Snapshot() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

struct AudioCaptureWorkerSnapshot {
    bool started = false;
    bool running = false;
    bool fatal_error = false;
    std::string last_error;
    AlsaCaptureInfo capture_info;
    AlsaCaptureSnapshot capture;
    std::uint64_t timed_chunks = 0U;
    std::uint64_t timed_frames = 0U;
    std::uint64_t timed_bytes = 0U;
    std::uint64_t queue_push_failures = 0U;
};

// Owns the ALSA PCM handle and keeps PCM backpressure explicit: WaitPush()
// never replaces an older chunk.
class AudioCaptureWorker final {
public:
    AudioCaptureWorker(
        AlsaCaptureConfig config,
        MediaClock* media_clock,
        BoundedQueue<TimedAudioChunk>* output_queue);
    ~AudioCaptureWorker();
    AudioCaptureWorker(const AudioCaptureWorker&) = delete;
    AudioCaptureWorker& operator=(const AudioCaptureWorker&) = delete;
    bool Start(std::string* error);
    void Stop() noexcept;
    [[nodiscard]] bool running() const noexcept {
        return running_.load(std::memory_order_acquire);
    }
    [[nodiscard]] AudioCaptureWorkerSnapshot Snapshot() const;

private:
    void ThreadMain() noexcept;
    void PublishStartup(bool success, const std::string& error);
    void PublishFatal(const std::string& error) noexcept;
    const AlsaCaptureConfig config_;
    MediaClock* media_clock_ = nullptr;
    BoundedQueue<TimedAudioChunk>* output_queue_ = nullptr;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    std::condition_variable startup_cv_;
    bool startup_done_ = false;
    bool startup_success_ = false;
    AudioCaptureWorkerSnapshot snapshot_;
};

}  // namespace visionarm
