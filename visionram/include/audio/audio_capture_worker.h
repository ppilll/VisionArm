#pragma once

#include "audio/alsa_capture.h"
#include "media/media_clock.h"
#include "pipeline/bounded_queue.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace visionarm {

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

// Owns the ALSA PCM handle and is the only thread that calls Open/Read/Close.
// PCM is never dropped to make room in the queue: WaitPush() applies bounded
// backpressure. If downstream blocks long enough, ALSA XRUN is surfaced by
// AlsaCapture and the next chunk is marked as a discontinuity.
class AudioCaptureWorker final {
public:
    AudioCaptureWorker(
        AlsaCaptureConfig config,
        MediaClock* media_clock,
        BoundedQueue<TimedAudioChunk>* output_queue);
    ~AudioCaptureWorker();

    AudioCaptureWorker(const AudioCaptureWorker&) = delete;
    AudioCaptureWorker& operator=(const AudioCaptureWorker&) = delete;

    // Waits until the worker either opens/configures ALSA successfully or
    // reports a startup error. Returns false on startup failure.
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
