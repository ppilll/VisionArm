#include "audio/audio_capture_worker.h"

#include <exception>
#include <stdexcept>
#include <utility>

namespace visionarm {

AudioCaptureWorker::AudioCaptureWorker(
    AlsaCaptureConfig config,
    MediaClock* media_clock,
    BoundedQueue<TimedAudioChunk>* output_queue)
    : config_(std::move(config)),
      media_clock_(media_clock),
      output_queue_(output_queue) {
    if (media_clock_ == nullptr || output_queue_ == nullptr) {
        throw std::invalid_argument(
            "AudioCaptureWorker requires MediaClock and output queue");
    }
}

AudioCaptureWorker::~AudioCaptureWorker() {
    Stop();
}

bool AudioCaptureWorker::Start(std::string* error) {
    if (thread_.joinable()) {
        if (error != nullptr) {
            *error = "AudioCaptureWorker already started";
        }
        return false;
    }

    stop_requested_.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        startup_done_ = false;
        startup_success_ = false;
        snapshot_ = {};
    }

    thread_ = std::thread(&AudioCaptureWorker::ThreadMain, this);

    std::unique_lock<std::mutex> lock(mutex_);
    startup_cv_.wait(lock, [this] { return startup_done_; });
    if (!startup_success_) {
        const std::string startup_error = snapshot_.last_error;
        lock.unlock();
        if (thread_.joinable()) {
            thread_.join();
        }
        if (error != nullptr) {
            *error = startup_error;
        }
        return false;
    }
    return true;
}

void AudioCaptureWorker::Stop() noexcept {
    stop_requested_.store(true, std::memory_order_release);
    if (output_queue_ != nullptr) {
        output_queue_->Stop();
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

AudioCaptureWorkerSnapshot AudioCaptureWorker::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    AudioCaptureWorkerSnapshot copy = snapshot_;
    copy.running = running_.load(std::memory_order_acquire);
    return copy;
}

void AudioCaptureWorker::PublishStartup(
    bool success,
    const std::string& error) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        startup_success_ = success;
        startup_done_ = true;
        snapshot_.started = success;
        if (!success) {
            snapshot_.fatal_error = true;
            snapshot_.last_error = error;
        }
    }
    startup_cv_.notify_all();
}

void AudioCaptureWorker::PublishFatal(const std::string& error) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.fatal_error = true;
    snapshot_.last_error = error;
}

void AudioCaptureWorker::ThreadMain() noexcept {
    AlsaCapture capture(config_);
    bool startup_published = false;
    try {
        capture.Open();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.capture_info = capture.Info();
        }
        running_.store(true, std::memory_order_release);
        PublishStartup(true, {});
        startup_published = true;

        while (!stop_requested_.load(std::memory_order_acquire)) {
            RawAudioChunk raw;
            if (!capture.Read(&raw)) {
                continue;
            }

            TimedAudioChunk timed = media_clock_->StampAudio(std::move(raw));
            const std::uint32_t frames = timed.raw.timing.frame_count;
            const std::size_t bytes = timed.raw.pcm.size();

            if (!output_queue_->WaitPush(std::move(timed))) {
                if (!stop_requested_.load(std::memory_order_acquire)) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++snapshot_.queue_push_failures;
                }
                break;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                ++snapshot_.timed_chunks;
                snapshot_.timed_frames += frames;
                snapshot_.timed_bytes += bytes;
                snapshot_.capture = capture.Snapshot();
            }
        }
    } catch (const std::exception& error) {
        if (!startup_published) {
            PublishStartup(false, error.what());
            startup_published = true;
        } else {
            PublishFatal(error.what());
        }
    } catch (...) {
        if (!startup_published) {
            PublishStartup(false, "unknown audio capture exception");
            startup_published = true;
        } else {
            PublishFatal("unknown audio capture exception");
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.capture = capture.Snapshot();
    }
    capture.Close();
    running_.store(false, std::memory_order_release);
    output_queue_->Stop();

    // If an exception happened before PublishStartup() was reached, make sure
    // Start() cannot remain blocked forever.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!startup_done_) {
            startup_done_ = true;
            startup_success_ = false;
            snapshot_.fatal_error = true;
            snapshot_.last_error = "audio worker terminated during startup";
        }
    }
    startup_cv_.notify_all();
}

}  // namespace visionarm
