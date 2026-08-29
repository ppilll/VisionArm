#include "audio/audio_capture.h"

#include <alsa/asoundlib.h>

#include <cerrno>
#include <cstring>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <time.h>

namespace visionarm {
namespace {

[[nodiscard]] std::int64_t MonotonicNowNs() {
    timespec now{};
    if (::clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        throw std::runtime_error(
            std::string("clock_gettime(CLOCK_MONOTONIC): ") +
            std::strerror(errno));
    }
    return static_cast<std::int64_t>(now.tv_sec) * 1'000'000'000LL +
           static_cast<std::int64_t>(now.tv_nsec);
}

[[nodiscard]] std::int64_t HtimestampToNs(const snd_htimestamp_t& timestamp) noexcept {
    return static_cast<std::int64_t>(timestamp.tv_sec) * 1'000'000'000LL +
           static_cast<std::int64_t>(timestamp.tv_nsec);
}

void CheckAlsa(int result, const std::string& operation) {
    if (result < 0) {
        throw std::runtime_error(
            operation + ": " + snd_strerror(result) +
            " (" + std::to_string(result) + ")");
    }
}

[[nodiscard]] snd_pcm_format_t ToAlsaFormat(AudioSampleFormat format) {
    switch (format) {
        case AudioSampleFormat::kS16LE:
            return SND_PCM_FORMAT_S16_LE;
    }
    return SND_PCM_FORMAT_UNKNOWN;
}

[[nodiscard]] const char* TimestampTypeName(snd_pcm_tstamp_type_t type) noexcept {
    switch (type) {
        case SND_PCM_TSTAMP_TYPE_MONOTONIC:
            return "MONOTONIC";
        case SND_PCM_TSTAMP_TYPE_MONOTONIC_RAW:
            return "MONOTONIC_RAW";
        case SND_PCM_TSTAMP_TYPE_GETTIMEOFDAY:
            return "GETTIMEOFDAY";
        default:
            return "UNKNOWN";
    }
}

}  // namespace

class AlsaCapture::Impl final {
public:
    explicit Impl(AlsaCaptureConfig config)
        : config_(std::move(config)) {}

    ~Impl() {
        Close();
    }

    void Open() {
        if (pcm_ != nullptr) {
            throw std::logic_error("AlsaCapture is already open");
        }
        ValidateConfig();

        CheckAlsa(
            snd_pcm_open(
                &pcm_,
                config_.device.c_str(),
                SND_PCM_STREAM_CAPTURE,
                0),
            "snd_pcm_open(" + config_.device + ")");

        try {
            CheckAlsa(snd_pcm_status_malloc(&status_), "snd_pcm_status_malloc");
            ConfigureHardware();
            ConfigureSoftware();
            CheckAlsa(snd_pcm_prepare(pcm_), "snd_pcm_prepare");
        } catch (...) {
            Close();
            throw;
        }
    }

    void Close() noexcept {
        if (status_ != nullptr) {
            snd_pcm_status_free(status_);
            status_ = nullptr;
        }
        if (pcm_ != nullptr) {
            (void)snd_pcm_drop(pcm_);
            (void)snd_pcm_close(pcm_);
            pcm_ = nullptr;
        }
    }

    [[nodiscard]] bool IsOpen() const noexcept {
        return pcm_ != nullptr;
    }

    [[nodiscard]] const AlsaCaptureInfo& Info() const {
        if (pcm_ == nullptr) {
            throw std::logic_error("AlsaCapture::Info called before Open");
        }
        return info_;
    }

    bool Read(RawAudioChunk* chunk) {
        if (chunk == nullptr) {
            throw std::invalid_argument("AlsaCapture::Read chunk is null");
        }
        if (pcm_ == nullptr) {
            throw std::logic_error("AlsaCapture::Read called before Open");
        }

        const std::size_t requested_bytes =
            static_cast<std::size_t>(info_.period_frames) *
            static_cast<std::size_t>(info_.format.BytesPerFrame());
        chunk->pcm.resize(requested_bytes);

        for (;;) {
            const std::int64_t read_begin_ns = MonotonicNowNs();
            const snd_pcm_sframes_t result =
                snd_pcm_readi(pcm_, chunk->pcm.data(), info_.period_frames);
            const std::int64_t read_end_ns = MonotonicNowNs();

            if (result == -EINTR) {
                chunk->pcm.clear();
                return false;
            }
            if (result == -EAGAIN) {
                IncrementEagain();
                continue;
            }
            if (result == -EPIPE) {
                IncrementXrun();
                Recover(static_cast<int>(result));
                pending_discontinuity_ = true;
                continue;
            }
            if (result == -ESTRPIPE) {
                IncrementSuspend();
                Recover(static_cast<int>(result));
                pending_discontinuity_ = true;
                continue;
            }
            if (result < 0) {
                throw std::runtime_error(
                    "snd_pcm_readi: " +
                    std::string(snd_strerror(static_cast<int>(result))));
            }
            if (result == 0) {
                continue;
            }

            const std::uint32_t frames_read =
                static_cast<std::uint32_t>(result);
            if (frames_read < info_.period_frames) {
                IncrementShortRead();
            }

            const std::size_t bytes_read =
                static_cast<std::size_t>(frames_read) *
                static_cast<std::size_t>(info_.format.BytesPerFrame());
            chunk->pcm.resize(bytes_read);
            chunk->format = info_.format;

            AudioCaptureTiming timing;
            timing.chunk_sequence = next_chunk_sequence_++;
            timing.first_sample_frame_index = delivered_frames_;
            timing.frame_count = frames_read;
            timing.app_read_begin_mono_ns = read_begin_ns;
            timing.app_read_end_mono_ns = read_end_ns;
            timing.discontinuity_before = pending_discontinuity_;
            timing.recovery_sequence = recovery_sequence_;
            pending_discontinuity_ = false;

            PopulateStatus(&timing);
            chunk->timing = timing;

            delivered_frames_ += frames_read;
            UpdateDelivered(frames_read, bytes_read);
            return true;
        }
    }

    [[nodiscard]] AlsaCaptureSnapshot Snapshot() const noexcept {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return stats_;
    }

private:
    void ValidateConfig() const {
        if (config_.device.rfind("hw:", 0) != 0) {
            throw std::invalid_argument(
                "ALSA capture requires an hw: PCM; default/plughw are not allowed");
        }
        if (config_.sample_rate_hz == 0U || config_.channels == 0U ||
            config_.period_frames == 0U || config_.buffer_frames == 0U) {
            throw std::invalid_argument("Invalid zero-valued ALSA capture parameter");
        }
        if (config_.buffer_frames < config_.period_frames) {
            throw std::invalid_argument("buffer_frames must be >= period_frames");
        }
        if (ToAlsaFormat(config_.sample_format) == SND_PCM_FORMAT_UNKNOWN) {
            throw std::invalid_argument("Unsupported AudioSampleFormat");
        }
    }

    void ConfigureHardware() {
        snd_pcm_hw_params_t* hw = nullptr;
        snd_pcm_hw_params_alloca(&hw);

        CheckAlsa(snd_pcm_hw_params_any(pcm_, hw), "snd_pcm_hw_params_any");
        CheckAlsa(
            snd_pcm_hw_params_set_access(pcm_, hw, SND_PCM_ACCESS_RW_INTERLEAVED),
            "snd_pcm_hw_params_set_access(RW_INTERLEAVED)");
        CheckAlsa(
            snd_pcm_hw_params_set_format(pcm_, hw, ToAlsaFormat(config_.sample_format)),
            "snd_pcm_hw_params_set_format");

        unsigned int channels = config_.channels;
        CheckAlsa(
            snd_pcm_hw_params_set_channels_near(pcm_, hw, &channels),
            "snd_pcm_hw_params_set_channels_near");

        unsigned int rate = config_.sample_rate_hz;
        int rate_dir = 0;
        CheckAlsa(
            snd_pcm_hw_params_set_rate_near(pcm_, hw, &rate, &rate_dir),
            "snd_pcm_hw_params_set_rate_near");

        snd_pcm_uframes_t period = config_.period_frames;
        int period_dir = 0;
        CheckAlsa(
            snd_pcm_hw_params_set_period_size_near(pcm_, hw, &period, &period_dir),
            "snd_pcm_hw_params_set_period_size_near");

        snd_pcm_uframes_t buffer = config_.buffer_frames;
        CheckAlsa(
            snd_pcm_hw_params_set_buffer_size_near(pcm_, hw, &buffer),
            "snd_pcm_hw_params_set_buffer_size_near");
        CheckAlsa(snd_pcm_hw_params(pcm_, hw), "snd_pcm_hw_params");

        snd_pcm_format_t actual_format = SND_PCM_FORMAT_UNKNOWN;
        unsigned int actual_channels = 0U;
        unsigned int actual_rate = 0U;
        snd_pcm_uframes_t actual_period = 0U;
        snd_pcm_uframes_t actual_buffer = 0U;
        rate_dir = 0;
        period_dir = 0;

        CheckAlsa(snd_pcm_hw_params_get_format(hw, &actual_format),
                  "snd_pcm_hw_params_get_format");
        CheckAlsa(snd_pcm_hw_params_get_channels(hw, &actual_channels),
                  "snd_pcm_hw_params_get_channels");
        CheckAlsa(snd_pcm_hw_params_get_rate(hw, &actual_rate, &rate_dir),
                  "snd_pcm_hw_params_get_rate");
        CheckAlsa(snd_pcm_hw_params_get_period_size(hw, &actual_period, &period_dir),
                  "snd_pcm_hw_params_get_period_size");
        CheckAlsa(snd_pcm_hw_params_get_buffer_size(hw, &actual_buffer),
                  "snd_pcm_hw_params_get_buffer_size");

        if (actual_format != ToAlsaFormat(config_.sample_format)) {
            throw std::runtime_error("ALSA changed the requested sample format");
        }

        if (config_.require_exact_hw_params &&
            (actual_channels != config_.channels ||
             actual_rate != config_.sample_rate_hz ||
             actual_period != config_.period_frames ||
             actual_buffer != config_.buffer_frames)) {
            throw std::runtime_error(
                "ALSA adjusted requested hardware parameters; exact mode required: "
                "requested rate=" + std::to_string(config_.sample_rate_hz) +
                " channels=" + std::to_string(config_.channels) +
                " period=" + std::to_string(config_.period_frames) +
                " buffer=" + std::to_string(config_.buffer_frames) +
                ", actual rate=" + std::to_string(actual_rate) +
                " channels=" + std::to_string(actual_channels) +
                " period=" + std::to_string(actual_period) +
                " buffer=" + std::to_string(actual_buffer));
        }

        if (actual_channels > 65'535U || actual_period > 0xffff'ffffULL ||
            actual_buffer > 0xffff'ffffULL) {
            throw std::runtime_error("Negotiated ALSA parameter exceeds the supported type range");
        }

        info_.device = config_.device;
        info_.format.sample_rate_hz = actual_rate;
        info_.format.channels = static_cast<std::uint16_t>(actual_channels);
        info_.format.sample_format = config_.sample_format;
        info_.period_frames = static_cast<std::uint32_t>(actual_period);
        info_.buffer_frames = static_cast<std::uint32_t>(actual_buffer);
    }

    void ConfigureSoftware() {
        snd_pcm_sw_params_t* sw = nullptr;
        snd_pcm_sw_params_alloca(&sw);

        CheckAlsa(snd_pcm_sw_params_current(pcm_, sw),
                  "snd_pcm_sw_params_current");
        CheckAlsa(
            snd_pcm_sw_params_set_avail_min(pcm_, sw, info_.period_frames),
            "snd_pcm_sw_params_set_avail_min");
        CheckAlsa(
            snd_pcm_sw_params_set_tstamp_mode(pcm_, sw, SND_PCM_TSTAMP_ENABLE),
            "snd_pcm_sw_params_set_tstamp_mode");

        // Video capture timestamps and the media epoch use CLOCK_MONOTONIC.
        // No fallback to realtime or MONOTONIC_RAW is allowed here.
        CheckAlsa(
            snd_pcm_sw_params_set_tstamp_type(
                pcm_, sw, SND_PCM_TSTAMP_TYPE_MONOTONIC),
            "snd_pcm_sw_params_set_tstamp_type(MONOTONIC)");
        CheckAlsa(snd_pcm_sw_params(pcm_, sw), "snd_pcm_sw_params");

        CheckAlsa(snd_pcm_sw_params_current(pcm_, sw),
                  "snd_pcm_sw_params_current(confirm)");
        snd_pcm_tstamp_type_t actual_type = SND_PCM_TSTAMP_TYPE_GETTIMEOFDAY;
        CheckAlsa(snd_pcm_sw_params_get_tstamp_type(sw, &actual_type),
                  "snd_pcm_sw_params_get_tstamp_type");
        if (actual_type != SND_PCM_TSTAMP_TYPE_MONOTONIC) {
            throw std::runtime_error(
                "ALSA did not keep MONOTONIC timestamp type; actual=" +
                std::string(TimestampTypeName(actual_type)));
        }
        info_.timestamp_type = TimestampTypeName(actual_type);
    }

    void PopulateStatus(AudioCaptureTiming* timing) {
        const int result = snd_pcm_status(pcm_, status_);
        if (result < 0) {
            IncrementStatusError();
            timing->alsa_status_valid = false;
            return;
        }

        snd_htimestamp_t now{};
        snd_htimestamp_t trigger{};
        snd_pcm_status_get_htstamp(status_, &now);
        snd_pcm_status_get_trigger_htstamp(status_, &trigger);

        timing->alsa_status_valid = true;
        timing->alsa_status_mono_ns = HtimestampToNs(now);
        timing->alsa_trigger_mono_ns = HtimestampToNs(trigger);
        timing->avail_frames = static_cast<std::int64_t>(snd_pcm_status_get_avail(status_));
        timing->delay_frames = static_cast<std::int64_t>(snd_pcm_status_get_delay(status_));
    }

    void Recover(int error_code) {
        const int recovery = snd_pcm_recover(pcm_, error_code, 1);
        if (recovery < 0) {
            throw std::runtime_error(
                "snd_pcm_recover failed: " +
                std::string(snd_strerror(recovery)));
        }
        ++recovery_sequence_;
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.recovery_count;
    }

    void IncrementXrun() noexcept {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.xrun_count;
    }

    void IncrementSuspend() noexcept {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.suspend_count;
    }

    void IncrementShortRead() noexcept {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.short_read_count;
    }

    void IncrementEagain() noexcept {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.eagain_count;
    }

    void IncrementStatusError() noexcept {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.status_error_count;
    }

    void UpdateDelivered(std::uint32_t frames, std::size_t bytes) noexcept {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.chunks;
        stats_.delivered_frames += frames;
        stats_.delivered_bytes += bytes;
    }

    AlsaCaptureConfig config_;
    AlsaCaptureInfo info_;
    snd_pcm_t* pcm_ = nullptr;
    snd_pcm_status_t* status_ = nullptr;

    std::uint64_t next_chunk_sequence_ = 0U;
    std::uint64_t delivered_frames_ = 0U;
    std::uint64_t recovery_sequence_ = 0U;
    bool pending_discontinuity_ = false;

    mutable std::mutex stats_mutex_;
    AlsaCaptureSnapshot stats_;
};

AlsaCapture::AlsaCapture(AlsaCaptureConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

AlsaCapture::~AlsaCapture() = default;

void AlsaCapture::Open() {
    impl_->Open();
}

void AlsaCapture::Close() noexcept {
    impl_->Close();
}

bool AlsaCapture::IsOpen() const noexcept {
    return impl_->IsOpen();
}

const AlsaCaptureInfo& AlsaCapture::Info() const {
    return impl_->Info();
}

bool AlsaCapture::Read(RawAudioChunk* chunk) {
    return impl_->Read(chunk);
}

AlsaCaptureSnapshot AlsaCapture::Snapshot() const noexcept {
    return impl_->Snapshot();
}

}  // namespace visionarm

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
