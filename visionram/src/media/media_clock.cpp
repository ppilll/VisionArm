#include "media/media_clock.h"

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <utility>

namespace visionarm {
namespace {

[[nodiscard]] std::int64_t AbsNs(std::int64_t value) noexcept {
    if (value == std::numeric_limits<std::int64_t>::min()) {
        return std::numeric_limits<std::int64_t>::max();
    }
    return value < 0 ? -value : value;
}

}  // namespace

MediaClock::MediaClock(std::int64_t media_epoch_monotonic_ns)
    : media_epoch_monotonic_ns_(media_epoch_monotonic_ns) {
    if (media_epoch_monotonic_ns_ <= 0) {
        throw std::invalid_argument(
            "MediaClock epoch must be a positive CLOCK_MONOTONIC timestamp");
    }
    snapshot_.media_epoch_monotonic_ns = media_epoch_monotonic_ns_;
}

std::int64_t MediaClock::FramesToNs(
    std::uint64_t frames,
    std::uint32_t sample_rate_hz) {
    if (sample_rate_hz == 0U) {
        throw std::invalid_argument("sample rate must be non-zero");
    }

    constexpr std::uint64_t kNsPerSecond = 1'000'000'000ULL;
    const std::uint64_t whole_seconds = frames / sample_rate_hz;
    const std::uint64_t remainder_frames = frames % sample_rate_hz;

    if (whole_seconds >
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) /
            kNsPerSecond) {
        throw std::overflow_error("audio sample-frame timestamp overflow");
    }

    const std::uint64_t ns =
        whole_seconds * kNsPerSecond +
        (remainder_frames * kNsPerSecond) / sample_rate_hz;
    if (ns > static_cast<std::uint64_t>(
                 std::numeric_limits<std::int64_t>::max())) {
        throw std::overflow_error("audio nanosecond timestamp overflow");
    }
    return static_cast<std::int64_t>(ns);
}

std::int64_t MediaClock::VideoPtsNs(
    std::int64_t capture_monotonic_ns) const {
    if (capture_monotonic_ns < media_epoch_monotonic_ns_) {
        throw std::invalid_argument(
            "video capture timestamp precedes media epoch");
    }
    return capture_monotonic_ns - media_epoch_monotonic_ns_;
}

std::int64_t MediaClock::PredictAudioMonoNs(
    std::uint64_t sample_frame_index,
    std::uint32_t sample_rate_hz) const {
    if (!snapshot_.audio_anchor_valid ||
        sample_frame_index < snapshot_.audio_anchor_sample_frame_index) {
        throw std::logic_error("invalid audio sample-clock anchor");
    }
    return snapshot_.audio_anchor_monotonic_ns + FramesToNs(
        sample_frame_index - snapshot_.audio_anchor_sample_frame_index,
        sample_rate_hz);
}

TimedAudioChunk MediaClock::StampAudio(RawAudioChunk chunk) {
    if (!chunk.Valid()) {
        throw std::invalid_argument("MediaClock received invalid RawAudioChunk");
    }
    if (!chunk.timing.alsa_status_valid) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.audio_timestamp_failures;
        throw std::runtime_error(
            "ALSA status timestamp is required for V8.3 audio PTS");
    }
    if (chunk.timing.alsa_status_mono_ns <= 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.audio_timestamp_failures;
        throw std::runtime_error("invalid ALSA monotonic status timestamp");
    }
    if (chunk.timing.delay_frames < 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.audio_timestamp_failures;
        throw std::runtime_error(
            "negative ALSA capture delay is unsupported by V8.3 timeline");
    }

    const std::uint64_t delay_frames =
        static_cast<std::uint64_t>(chunk.timing.delay_frames);
    const std::uint64_t chunk_frames = chunk.timing.frame_count;
    if (delay_frames >
        std::numeric_limits<std::uint64_t>::max() - chunk_frames) {
        throw std::overflow_error("ALSA delay frame count overflow");
    }

    const std::int64_t observed_first_sample_mono_ns =
        chunk.timing.alsa_status_mono_ns - FramesToNs(
            delay_frames + chunk_frames,
            chunk.format.sample_rate_hz);

    std::lock_guard<std::mutex> lock(mutex_);

    const bool reanchor =
        !snapshot_.audio_anchor_valid || chunk.timing.discontinuity_before;
    if (reanchor) {
        snapshot_.audio_anchor_valid = true;
        snapshot_.audio_anchor_sample_frame_index =
            chunk.timing.first_sample_frame_index;
        snapshot_.audio_anchor_monotonic_ns = observed_first_sample_mono_ns;
        ++snapshot_.audio_reanchors;
        if (chunk.timing.discontinuity_before) {
            ++snapshot_.audio_discontinuities;
        }
    }

    const std::int64_t first_sample_mono_ns = PredictAudioMonoNs(
        chunk.timing.first_sample_frame_index,
        chunk.format.sample_rate_hz);
    const std::uint64_t end_sample_index =
        chunk.timing.first_sample_frame_index + chunk.timing.frame_count;
    if (end_sample_index < chunk.timing.first_sample_frame_index) {
        throw std::overflow_error("audio sample-frame index overflow");
    }
    const std::int64_t end_sample_mono_ns = PredictAudioMonoNs(
        end_sample_index,
        chunk.format.sample_rate_hz);

    if (first_sample_mono_ns < media_epoch_monotonic_ns_) {
        ++snapshot_.audio_timestamp_failures;
        throw std::runtime_error(
            "audio first-sample timestamp precedes media epoch");
    }
    if (end_sample_mono_ns <= first_sample_mono_ns) {
        ++snapshot_.audio_timestamp_failures;
        throw std::runtime_error("non-positive audio chunk duration");
    }

    const std::int64_t pts_ns =
        first_sample_mono_ns - media_epoch_monotonic_ns_;
    const std::int64_t end_pts_ns =
        end_sample_mono_ns - media_epoch_monotonic_ns_;

    // During continuous capture the sample index must never regress and the
    // generated PTS must not overlap the previous audio interval. After a
    // discontinuity a forward gap is valid and intentionally preserved.
    if (snapshot_.audio_chunks_stamped > 0U &&
        pts_ns < snapshot_.last_audio_end_pts_ns) {
        ++snapshot_.audio_timestamp_failures;
        throw std::runtime_error("audio PTS regressed/overlapped");
    }

    const std::int64_t timing_error_ns =
        observed_first_sample_mono_ns - first_sample_mono_ns;
    snapshot_.latest_audio_timing_error_ns = timing_error_ns;
    snapshot_.maximum_abs_audio_timing_error_ns = std::max(
        snapshot_.maximum_abs_audio_timing_error_ns,
        AbsNs(timing_error_ns));
    snapshot_.last_audio_pts_ns = pts_ns;
    snapshot_.last_audio_end_pts_ns = end_pts_ns;
    ++snapshot_.audio_chunks_stamped;

    TimedAudioChunk timed;
    timed.raw = std::move(chunk);
    timed.media.pts_ns = pts_ns;
    timed.media.duration_ns = end_pts_ns - pts_ns;
    timed.media.first_sample_mono_ns = first_sample_mono_ns;
    timed.media.observed_first_sample_mono_ns =
        observed_first_sample_mono_ns;
    timed.media.timing_error_ns = timing_error_ns;
    timed.media.reanchored = reanchor;
    return timed;
}

MediaClockSnapshot MediaClock::Snapshot() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

}  // namespace visionarm
