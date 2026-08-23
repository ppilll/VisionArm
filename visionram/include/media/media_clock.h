#pragma once

#include "audio/audio_chunk.h"

#include <cstdint>
#include <mutex>

namespace visionarm {

struct AudioMediaTiming {
    // PTS/duration are nanoseconds from the common monotonic media epoch.
    std::int64_t pts_ns = 0;
    std::int64_t duration_ns = 0;

    // The sample-clock prediction used for PTS, expressed in CLOCK_MONOTONIC.
    std::int64_t first_sample_mono_ns = 0;

    // Independent observation derived from ALSA status time + capture delay.
    // The difference from first_sample_mono_ns is a clock/jitter diagnostic;
    // it is not used to continuously retime otherwise-contiguous PCM.
    std::int64_t observed_first_sample_mono_ns = 0;
    std::int64_t timing_error_ns = 0;

    // True on the first chunk and after an ALSA discontinuity. A re-anchor
    // preserves real monotonic time across XRUN/suspend gaps instead of
    // compressing the missing physical audio interval.
    bool reanchored = false;
};

struct TimedAudioChunk {
    RawAudioChunk raw;
    AudioMediaTiming media;

    [[nodiscard]] bool Valid() const noexcept {
        return raw.Valid() && media.duration_ns > 0;
    }
};

struct MediaClockSnapshot {
    std::int64_t media_epoch_monotonic_ns = 0;

    bool audio_anchor_valid = false;
    std::uint64_t audio_anchor_sample_frame_index = 0U;
    std::int64_t audio_anchor_monotonic_ns = 0;

    std::uint64_t audio_chunks_stamped = 0U;
    std::uint64_t audio_reanchors = 0U;
    std::uint64_t audio_discontinuities = 0U;
    std::uint64_t audio_timestamp_failures = 0U;

    std::int64_t latest_audio_timing_error_ns = 0;
    std::int64_t maximum_abs_audio_timing_error_ns = 0;
    std::int64_t last_audio_pts_ns = 0;
    std::int64_t last_audio_end_pts_ns = 0;
};

// Common CLOCK_MONOTONIC based timeline for V8.3.
//
// Video contract:
//   video_pts_ns = v4l2_capture_monotonic_ns - media_epoch_monotonic_ns
//
// Audio contract:
//   1. ALSA status timestamp is only a monotonic timing observation.
//   2. For capture, status.delay_frames is the distance between the current
//      application position and the hardware/sound position.
//   3. After Read() returns one chunk, the first frame of that chunk is
//      estimated at:
//        status_now - (delay_frames + chunk_frames) / sample_rate
//   4. That observation establishes the first anchor. Normal chunks advance
//      strictly from cumulative PCM sample-frame count, avoiding per-period
//      timestamp jitter. XRUN/suspend discontinuities re-anchor to monotonic
//      time so lost physical time is represented as a PTS gap.
class MediaClock final {
public:
    explicit MediaClock(std::int64_t media_epoch_monotonic_ns);

    MediaClock(const MediaClock&) = delete;
    MediaClock& operator=(const MediaClock&) = delete;

    [[nodiscard]] std::int64_t epoch_monotonic_ns() const noexcept {
        return media_epoch_monotonic_ns_;
    }

    [[nodiscard]] std::int64_t VideoPtsNs(
        std::int64_t capture_monotonic_ns) const;

    [[nodiscard]] TimedAudioChunk StampAudio(RawAudioChunk chunk);

    [[nodiscard]] MediaClockSnapshot Snapshot() const noexcept;

private:
    [[nodiscard]] static std::int64_t FramesToNs(
        std::uint64_t frames,
        std::uint32_t sample_rate_hz);

    [[nodiscard]] std::int64_t PredictAudioMonoNs(
        std::uint64_t sample_frame_index,
        std::uint32_t sample_rate_hz) const;

    const std::int64_t media_epoch_monotonic_ns_;

    mutable std::mutex mutex_;
    MediaClockSnapshot snapshot_;
};

}  // namespace visionarm
