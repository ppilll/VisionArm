#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace visionarm {

enum class AudioSampleFormat : std::uint8_t {
    kS16LE = 0,
};

struct AudioStreamFormat {
    std::uint32_t sample_rate_hz = 0U;
    std::uint16_t channels = 0U;
    AudioSampleFormat sample_format = AudioSampleFormat::kS16LE;

    [[nodiscard]] std::uint16_t BytesPerSample() const noexcept {
        switch (sample_format) {
            case AudioSampleFormat::kS16LE:
                return 2U;
        }
        return 0U;
    }

    [[nodiscard]] std::uint32_t BytesPerFrame() const noexcept {
        return static_cast<std::uint32_t>(channels) * BytesPerSample();
    }

    [[nodiscard]] bool Valid() const noexcept {
        return sample_rate_hz != 0U && channels != 0U && BytesPerFrame() != 0U;
    }
};

// Timing evidence attached to one ALSA PCM read.
//
// IMPORTANT V8.3 CONTRACT:
// - first_sample_frame_index is the cumulative ALSA PCM frame index maintained
//   by this producer. A PCM frame contains one sample for every channel.
// - alsa_status_mono_ns is snd_pcm_status_get_htstamp() converted to ns. ALSA
//   defines it as the status "now" timestamp; it is NOT the PTS of the first
//   sample in this chunk.
// - delay_frames/avail_frames and the cumulative sample-frame index are kept
//   so the later MediaClock can construct the audio timeline without inventing
//   timestamps from snd_pcm_readi() return time.
struct AudioCaptureTiming {
    std::uint64_t chunk_sequence = 0U;
    std::uint64_t first_sample_frame_index = 0U;
    std::uint32_t frame_count = 0U;

    std::int64_t app_read_begin_mono_ns = 0;
    std::int64_t app_read_end_mono_ns = 0;

    bool alsa_status_valid = false;
    std::int64_t alsa_status_mono_ns = 0;
    std::int64_t alsa_trigger_mono_ns = 0;
    std::int64_t avail_frames = 0;
    std::int64_t delay_frames = 0;

    // True on the first delivered chunk after XRUN/suspend recovery. The
    // cumulative sample index remains monotonic, but the physical stream had
    // a discontinuity and MediaClock/encoder/mux must account for it.
    bool discontinuity_before = false;
    std::uint64_t recovery_sequence = 0U;
};

struct RawAudioChunk {
    AudioStreamFormat format;
    AudioCaptureTiming timing;
    std::vector<std::uint8_t> pcm;

    [[nodiscard]] std::size_t ExpectedBytes() const noexcept {
        return static_cast<std::size_t>(timing.frame_count) *
               static_cast<std::size_t>(format.BytesPerFrame());
    }

    [[nodiscard]] bool Valid() const noexcept {
        return format.Valid() && timing.frame_count != 0U &&
               pcm.size() == ExpectedBytes();
    }

    [[nodiscard]] std::int64_t NominalDurationNs() const {
        if (format.sample_rate_hz == 0U) {
            throw std::logic_error("RawAudioChunk has zero sample rate");
        }
        return static_cast<std::int64_t>(
            (static_cast<std::uint64_t>(timing.frame_count) * 1'000'000'000ULL) /
            static_cast<std::uint64_t>(format.sample_rate_hz));
    }
};

}  // namespace visionarm
