#pragma once

#include <cstdint>
#include <vector>

namespace visionarm {

enum class EncodedAudioCodec : std::uint8_t {
    kAacLc = 0,
};

// Encoder-wide AAC stream metadata. codec_config is the codec extradata
// produced by libavcodec (AudioSpecificConfig for AAC when GLOBAL_HEADER is
// enabled). It is owned by this object and is intended to be copied into the
// later mux stream configuration.
struct AudioEncoderStreamInfo {
    EncodedAudioCodec codec = EncodedAudioCodec::kAacLc;
    std::uint32_t sample_rate_hz = 0U;
    std::uint16_t channels = 0U;
    std::int32_t bit_rate_bps = 0;
    std::uint32_t frame_size_frames = 0U;
    std::uint32_t initial_padding_frames = 0U;
    std::vector<std::uint8_t> codec_config;

    [[nodiscard]] bool Valid() const noexcept {
        return sample_rate_hz != 0U && channels != 0U && bit_rate_bps > 0 &&
               frame_size_frames != 0U && !codec_config.empty();
    }
};

// One independently-owned encoded AAC access unit.
//
// Timing contract:
// - pts_ns/dts_ns are the libavcodec output packet timestamps rescaled from the
//   codec time base to the common V8.3 media timeline. They intentionally keep
//   AAC encoder priming semantics; the first packet may therefore precede the
//   first source PCM sample by initial_padding_frames.
// - source_* describes the real PCM samples from TimedAudioChunk that overlap
//   this encoded packet. AAC priming/trailing-padding packets may contain zero
//   or fewer source frames than their decoded duration.
// - packet bytes are owned here. No PCM buffer, Camera FrameLease, or DMA-BUF
//   lifetime escapes through this contract.
struct EncodedAudioPacket {
    EncodedAudioCodec codec = EncodedAudioCodec::kAacLc;
    std::vector<std::uint8_t> data;

    std::int64_t pts_ns = 0;
    std::int64_t dts_ns = 0;
    std::int64_t duration_ns = 0;

    bool has_source_samples = false;
    std::int64_t source_pts_ns = 0;
    std::uint64_t source_first_sample_frame_index = 0U;
    std::uint32_t source_frame_count = 0U;

    bool discontinuity_before = false;
    bool contains_encoder_padding = false;

    // FFmpeg AV_PKT_DATA_SKIP_SAMPLES semantics, preserved across the
    // application-owned packet boundary so MP4 can represent AAC priming and
    // final discard padding correctly. Units are PCM samples per channel.
    std::uint32_t skip_samples = 0U;
    std::uint32_t discard_padding_samples = 0U;

    [[nodiscard]] bool Valid() const noexcept {
        if (data.empty() || duration_ns <= 0) {
            return false;
        }
        if (!has_source_samples) {
            return source_frame_count == 0U;
        }
        return source_frame_count != 0U;
    }
};

}  // namespace visionarm
