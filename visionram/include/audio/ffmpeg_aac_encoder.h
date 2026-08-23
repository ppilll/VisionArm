#pragma once

#include "audio/encoded_audio_packet.h"
#include "media/media_clock.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace visionarm {

struct FfmpegAacEncoderConfig {
    AudioStreamFormat input_format;
    std::int32_t bit_rate_bps = 128'000;
};

struct AudioEncoderSnapshot {
    std::uint64_t input_chunks = 0U;
    std::uint64_t input_frames = 0U;
    std::uint64_t input_bytes = 0U;
    std::uint64_t input_discontinuities = 0U;

    std::uint64_t submitted_codec_frames = 0U;
    std::uint64_t emitted_packets = 0U;
    std::uint64_t emitted_bytes = 0U;
    std::uint64_t encoder_padding_packets = 0U;
    std::uint64_t encode_failures = 0U;

    std::uint32_t buffered_input_frames = 0U;
    bool drained = false;
    std::string last_error;
};

// FFmpeg 4.4 native AAC-LC encoder for the V8.3 TimedAudioChunk contract.
//
// The public header intentionally exposes no libavcodec types. This keeps the
// rest of the product graph independent from FFmpeg ABI details and gives the
// later mux layer an application-owned EncodedAudioPacket contract.
class FfmpegAacEncoder final {
public:
    FfmpegAacEncoder();
    ~FfmpegAacEncoder();

    FfmpegAacEncoder(const FfmpegAacEncoder&) = delete;
    FfmpegAacEncoder& operator=(const FfmpegAacEncoder&) = delete;

    void Initialize(const FfmpegAacEncoderConfig& config);
    void Shutdown() noexcept;

    [[nodiscard]] bool initialized() const noexcept;

    // Consumes one TimedAudioChunk. Any complete AAC access units produced by
    // libavcodec are appended to packets. The encoder owns all temporarily
    // buffered PCM after this call returns.
    [[nodiscard]] bool Encode(
        TimedAudioChunk chunk,
        std::vector<EncodedAudioPacket>* packets) noexcept;

    // Encodes any final partial frame supported by the codec, drains delayed
    // AAC packets, and enters a terminal drained state.
    [[nodiscard]] bool Flush(
        std::vector<EncodedAudioPacket>* packets) noexcept;

    [[nodiscard]] AudioEncoderStreamInfo stream_info() const;
    [[nodiscard]] AudioEncoderSnapshot snapshot() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace visionarm
