#pragma once

#include "audio/audio_types.h"
#include "video/video_types.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace visionarm {

struct FfmpegMp4MuxerConfig {
    std::string path;

    std::int32_t video_width = 0;
    std::int32_t video_height = 0;
    std::int32_t video_bit_rate_bps = 0;
    std::int32_t video_fps_numerator = 0;
    std::int32_t video_fps_denominator = 1;

    // Annex-B VPS/SPS/PPS returned by MPP_ENC_GET_HDR_SYNC. The FFmpeg MOV/MP4
    // muxer derives HEVC configuration from this extradata and converts
    // Annex-B access units to MP4 length-prefixed samples.
    std::vector<std::uint8_t> hevc_annexb_codec_config;

    AudioEncoderStreamInfo audio;
};

struct FfmpegMp4MuxerSnapshot {
    bool opened = false;
    bool header_written = false;
    bool finalized = false;
    bool fatal_error = false;
    std::string last_error;

    std::uint64_t video_fragments_received = 0U;
    std::uint64_t video_samples_written = 0U;
    std::uint64_t video_bytes_written = 0U;
    std::uint64_t audio_packets_written = 0U;
    std::uint64_t audio_bytes_written = 0U;
    std::uint64_t write_failures = 0U;

    std::int64_t first_video_pts_us = -1;
    std::int64_t last_video_pts_us = -1;
    std::int64_t first_audio_pts_ns = -1;
    std::int64_t last_audio_pts_ns = -1;
};

class FfmpegMp4Muxer final :
    public IEncodedPacketSink,
    public IEncodedAudioPacketSink {
public:
    FfmpegMp4Muxer();
    ~FfmpegMp4Muxer() override;

    FfmpegMp4Muxer(const FfmpegMp4Muxer&) = delete;
    FfmpegMp4Muxer& operator=(const FfmpegMp4Muxer&) = delete;

    void Initialize(const FfmpegMp4MuxerConfig& config);

    [[nodiscard]] bool Write(const EncodedPacket& packet) noexcept override;
    void Flush() noexcept override;

    [[nodiscard]] bool WriteAudio(
        const EncodedAudioPacket& packet) noexcept override;
    void FlushAudio() noexcept override;

    // Writes the MP4 trailer/moov and closes the file. Must be called only
    // after both encoded-video and encoded-audio producer chains are drained.
    [[nodiscard]] bool Finalize() noexcept;

    [[nodiscard]] FfmpegMp4MuxerSnapshot Snapshot() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace visionarm
