#pragma once

#include "audio/encoded_audio_packet.h"
#include "audio/encoded_audio_packet_sink.h"
#include "video/encoded_packet_sink.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace visionarm {

struct FfmpegMpegTsUdpMuxerConfig {
    // Example:
    // udp://192.168.1.50:5600?pkt_size=1316&buffer_size=1048576&connect=1
    std::string url;

    std::int32_t video_width = 0;
    std::int32_t video_height = 0;
    std::int32_t video_bit_rate_bps = 0;
    std::int32_t video_fps_numerator = 0;
    std::int32_t video_fps_denominator = 1;

    // Annex-B VPS/SPS/PPS from MPP_ENC_GET_HDR_SYNC. MPEG-TS keeps HEVC in
    // Annex-B form and FFmpeg can prepend the extradata at random-access
    // pictures when required.
    std::vector<std::uint8_t> hevc_annexb_codec_config;

    // AAC stream metadata from the frozen V8.3 encoder. codec_config remains
    // available for validation/debug, but the MPEG-TS stream intentionally does
    // not publish ASC extradata: V8.4 explicitly prepends a 7-byte ADTS header
    // to every raw AAC-LC access unit, so AAC configuration is carried in-band.
    AudioEncoderStreamInfo audio;
};

struct FfmpegMpegTsUdpMuxerSnapshot {
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

class FfmpegMpegTsUdpMuxer final :
    public IEncodedPacketSink,
    public IEncodedAudioPacketSink {
public:
    FfmpegMpegTsUdpMuxer();
    ~FfmpegMpegTsUdpMuxer() override;

    FfmpegMpegTsUdpMuxer(const FfmpegMpegTsUdpMuxer&) = delete;
    FfmpegMpegTsUdpMuxer& operator=(const FfmpegMpegTsUdpMuxer&) = delete;

    void Initialize(const FfmpegMpegTsUdpMuxerConfig& config);

    [[nodiscard]] bool Write(const EncodedPacket& packet) noexcept override;
    void Flush() noexcept override;

    [[nodiscard]] bool WriteAudio(
        const EncodedAudioPacket& packet) noexcept override;
    void FlushAudio() noexcept override;

    // Drains FFmpeg's mux state, flushes AVIO and closes the UDP socket.
    [[nodiscard]] bool Finalize() noexcept;

    [[nodiscard]] FfmpegMpegTsUdpMuxerSnapshot Snapshot() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace visionarm
