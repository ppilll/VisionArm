#pragma once

#include "audio/encoded_audio_packet.h"
#include "audio/encoded_audio_packet_sink.h"
#include "common/pipeline_types.h"
#include "video/encoded_packet_sink.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace visionarm {

struct FfmpegMpegTsUdpSinkConfig {
    // Must be an explicit udp:// URL. Query parameters are passed through to
    // FFmpeg's UDP protocol; pkt_size=1316 is recommended for MPEG-TS.
    std::string url;

    std::int32_t video_width = 0;
    std::int32_t video_height = 0;
    std::int32_t video_bit_rate_bps = 0;
    std::int32_t video_fps_numerator = 0;
    std::int32_t video_fps_denominator = 1;
    std::vector<std::uint8_t> hevc_annexb_codec_config;

    AudioEncoderStreamInfo audio;

    // One queue contains complete HEVC access units and AAC access units. It
    // owns all bytes and therefore never extends a Camera FrameLease lifetime.
    std::size_t packet_queue_capacity = 256U;
    std::int64_t io_timeout_us = 1'000'000;
};

struct FfmpegMpegTsUdpSinkSnapshot {
    bool started = false;
    bool running = false;
    bool opened = false;
    bool finalized = false;
    bool fatal_error = false;
    std::string last_error;

    std::uint64_t video_fragments_received = 0U;
    std::uint64_t video_access_units_enqueued = 0U;
    std::uint64_t video_access_units_written = 0U;
    std::uint64_t video_bytes_written = 0U;
    std::uint64_t audio_packets_enqueued = 0U;
    std::uint64_t audio_packets_written = 0U;
    std::uint64_t audio_bytes_written = 0U;

    std::uint64_t queue_overload_failures = 0U;
    std::uint64_t write_failures = 0U;
    QueueStatsSnapshot packet_queue;

    std::int64_t first_video_pts_us = -1;
    std::int64_t last_video_pts_us = -1;
    std::int64_t first_audio_pts_ns = -1;
    std::int64_t last_audio_pts_ns = -1;
};

// Asynchronous MPEG-TS/UDP sink for the V8.4 LAN return path.
//
// Write()/WriteAudio() only copy application-owned encoded bytes into a
// bounded queue. A single worker owns libavformat and all network I/O. Queue
// overload is an explicit fatal error; video or audio is never silently
// discarded. Stop() drains accepted packets, writes the MPEG-TS trailer, and
// closes the UDP AVIO context.
class FfmpegMpegTsUdpSink final :
    public IEncodedPacketSink,
    public IEncodedAudioPacketSink {
public:
    FfmpegMpegTsUdpSink();
    ~FfmpegMpegTsUdpSink() override;

    FfmpegMpegTsUdpSink(const FfmpegMpegTsUdpSink&) = delete;
    FfmpegMpegTsUdpSink& operator=(const FfmpegMpegTsUdpSink&) = delete;

    void Initialize(const FfmpegMpegTsUdpSinkConfig& config);

    [[nodiscard]] bool Write(const EncodedPacket& packet) noexcept override;
    void Flush() noexcept override;

    [[nodiscard]] bool WriteAudio(
        const EncodedAudioPacket& packet) noexcept override;
    void FlushAudio() noexcept override;

    [[nodiscard]] bool Stop() noexcept;
    [[nodiscard]] FfmpegMpegTsUdpSinkSnapshot Snapshot() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace visionarm
