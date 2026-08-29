#pragma once

#include "audio/audio_types.h"
#include "common/pipeline_types.h"
#include "video/video_types.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace visionarm {

struct FfmpegMpegTsUdpSinkConfig {
    // Must be an explicit udp:// URL. Transport options below are passed to
    // FFmpeg explicitly, so a query-free URL is recommended.
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

    // Pace UDP output instead of emitting each encoded access unit as a burst.
    // 1316 bytes carries exactly seven 188-byte MPEG-TS packets and remains
    // below the normal Ethernet MTU. udp_bit_rate_bps must cover encoded A/V
    // plus MPEG-TS overhead; the runtime derives a 20% headroom
    // value by default.
    std::int32_t udp_packet_size = 1'316;
    std::int32_t udp_send_buffer_bytes = 4 * 1'024 * 1'024;
    std::int64_t udp_bit_rate_bps = 0;
    std::int64_t udp_burst_bits = 0;
    std::int64_t max_interleave_delta_us = 100'000;
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
    std::uint64_t video_parameter_set_injections = 0U;
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

// Asynchronous MPEG-TS/UDP sink for the LAN return path.
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
