#pragma once

#include "audio/encoded_audio_packet_sink.h"
#include "media/ffmpeg_mpegts_udp_muxer.h"
#include "pipeline/bounded_queue.h"
#include "video/encoded_packet_sink.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace visionarm {

struct NetworkMediaSinkConfig {
    FfmpegMpegTsUdpMuxerConfig mux;
    std::size_t video_queue_capacity = 8U;
    std::size_t audio_queue_capacity = 64U;
    std::uint32_t reconnect_interval_ms = 500U;
};

struct NetworkMediaSinkSnapshot {
    bool started = false;
    bool running = false;
    bool connected = false;
    bool clean_stop = false;
    bool fatal_error = false;
    std::string last_error;

    std::uint64_t connect_attempts = 0U;
    std::uint64_t connections_opened = 0U;
    std::uint64_t reconnect_successes = 0U;
    std::uint64_t disconnect_events = 0U;
    std::uint64_t mux_finalize_failures = 0U;

    std::uint64_t video_fragments_received = 0U;
    std::uint64_t video_access_units_queued = 0U;
    std::uint64_t video_access_units_sent = 0U;
    std::uint64_t video_access_units_dropped_queue_full = 0U;
    std::uint64_t video_access_units_dropped_disconnected = 0U;
    std::uint64_t video_access_units_dropped_waiting_keyframe = 0U;
    std::uint64_t video_bytes_queued = 0U;
    std::uint64_t video_bytes_sent = 0U;

    std::uint64_t audio_packets_received = 0U;
    std::uint64_t audio_packets_queued = 0U;
    std::uint64_t audio_packets_sent = 0U;
    std::uint64_t audio_packets_dropped_disconnected = 0U;
    std::uint64_t audio_queue_overload_faults = 0U;
    std::uint64_t audio_bytes_queued = 0U;
    std::uint64_t audio_bytes_sent = 0U;

    QueueStatsSnapshot video_queue;
    QueueStatsSnapshot audio_queue;
};

// Asynchronous, bounded network sink for V8.4.
//
// The producer-facing Write()/WriteAudio() methods never perform UDP I/O.
// Video MPP fragments are first coalesced into one owned access unit and then
// queued. A full video queue drops the whole access unit explicitly. Audio is
// never silently replaced: queue exhaustion is an explicit network fault.
//
// The worker owns FfmpegMpegTsUdpMuxer and recreates it after transport write
// failures. While disconnected it drains and explicitly accounts undeliverable
// packets so network failure cannot backpressure Camera or the frozen local MP4
// recording path indefinitely.
class NetworkMediaSink final :
    public IEncodedPacketSink,
    public IEncodedAudioPacketSink {
public:
    explicit NetworkMediaSink(const NetworkMediaSinkConfig& config);
    ~NetworkMediaSink() override;

    NetworkMediaSink(const NetworkMediaSink&) = delete;
    NetworkMediaSink& operator=(const NetworkMediaSink&) = delete;

    bool Start(std::string* error);
    void Stop() noexcept;

    [[nodiscard]] bool Write(const EncodedPacket& packet) noexcept override;
    void Flush() noexcept override;

    [[nodiscard]] bool WriteAudio(
        const EncodedAudioPacket& packet) noexcept override;
    void FlushAudio() noexcept override;

    [[nodiscard]] NetworkMediaSinkSnapshot Snapshot() const;

private:
    void ThreadMain() noexcept;
    void NotifyWorker() noexcept;
    void SetFatal(const std::string& error) noexcept;

    const NetworkMediaSinkConfig config_;
    BoundedQueue<EncodedPacket> video_queue_;
    BoundedQueue<EncodedAudioPacket> audio_queue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread thread_;

    mutable std::mutex state_mutex_;
    NetworkMediaSinkSnapshot snapshot_;

    std::mutex video_assembly_mutex_;
    EncodedPacket pending_video_;
    bool have_pending_video_ = false;

    std::mutex wake_mutex_;
    std::condition_variable wake_cv_;
};

}  // namespace visionarm
