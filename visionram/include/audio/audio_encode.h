#pragma once

#include "audio/audio_types.h"
#include "media/media_clock.h"
#include "pipeline/bounded_queue.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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

struct AudioEncodeWorkerSnapshot {
    bool started = false;
    bool running = false;
    bool fatal_error = false;
    std::string last_error;
    std::uint64_t chunks_consumed = 0U;
    std::uint64_t packets_pushed = 0U;
    std::uint64_t bytes_pushed = 0U;
    std::uint64_t queue_push_failures = 0U;
    AudioEncoderSnapshot encoder;
};

class AudioEncodeWorker final {
public:
    using ChunkObserver = std::function<void(const TimedAudioChunk&)>;
    AudioEncodeWorker(FfmpegAacEncoderConfig config,
                      BoundedQueue<TimedAudioChunk>* input_queue,
                      BoundedQueue<EncodedAudioPacket>* output_queue,
                      ChunkObserver observer = {});
    ~AudioEncodeWorker();
    AudioEncodeWorker(const AudioEncodeWorker&) = delete;
    AudioEncodeWorker& operator=(const AudioEncodeWorker&) = delete;
    bool Start(std::string* error);
    void Stop() noexcept;
    [[nodiscard]] bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    [[nodiscard]] AudioEncoderStreamInfo stream_info() const;
    [[nodiscard]] AudioEncodeWorkerSnapshot Snapshot() const;

private:
    void ThreadMain() noexcept;
    void PublishFatal(const std::string& error) noexcept;
    [[nodiscard]] bool PublishPackets(std::vector<EncodedAudioPacket>* packets) noexcept;
    const FfmpegAacEncoderConfig config_;
    BoundedQueue<TimedAudioChunk>* input_queue_ = nullptr;
    BoundedQueue<EncodedAudioPacket>* output_queue_ = nullptr;
    ChunkObserver observer_;
    FfmpegAacEncoder encoder_;
    AudioEncoderStreamInfo stream_info_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    AudioEncodeWorkerSnapshot snapshot_;
};

struct EncodedAudioSinkWorkerSnapshot {
    bool started = false;
    bool running = false;
    bool fatal_error = false;
    std::string last_error;
    std::uint64_t packets_written = 0U;
    std::uint64_t bytes_written = 0U;
    std::uint64_t sink_failures = 0U;
};

class EncodedAudioSinkWorker final {
public:
    EncodedAudioSinkWorker(BoundedQueue<EncodedAudioPacket>* input_queue, IEncodedAudioPacketSink* sink);
    ~EncodedAudioSinkWorker();
    EncodedAudioSinkWorker(const EncodedAudioSinkWorker&) = delete;
    EncodedAudioSinkWorker& operator=(const EncodedAudioSinkWorker&) = delete;
    bool Start(std::string* error);
    void Stop() noexcept;
    [[nodiscard]] bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    [[nodiscard]] EncodedAudioSinkWorkerSnapshot Snapshot() const;

private:
    void ThreadMain() noexcept;
    void PublishFatal(const std::string& error) noexcept;
    BoundedQueue<EncodedAudioPacket>* input_queue_ = nullptr;
    IEncodedAudioPacketSink* sink_ = nullptr;
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    EncodedAudioSinkWorkerSnapshot snapshot_;
};

}  // namespace visionarm
