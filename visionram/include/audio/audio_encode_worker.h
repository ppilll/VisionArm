#pragma once

#include "audio/encoded_audio_packet.h"
#include "audio/ffmpeg_aac_encoder.h"
#include "media/media_clock.h"
#include "pipeline/bounded_queue.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace visionarm {

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

    AudioEncodeWorker(
        FfmpegAacEncoderConfig config,
        BoundedQueue<TimedAudioChunk>* input_queue,
        BoundedQueue<EncodedAudioPacket>* output_queue,
        ChunkObserver observer = {});
    ~AudioEncodeWorker();

    AudioEncodeWorker(const AudioEncodeWorker&) = delete;
    AudioEncodeWorker& operator=(const AudioEncodeWorker&) = delete;

    bool Start(std::string* error);

    // Graceful producer-chain stop. BoundedQueue::Stop() keeps queued PCM
    // drainable; the worker then Flush()es libavcodec and closes the encoded
    // packet queue after all delayed AAC packets have been published.
    void Stop() noexcept;

    [[nodiscard]] bool running() const noexcept {
        return running_.load(std::memory_order_acquire);
    }

    [[nodiscard]] AudioEncoderStreamInfo stream_info() const;
    [[nodiscard]] AudioEncodeWorkerSnapshot Snapshot() const;

private:
    void ThreadMain() noexcept;
    void PublishFatal(const std::string& error) noexcept;
    [[nodiscard]] bool PublishPackets(
        std::vector<EncodedAudioPacket>* packets) noexcept;

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

}  // namespace visionarm
