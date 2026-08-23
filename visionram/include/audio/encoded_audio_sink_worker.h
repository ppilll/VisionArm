#pragma once

#include "audio/encoded_audio_packet.h"
#include "audio/encoded_audio_packet_sink.h"
#include "pipeline/bounded_queue.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace visionarm {

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
    EncodedAudioSinkWorker(
        BoundedQueue<EncodedAudioPacket>* input_queue,
        IEncodedAudioPacketSink* sink);
    ~EncodedAudioSinkWorker();

    EncodedAudioSinkWorker(const EncodedAudioSinkWorker&) = delete;
    EncodedAudioSinkWorker& operator=(const EncodedAudioSinkWorker&) = delete;

    bool Start(std::string* error);
    void Stop() noexcept;

    [[nodiscard]] bool running() const noexcept {
        return running_.load(std::memory_order_acquire);
    }
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
