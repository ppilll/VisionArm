#include "audio/audio_encode_worker.h"

#include <exception>
#include <stdexcept>
#include <utility>
#include <vector>

namespace visionarm {

AudioEncodeWorker::AudioEncodeWorker(
    FfmpegAacEncoderConfig config,
    BoundedQueue<TimedAudioChunk>* input_queue,
    BoundedQueue<EncodedAudioPacket>* output_queue,
    ChunkObserver observer)
    : config_(std::move(config)),
      input_queue_(input_queue),
      output_queue_(output_queue),
      observer_(std::move(observer)) {
    if (input_queue_ == nullptr || output_queue_ == nullptr) {
        throw std::invalid_argument(
            "AudioEncodeWorker requires input and output queues");
    }
}

AudioEncodeWorker::~AudioEncodeWorker() {
    Stop();
}

bool AudioEncodeWorker::Start(std::string* error) {
    if (thread_.joinable()) {
        if (error != nullptr) {
            *error = "AudioEncodeWorker already started";
        }
        return false;
    }

    try {
        encoder_.Initialize(config_);
        stream_info_ = encoder_.stream_info();
        if (!stream_info_.Valid()) {
            throw std::runtime_error("AAC encoder stream info is invalid");
        }
    } catch (const std::exception& ex) {
        if (error != nullptr) {
            *error = ex.what();
        }
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = {};
        snapshot_.started = true;
        snapshot_.encoder = encoder_.snapshot();
    }
    running_.store(true, std::memory_order_release);

    try {
        thread_ = std::thread(&AudioEncodeWorker::ThreadMain, this);
    } catch (const std::exception& ex) {
        running_.store(false, std::memory_order_release);
        encoder_.Shutdown();
        if (error != nullptr) {
            *error = ex.what();
        }
        return false;
    }
    return true;
}

void AudioEncodeWorker::Stop() noexcept {
    if (input_queue_ != nullptr) {
        input_queue_->Stop();
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);
}

AudioEncoderStreamInfo AudioEncodeWorker::stream_info() const {
    return stream_info_;
}

AudioEncodeWorkerSnapshot AudioEncodeWorker::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    AudioEncodeWorkerSnapshot copy = snapshot_;
    copy.running = running_.load(std::memory_order_acquire);
    return copy;
}

void AudioEncodeWorker::PublishFatal(const std::string& error) noexcept {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.fatal_error = true;
        snapshot_.last_error = error;
    }
    input_queue_->Stop();
    output_queue_->Stop();
}

bool AudioEncodeWorker::PublishPackets(
    std::vector<EncodedAudioPacket>* packets) noexcept {
    if (packets == nullptr) {
        return false;
    }
    for (EncodedAudioPacket& packet : *packets) {
        const std::uint64_t bytes = packet.data.size();
        if (!output_queue_->WaitPush(std::move(packet))) {
            std::lock_guard<std::mutex> lock(mutex_);
            ++snapshot_.queue_push_failures;
            snapshot_.fatal_error = true;
            snapshot_.last_error = "encoded-audio queue closed while publishing";
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.packets_pushed;
        snapshot_.bytes_pushed += bytes;
    }
    packets->clear();
    return true;
}

void AudioEncodeWorker::ThreadMain() noexcept {
    try {
        TimedAudioChunk chunk;
        while (input_queue_->WaitPop(&chunk)) {
            if (observer_) {
                observer_(chunk);
            }

            std::vector<EncodedAudioPacket> packets;
            if (!encoder_.Encode(std::move(chunk), &packets)) {
                PublishFatal(encoder_.snapshot().last_error);
                running_.store(false, std::memory_order_release);
                return;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                ++snapshot_.chunks_consumed;
                snapshot_.encoder = encoder_.snapshot();
            }
            if (!PublishPackets(&packets)) {
                input_queue_->Stop();
                output_queue_->Stop();
                running_.store(false, std::memory_order_release);
                return;
            }
        }

        std::vector<EncodedAudioPacket> tail_packets;
        if (!encoder_.Flush(&tail_packets)) {
            PublishFatal(encoder_.snapshot().last_error);
            running_.store(false, std::memory_order_release);
            return;
        }
        if (!PublishPackets(&tail_packets)) {
            input_queue_->Stop();
            output_queue_->Stop();
            running_.store(false, std::memory_order_release);
            return;
        }
    } catch (const std::exception& ex) {
        PublishFatal(ex.what());
        running_.store(false, std::memory_order_release);
        return;
    } catch (...) {
        PublishFatal("unknown AudioEncodeWorker exception");
        running_.store(false, std::memory_order_release);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.encoder = encoder_.snapshot();
    }
    output_queue_->Stop();
    running_.store(false, std::memory_order_release);
}

}  // namespace visionarm
