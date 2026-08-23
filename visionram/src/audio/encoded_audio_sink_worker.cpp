#include "audio/encoded_audio_sink_worker.h"

#include <exception>
#include <stdexcept>

namespace visionarm {

EncodedAudioSinkWorker::EncodedAudioSinkWorker(
    BoundedQueue<EncodedAudioPacket>* input_queue,
    IEncodedAudioPacketSink* sink)
    : input_queue_(input_queue), sink_(sink) {
    if (input_queue_ == nullptr || sink_ == nullptr) {
        throw std::invalid_argument(
            "EncodedAudioSinkWorker requires queue and sink");
    }
}

EncodedAudioSinkWorker::~EncodedAudioSinkWorker() {
    Stop();
}

bool EncodedAudioSinkWorker::Start(std::string* error) {
    if (thread_.joinable()) {
        if (error != nullptr) {
            *error = "EncodedAudioSinkWorker already started";
        }
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = {};
        snapshot_.started = true;
    }
    running_.store(true, std::memory_order_release);
    try {
        thread_ = std::thread(&EncodedAudioSinkWorker::ThreadMain, this);
    } catch (const std::exception& ex) {
        running_.store(false, std::memory_order_release);
        if (error != nullptr) {
            *error = ex.what();
        }
        return false;
    }
    return true;
}

void EncodedAudioSinkWorker::Stop() noexcept {
    if (input_queue_ != nullptr) {
        input_queue_->Stop();
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);
}

EncodedAudioSinkWorkerSnapshot EncodedAudioSinkWorker::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    EncodedAudioSinkWorkerSnapshot copy = snapshot_;
    copy.running = running_.load(std::memory_order_acquire);
    return copy;
}

void EncodedAudioSinkWorker::PublishFatal(const std::string& error) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.fatal_error = true;
    snapshot_.last_error = error;
    ++snapshot_.sink_failures;
}

void EncodedAudioSinkWorker::ThreadMain() noexcept {
    try {
        EncodedAudioPacket packet;
        while (input_queue_->WaitPop(&packet)) {
            const std::uint64_t bytes = packet.data.size();
            if (!sink_->WriteAudio(packet)) {
                PublishFatal("encoded audio sink WriteAudio failed");
                input_queue_->Stop();
                running_.store(false, std::memory_order_release);
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            ++snapshot_.packets_written;
            snapshot_.bytes_written += bytes;
        }
        sink_->FlushAudio();
    } catch (const std::exception& ex) {
        PublishFatal(ex.what());
        input_queue_->Stop();
        running_.store(false, std::memory_order_release);
        return;
    } catch (...) {
        PublishFatal("unknown EncodedAudioSinkWorker exception");
        input_queue_->Stop();
        running_.store(false, std::memory_order_release);
        return;
    }
    running_.store(false, std::memory_order_release);
}

}  // namespace visionarm
