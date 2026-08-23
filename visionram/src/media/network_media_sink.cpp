#include "media/network_media_sink.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace visionarm {
namespace {

using SteadyClock = std::chrono::steady_clock;

[[nodiscard]] std::int64_t VideoDtsNs(const EncodedPacket& packet) noexcept {
    if (packet.dts_us > std::numeric_limits<std::int64_t>::max() / 1000LL) {
        return std::numeric_limits<std::int64_t>::max();
    }
    if (packet.dts_us < std::numeric_limits<std::int64_t>::min() / 1000LL) {
        return std::numeric_limits<std::int64_t>::min();
    }
    return packet.dts_us * 1000LL;
}

}  // namespace

NetworkMediaSink::NetworkMediaSink(const NetworkMediaSinkConfig& config)
    : config_(config),
      video_queue_(config.video_queue_capacity),
      audio_queue_(config.audio_queue_capacity) {
    if (config_.mux.url.rfind("udp://", 0) != 0 ||
        config_.video_queue_capacity == 0U ||
        config_.audio_queue_capacity == 0U ||
        config_.reconnect_interval_ms == 0U) {
        throw std::invalid_argument("invalid NetworkMediaSink config");
    }
}

NetworkMediaSink::~NetworkMediaSink() {
    Stop();
}

bool NetworkMediaSink::Start(std::string* error) {
    if (thread_.joinable()) {
        if (error != nullptr) {
            *error = "NetworkMediaSink already started";
        }
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot_ = {};
        snapshot_.started = true;
    }
    stop_requested_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    try {
        thread_ = std::thread(&NetworkMediaSink::ThreadMain, this);
    } catch (const std::exception& ex) {
        running_.store(false, std::memory_order_release);
        SetFatal(ex.what());
        if (error != nullptr) {
            *error = ex.what();
        }
        return false;
    }
    return true;
}

void NetworkMediaSink::Stop() noexcept {
    stop_requested_.store(true, std::memory_order_release);
    video_queue_.Stop();
    audio_queue_.Stop();
    NotifyWorker();
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);
}

bool NetworkMediaSink::Write(const EncodedPacket& packet) noexcept {
    if (!running_.load(std::memory_order_acquire)) {
        return false;
    }
    if (packet.codec_config) {
        return true;
    }
    if (packet.bytes.empty() || packet.duration_us <= 0) {
        SetFatal("invalid HEVC packet passed to NetworkMediaSink");
        return false;
    }

    EncodedPacket access_unit;
    bool access_unit_ready = false;
    {
        std::lock_guard<std::mutex> assembly_lock(video_assembly_mutex_);
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            ++snapshot_.video_fragments_received;
        }

        if (!have_pending_video_) {
            pending_video_ = packet;
            pending_video_.bytes.clear();
            pending_video_.end_of_frame = true;
            pending_video_.codec_config = false;
            have_pending_video_ = true;
        } else if (packet.pts_us != pending_video_.pts_us ||
                   packet.dts_us != pending_video_.dts_us ||
                   packet.duration_us != pending_video_.duration_us) {
            SetFatal("MPP fragment timestamp changed before end_of_frame in NetworkMediaSink");
            return false;
        }

        pending_video_.keyframe = pending_video_.keyframe || packet.keyframe;
        pending_video_.end_of_stream =
            pending_video_.end_of_stream || packet.end_of_stream;
        pending_video_.bytes.insert(
            pending_video_.bytes.end(), packet.bytes.begin(), packet.bytes.end());

        if (packet.end_of_frame) {
            access_unit = std::move(pending_video_);
            pending_video_ = {};
            have_pending_video_ = false;
            access_unit_ready = true;
        }
    }

    if (!access_unit_ready) {
        return true;
    }

    const std::uint64_t bytes = access_unit.bytes.size();
    if (!video_queue_.TryPush(std::move(access_unit))) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++snapshot_.video_access_units_dropped_queue_full;
        return true;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++snapshot_.video_access_units_queued;
        snapshot_.video_bytes_queued += bytes;
    }
    NotifyWorker();
    return true;
}

void NetworkMediaSink::Flush() noexcept {
    NotifyWorker();
}

bool NetworkMediaSink::WriteAudio(
    const EncodedAudioPacket& packet) noexcept {
    if (!running_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!packet.Valid()) {
        SetFatal("invalid AAC packet passed to NetworkMediaSink");
        return false;
    }

    const std::uint64_t bytes = packet.data.size();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++snapshot_.audio_packets_received;
    }
    if (!audio_queue_.TryPush(packet)) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ++snapshot_.audio_queue_overload_faults;
            snapshot_.fatal_error = true;
            snapshot_.last_error =
                "network audio queue exhausted; audio was not silently replaced";
        }
        NotifyWorker();
        // Return true so a simultaneous local recorder can still consume this
        // packet. The main integration loop observes fatal_error and terminates
        // the run with explicit diagnostics rather than corrupting local media.
        return true;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++snapshot_.audio_packets_queued;
        snapshot_.audio_bytes_queued += bytes;
    }
    NotifyWorker();
    return true;
}

void NetworkMediaSink::FlushAudio() noexcept {
    NotifyWorker();
}

NetworkMediaSinkSnapshot NetworkMediaSink::Snapshot() const {
    NetworkMediaSinkSnapshot copy;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        copy = snapshot_;
        copy.running = running_.load(std::memory_order_acquire);
    }
    copy.video_queue = video_queue_.Snapshot();
    copy.audio_queue = audio_queue_.Snapshot();
    return copy;
}

void NetworkMediaSink::NotifyWorker() noexcept {
    wake_cv_.notify_one();
}

void NetworkMediaSink::SetFatal(const std::string& error) noexcept {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snapshot_.fatal_error = true;
    snapshot_.last_error = error;
}

void NetworkMediaSink::ThreadMain() noexcept {
    std::unique_ptr<FfmpegMpegTsUdpMuxer> muxer;
    bool connected = false;
    bool ever_connected = false;
    bool waiting_for_keyframe = true;
    auto next_connect_attempt = SteadyClock::now();

    std::optional<EncodedPacket> pending_video;
    std::optional<EncodedAudioPacket> pending_audio;

    auto set_connected = [&](bool value) {
        connected = value;
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot_.connected = value;
    };

    auto close_muxer = [&](bool count_finalize_failure) {
        if (muxer != nullptr) {
            const bool finalize_ok = muxer->Finalize();
            if (!finalize_ok && count_finalize_failure) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ++snapshot_.mux_finalize_failures;
            }
            muxer.reset();
        }
        set_connected(false);
    };

    auto register_disconnect = [&](const std::string& error) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ++snapshot_.disconnect_events;
            snapshot_.last_error = error;
        }
        close_muxer(false);
        waiting_for_keyframe = true;
        next_connect_attempt = SteadyClock::now() +
            std::chrono::milliseconds(config_.reconnect_interval_ms);
    };

    try {
        while (true) {
            const bool stopping =
                stop_requested_.load(std::memory_order_acquire);
            const bool fatal = [&]() {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return snapshot_.fatal_error;
            }();
            if (fatal) {
                break;
            }

            if (!connected && !stopping && SteadyClock::now() >= next_connect_attempt) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    ++snapshot_.connect_attempts;
                }
                try {
                    auto candidate = std::make_unique<FfmpegMpegTsUdpMuxer>();
                    candidate->Initialize(config_.mux);
                    muxer = std::move(candidate);
                    set_connected(true);
                    waiting_for_keyframe = true;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        ++snapshot_.connections_opened;
                        if (ever_connected) {
                            ++snapshot_.reconnect_successes;
                        }
                        snapshot_.last_error.clear();
                    }
                    ever_connected = true;
                } catch (const std::exception& ex) {
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        snapshot_.last_error = ex.what();
                    }
                    next_connect_attempt = SteadyClock::now() +
                        std::chrono::milliseconds(config_.reconnect_interval_ms);
                }
            }

            if (!pending_video.has_value()) {
                EncodedPacket packet;
                if (video_queue_.TryPop(&packet)) {
                    pending_video = std::move(packet);
                }
            }
            if (!pending_audio.has_value()) {
                EncodedAudioPacket packet;
                if (audio_queue_.TryPop(&packet)) {
                    pending_audio = std::move(packet);
                }
            }

            if (stopping && !pending_video.has_value() &&
                !pending_audio.has_value() && video_queue_.Size() == 0U &&
                audio_queue_.Size() == 0U) {
                break;
            }

            if (!connected) {
                // A disconnected UDP destination must never create unbounded
                // producer backpressure. Drop only inside this network branch
                // and account every discontinuity explicitly.
                if (pending_video.has_value()) {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    ++snapshot_.video_access_units_dropped_disconnected;
                    pending_video.reset();
                }
                if (pending_audio.has_value()) {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    ++snapshot_.audio_packets_dropped_disconnected;
                    pending_audio.reset();
                }
            } else {
                bool choose_video = false;
                if (pending_video.has_value() && pending_audio.has_value()) {
                    choose_video =
                        VideoDtsNs(*pending_video) <= pending_audio->dts_ns;
                } else {
                    choose_video = pending_video.has_value();
                }

                if (choose_video && pending_video.has_value()) {
                    if (waiting_for_keyframe && !pending_video->keyframe) {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        ++snapshot_.video_access_units_dropped_waiting_keyframe;
                        pending_video.reset();
                    } else {
                        const std::uint64_t bytes = pending_video->bytes.size();
                        if (!muxer->Write(*pending_video)) {
                            const auto mux_snapshot = muxer->Snapshot();
                            {
                                std::lock_guard<std::mutex> lock(state_mutex_);
                                ++snapshot_.video_access_units_dropped_disconnected;
                            }
                            pending_video.reset();
                            register_disconnect(
                                mux_snapshot.last_error.empty() ?
                                    "network video mux write failed" :
                                    mux_snapshot.last_error);
                        } else {
                            waiting_for_keyframe = false;
                            {
                                std::lock_guard<std::mutex> lock(state_mutex_);
                                ++snapshot_.video_access_units_sent;
                                snapshot_.video_bytes_sent += bytes;
                            }
                            pending_video.reset();
                        }
                    }
                } else if (pending_audio.has_value()) {
                    const std::uint64_t bytes = pending_audio->data.size();
                    if (!muxer->WriteAudio(*pending_audio)) {
                        const auto mux_snapshot = muxer->Snapshot();
                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            ++snapshot_.audio_packets_dropped_disconnected;
                        }
                        pending_audio.reset();
                        register_disconnect(
                            mux_snapshot.last_error.empty() ?
                                "network audio mux write failed" :
                                mux_snapshot.last_error);
                    } else {
                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            ++snapshot_.audio_packets_sent;
                            snapshot_.audio_bytes_sent += bytes;
                        }
                        pending_audio.reset();
                    }
                }
            }

            if (!pending_video.has_value() && !pending_audio.has_value() &&
                video_queue_.Size() == 0U && audio_queue_.Size() == 0U) {
                std::unique_lock<std::mutex> lock(wake_mutex_);
                auto wait_duration = std::chrono::milliseconds(50);
                if (!connected && !stopping) {
                    const auto now = SteadyClock::now();
                    if (next_connect_attempt > now) {
                        wait_duration = std::min(
                            wait_duration,
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                next_connect_attempt - now));
                    } else {
                        wait_duration = std::chrono::milliseconds(1);
                    }
                }
                wake_cv_.wait_for(lock, wait_duration);
            }
        }

        if (muxer != nullptr) {
            close_muxer(true);
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            snapshot_.clean_stop = !snapshot_.fatal_error;
            snapshot_.connected = false;
        }
    } catch (const std::exception& ex) {
        SetFatal(ex.what());
        if (muxer != nullptr) {
            close_muxer(false);
        }
    } catch (...) {
        SetFatal("unknown NetworkMediaSink worker exception");
        if (muxer != nullptr) {
            close_muxer(false);
        }
    }

    running_.store(false, std::memory_order_release);
}

}  // namespace visionarm
