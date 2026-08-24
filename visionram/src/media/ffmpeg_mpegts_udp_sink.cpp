#include "media/ffmpeg_mpegts_udp_sink.h"

#include "pipeline/bounded_queue.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/dict.h>
#include <libavutil/error.h>
#include <libavutil/mem.h>
#include <libavutil/pixfmt.h>
#include <libavutil/version.h>
}

#include <atomic>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace visionarm {
namespace {

constexpr AVRational kVideoTimeBase{1, 1'000'000};
constexpr AVRational kAudioTimeBase{1, 1'000'000'000};

[[nodiscard]] std::string AvErrorString(int error) {
    char text[AV_ERROR_MAX_STRING_SIZE]{};
    if (av_strerror(error, text, sizeof(text)) == 0) {
        return text;
    }
    std::ostringstream stream;
    stream << "FFmpeg error " << error;
    return stream.str();
}

[[nodiscard]] bool CopyExtradata(
    AVCodecParameters* parameters,
    const std::vector<std::uint8_t>& bytes) {
    if (parameters == nullptr || bytes.empty() ||
        bytes.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return false;
    }
    const std::size_t padded_size =
        bytes.size() + static_cast<std::size_t>(AV_INPUT_BUFFER_PADDING_SIZE);
    auto* data = static_cast<std::uint8_t*>(av_mallocz(padded_size));
    if (data == nullptr) {
        return false;
    }
    std::memcpy(data, bytes.data(), bytes.size());
    av_freep(&parameters->extradata);
    parameters->extradata = data;
    parameters->extradata_size = static_cast<int>(bytes.size());
    return true;
}

enum class NetworkPacketKind : std::uint8_t {
    kVideo,
    kAudio,
};

struct NetworkPacket {
    NetworkPacketKind kind = NetworkPacketKind::kVideo;
    std::vector<std::uint8_t> data;
    std::int64_t pts = 0;
    std::int64_t dts = 0;
    std::int64_t duration = 0;
    bool keyframe = false;
};

}  // namespace

struct FfmpegMpegTsUdpSink::Impl {
    FfmpegMpegTsUdpSinkConfig config;
    mutable std::mutex mutex;
    FfmpegMpegTsUdpSinkSnapshot snapshot;
    std::atomic<bool> accepting{false};
    std::atomic<bool> running{false};

    std::unique_ptr<BoundedQueue<NetworkPacket>> queue;
    std::thread worker;

    AVFormatContext* format = nullptr;
    AVStream* video_stream = nullptr;
    AVStream* audio_stream = nullptr;

    // MPP may emit one access unit as multiple partition packets. Assembly is
    // producer-side so one queue item always equals one complete video frame.
    std::mutex video_mutex;
    NetworkPacket pending_video;
    bool have_pending_video = false;

    std::int64_t last_video_dts_us = -1;
    std::int64_t last_audio_dts_ns = -1;

    void SetError(const std::string& error, bool overload = false) noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex);
            snapshot.fatal_error = true;
            snapshot.last_error = error;
            if (overload) {
                ++snapshot.queue_overload_failures;
            } else {
                ++snapshot.write_failures;
            }
        }
        accepting.store(false, std::memory_order_release);
        if (queue != nullptr) {
            queue->Stop();
        }
    }

    void CloseFormat() noexcept {
        if (format != nullptr) {
            if (format->pb != nullptr && format->oformat != nullptr &&
                (format->oformat->flags & AVFMT_NOFILE) == 0) {
                avio_closep(&format->pb);
            }
            avformat_free_context(format);
        }
        format = nullptr;
        video_stream = nullptr;
        audio_stream = nullptr;
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.opened = false;
    }

    [[nodiscard]] bool WritePacket(const NetworkPacket& input) noexcept {
        AVStream* stream = input.kind == NetworkPacketKind::kVideo
            ? video_stream : audio_stream;
        const AVRational source_time_base =
            input.kind == NetworkPacketKind::kVideo
                ? kVideoTimeBase : kAudioTimeBase;
        if (format == nullptr || stream == nullptr || input.data.empty() ||
            input.duration <= 0 ||
            input.data.size() >
                static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            SetError("invalid packet in MPEG-TS/UDP worker");
            return false;
        }

        if (input.kind == NetworkPacketKind::kVideo) {
            if (last_video_dts_us >= 0 && input.dts < last_video_dts_us) {
                SetError("video DTS regression before MPEG-TS mux");
                return false;
            }
        } else if (last_audio_dts_ns >= 0 && input.dts < last_audio_dts_ns) {
            SetError("audio DTS regression before MPEG-TS mux");
            return false;
        }

        AVPacket* packet = av_packet_alloc();
        if (packet == nullptr) {
            SetError("av_packet_alloc failed for MPEG-TS/UDP");
            return false;
        }
        const int allocation =
            av_new_packet(packet, static_cast<int>(input.data.size()));
        if (allocation < 0) {
            av_packet_free(&packet);
            SetError("av_new_packet failed: " + AvErrorString(allocation));
            return false;
        }
        std::memcpy(packet->data, input.data.data(), input.data.size());
        packet->stream_index = stream->index;
        packet->pts = input.pts;
        packet->dts = input.dts;
        packet->duration = input.duration;
        if (input.keyframe) {
            packet->flags |= AV_PKT_FLAG_KEY;
        }
        av_packet_rescale_ts(packet, source_time_base, stream->time_base);

        const int result = av_interleaved_write_frame(format, packet);
        av_packet_free(&packet);
        if (result < 0) {
            SetError(
                "MPEG-TS/UDP av_interleaved_write_frame failed: " +
                AvErrorString(result));
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex);
        if (input.kind == NetworkPacketKind::kVideo) {
            if (snapshot.video_access_units_written == 0U) {
                snapshot.first_video_pts_us = input.pts;
            }
            snapshot.last_video_pts_us = input.pts;
            last_video_dts_us = input.dts;
            ++snapshot.video_access_units_written;
            snapshot.video_bytes_written += input.data.size();
        } else {
            if (snapshot.audio_packets_written == 0U) {
                snapshot.first_audio_pts_ns = input.pts;
            }
            snapshot.last_audio_pts_ns = input.pts;
            last_audio_dts_ns = input.dts;
            ++snapshot.audio_packets_written;
            snapshot.audio_bytes_written += input.data.size();
        }
        return true;
    }

    void WorkerMain() noexcept {
        NetworkPacket packet;
        while (queue->WaitPop(&packet)) {
            if (!WritePacket(packet)) {
                break;
            }
        }

        bool can_finalize = false;
        {
            std::lock_guard<std::mutex> video_lock(video_mutex);
            if (have_pending_video) {
                SetError("network stop reached with incomplete HEVC access unit");
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            can_finalize = !snapshot.fatal_error && format != nullptr;
        }
        if (can_finalize) {
            const int result = av_write_trailer(format);
            if (result < 0) {
                SetError(
                    "MPEG-TS/UDP av_write_trailer failed: " +
                    AvErrorString(result));
            } else {
                std::lock_guard<std::mutex> lock(mutex);
                snapshot.finalized = true;
            }
        }
        if (format != nullptr && format->pb != nullptr) {
            avio_flush(format->pb);
        }
        CloseFormat();
        running.store(false, std::memory_order_release);
    }
};

FfmpegMpegTsUdpSink::FfmpegMpegTsUdpSink()
    : impl_(std::make_unique<Impl>()) {}

FfmpegMpegTsUdpSink::~FfmpegMpegTsUdpSink() {
    (void)Stop();
}

void FfmpegMpegTsUdpSink::Initialize(
    const FfmpegMpegTsUdpSinkConfig& config) {
    if (impl_ == nullptr) {
        throw std::runtime_error("MPEG-TS/UDP sink implementation is missing");
    }
    if (impl_->worker.joinable() || impl_->format != nullptr) {
        throw std::logic_error("MPEG-TS/UDP sink supports one lifecycle");
    }
    if (config.url.rfind("udp://", 0U) != 0U ||
        config.video_width <= 0 || config.video_height <= 0 ||
        config.video_bit_rate_bps <= 0 || config.video_fps_numerator <= 0 ||
        config.video_fps_denominator <= 0 ||
        config.hevc_annexb_codec_config.empty() || !config.audio.Valid() ||
        config.packet_queue_capacity == 0U || config.io_timeout_us <= 0) {
        throw std::invalid_argument("invalid MPEG-TS/UDP sink config");
    }

    impl_->config = config;
    impl_->queue =
        std::make_unique<BoundedQueue<NetworkPacket>>(config.packet_queue_capacity);
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot = {};
    }

    const unsigned runtime_avformat_major = avformat_version() >> 16U;
    const unsigned runtime_avcodec_major = avcodec_version() >> 16U;
    const unsigned runtime_avutil_major = avutil_version() >> 16U;
    if (runtime_avformat_major != LIBAVFORMAT_VERSION_MAJOR ||
        runtime_avcodec_major != LIBAVCODEC_VERSION_MAJOR ||
        runtime_avutil_major != LIBAVUTIL_VERSION_MAJOR) {
        throw std::runtime_error(
            "FFmpeg header/runtime major-version mismatch for MPEG-TS/UDP");
    }

    int result = avformat_alloc_output_context2(
        &impl_->format, nullptr, "mpegts", config.url.c_str());
    if (result < 0 || impl_->format == nullptr) {
        impl_->CloseFormat();
        throw std::runtime_error(
            "avformat_alloc_output_context2(mpegts) failed: " +
            AvErrorString(result));
    }

    impl_->video_stream = avformat_new_stream(impl_->format, nullptr);
    impl_->audio_stream = avformat_new_stream(impl_->format, nullptr);
    if (impl_->video_stream == nullptr || impl_->audio_stream == nullptr ||
        impl_->video_stream->codecpar == nullptr ||
        impl_->audio_stream->codecpar == nullptr) {
        impl_->CloseFormat();
        throw std::runtime_error("avformat_new_stream failed for MPEG-TS/UDP");
    }

    AVCodecParameters* video = impl_->video_stream->codecpar;
    video->codec_type = AVMEDIA_TYPE_VIDEO;
    video->codec_id = AV_CODEC_ID_HEVC;
    video->codec_tag = 0;
    video->width = config.video_width;
    video->height = config.video_height;
    video->bit_rate = config.video_bit_rate_bps;
    video->format = AV_PIX_FMT_YUV420P;
    if (!CopyExtradata(video, config.hevc_annexb_codec_config)) {
        impl_->CloseFormat();
        throw std::runtime_error("failed to copy HEVC config for MPEG-TS/UDP");
    }
    impl_->video_stream->time_base = AVRational{1, 90'000};
    impl_->video_stream->avg_frame_rate = AVRational{
        config.video_fps_numerator, config.video_fps_denominator};

    AVCodecParameters* audio = impl_->audio_stream->codecpar;
    audio->codec_type = AVMEDIA_TYPE_AUDIO;
    audio->codec_id = AV_CODEC_ID_AAC;
    audio->codec_tag = 0;
    audio->profile = FF_PROFILE_AAC_LOW;
    audio->sample_rate = static_cast<int>(config.audio.sample_rate_hz);
    audio->channels = static_cast<int>(config.audio.channels);
    audio->channel_layout = config.audio.channels == 1U
        ? AV_CH_LAYOUT_MONO : AV_CH_LAYOUT_STEREO;
    audio->bit_rate = config.audio.bit_rate_bps;
    audio->frame_size = static_cast<int>(config.audio.frame_size_frames);
    audio->initial_padding =
        static_cast<int>(config.audio.initial_padding_frames);
    if (!CopyExtradata(audio, config.audio.codec_config)) {
        impl_->CloseFormat();
        throw std::runtime_error("failed to copy AAC config for MPEG-TS/UDP");
    }
    impl_->audio_stream->time_base = AVRational{
        1, static_cast<int>(config.audio.sample_rate_hz)};

    impl_->format->flags |= AVFMT_FLAG_FLUSH_PACKETS;
    impl_->format->max_delay = 0;
    impl_->format->avoid_negative_ts = AVFMT_AVOID_NEG_TS_MAKE_NON_NEGATIVE;

    AVDictionary* io_options = nullptr;
    const std::string timeout = std::to_string(config.io_timeout_us);
    av_dict_set(&io_options, "rw_timeout", timeout.c_str(), 0);
    if ((impl_->format->oformat->flags & AVFMT_NOFILE) == 0) {
        result = avio_open2(
            &impl_->format->pb,
            config.url.c_str(),
            AVIO_FLAG_WRITE,
            nullptr,
            &io_options);
    }
    av_dict_free(&io_options);
    if (result < 0) {
        impl_->CloseFormat();
        throw std::runtime_error(
            "avio_open2(udp) failed: " + AvErrorString(result));
    }

    AVDictionary* mux_options = nullptr;
    // Re-emit PAT/PMT at random-access boundaries so a PC receiver can join
    // an already-running UDP stream at the next keyframe.
    av_dict_set(&mux_options, "mpegts_flags", "+resend_headers", 0);
    result = avformat_write_header(impl_->format, &mux_options);
    av_dict_free(&mux_options);
    if (result < 0) {
        impl_->CloseFormat();
        throw std::runtime_error(
            "avformat_write_header(mpegts) failed: " + AvErrorString(result));
    }

    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.started = true;
        impl_->snapshot.opened = true;
    }
    impl_->accepting.store(true, std::memory_order_release);
    impl_->running.store(true, std::memory_order_release);
    try {
        impl_->worker = std::thread(&Impl::WorkerMain, impl_.get());
    } catch (...) {
        impl_->accepting.store(false, std::memory_order_release);
        impl_->running.store(false, std::memory_order_release);
        impl_->CloseFormat();
        throw;
    }
}

bool FfmpegMpegTsUdpSink::Write(const EncodedPacket& packet) noexcept {
    if (impl_ == nullptr || packet.codec_config) {
        return impl_ != nullptr;
    }
    if (!impl_->accepting.load(std::memory_order_acquire) ||
        packet.bytes.empty() || packet.duration_us <= 0) {
        return false;
    }

    std::lock_guard<std::mutex> lock(impl_->video_mutex);
    {
        std::lock_guard<std::mutex> stats_lock(impl_->mutex);
        ++impl_->snapshot.video_fragments_received;
    }
    if (!impl_->have_pending_video) {
        impl_->pending_video = {};
        impl_->pending_video.kind = NetworkPacketKind::kVideo;
        impl_->pending_video.pts = packet.pts_us;
        impl_->pending_video.dts = packet.dts_us;
        impl_->pending_video.duration = packet.duration_us;
        impl_->pending_video.keyframe = packet.keyframe;
        impl_->have_pending_video = true;
    } else if (packet.pts_us != impl_->pending_video.pts ||
               packet.dts_us != impl_->pending_video.dts) {
        impl_->SetError(
            "MPP partition timestamp changed before network end_of_frame");
        return false;
    } else {
        impl_->pending_video.keyframe =
            impl_->pending_video.keyframe || packet.keyframe;
    }
    impl_->pending_video.data.insert(
        impl_->pending_video.data.end(), packet.bytes.begin(), packet.bytes.end());
    if (!packet.end_of_frame) {
        return true;
    }

    NetworkPacket complete = std::move(impl_->pending_video);
    impl_->pending_video = {};
    impl_->have_pending_video = false;
    if (!impl_->queue->TryPush(std::move(complete))) {
        impl_->SetError("MPEG-TS/UDP packet queue overloaded", true);
        return false;
    }
    std::lock_guard<std::mutex> stats_lock(impl_->mutex);
    ++impl_->snapshot.video_access_units_enqueued;
    return true;
}

void FfmpegMpegTsUdpSink::Flush() noexcept {
    // The worker enables AVFMT_FLAG_FLUSH_PACKETS. Touching AVIO here would
    // violate its single-thread ownership, so producer-side Flush is a no-op.
}

bool FfmpegMpegTsUdpSink::WriteAudio(
    const EncodedAudioPacket& packet) noexcept {
    if (impl_ == nullptr ||
        !impl_->accepting.load(std::memory_order_acquire) || !packet.Valid()) {
        return false;
    }
    NetworkPacket output;
    output.kind = NetworkPacketKind::kAudio;
    output.data = packet.data;
    output.pts = packet.pts_ns;
    output.dts = packet.dts_ns;
    output.duration = packet.duration_ns;
    output.keyframe = true;
    if (!impl_->queue->TryPush(std::move(output))) {
        impl_->SetError("MPEG-TS/UDP packet queue overloaded", true);
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    ++impl_->snapshot.audio_packets_enqueued;
    return true;
}

void FfmpegMpegTsUdpSink::FlushAudio() noexcept {
    Flush();
}

bool FfmpegMpegTsUdpSink::Stop() noexcept {
    if (impl_ == nullptr) {
        return false;
    }
    impl_->accepting.store(false, std::memory_order_release);
    if (impl_->queue != nullptr) {
        impl_->queue->Stop();
    }
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    } else if (impl_->format != nullptr) {
        impl_->CloseFormat();
    }
    impl_->running.store(false, std::memory_order_release);
    const FfmpegMpegTsUdpSinkSnapshot snapshot = Snapshot();
    return snapshot.finalized && !snapshot.fatal_error;
}

FfmpegMpegTsUdpSinkSnapshot FfmpegMpegTsUdpSink::Snapshot() const {
    if (impl_ == nullptr) {
        return {};
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    FfmpegMpegTsUdpSinkSnapshot copy = impl_->snapshot;
    copy.running = impl_->running.load(std::memory_order_acquire);
    if (impl_->queue != nullptr) {
        copy.packet_queue = impl_->queue->Snapshot();
    }
    return copy;
}

}  // namespace visionarm
