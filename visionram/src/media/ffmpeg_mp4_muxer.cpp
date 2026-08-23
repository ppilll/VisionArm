#include "media/ffmpeg_mp4_muxer.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/common.h>
#include <libavutil/error.h>
#include <libavutil/mem.h>
#include <libavutil/pixfmt.h>
#include <libavutil/version.h>
}

#include <algorithm>
#include <cstring>
#include <limits>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace visionarm {
namespace {

constexpr AVRational kMicrosecondsTimeBase{1, 1'000'000};
constexpr AVRational kNanosecondsTimeBase{1, 1'000'000'000};

[[nodiscard]] std::string AvErrorString(int error) {
    char text[AV_ERROR_MAX_STRING_SIZE]{};
    if (av_strerror(error, text, sizeof(text)) == 0) {
        return text;
    }
    std::ostringstream stream;
    stream << "FFmpeg error " << error;
    return stream.str();
}

void WriteLe32(std::uint8_t* out, std::uint32_t value) noexcept {
    out[0] = static_cast<std::uint8_t>(value & 0xFFU);
    out[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    out[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    out[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
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

}  // namespace

struct FfmpegMp4Muxer::Impl {
    FfmpegMp4MuxerConfig config;
    mutable std::mutex mutex;
    FfmpegMp4MuxerSnapshot snapshot;

    AVFormatContext* format = nullptr;
    AVStream* video_stream = nullptr;
    AVStream* audio_stream = nullptr;

    std::vector<std::uint8_t> pending_video;
    std::int64_t pending_video_pts_us = 0;
    std::int64_t pending_video_dts_us = 0;
    std::int64_t pending_video_duration_us = 0;
    bool pending_video_keyframe = false;
    bool have_pending_video = false;

    std::int64_t last_video_dts_us = -1;
    std::int64_t last_audio_dts_ns = -1;

    void SetErrorLocked(const std::string& error) noexcept {
        snapshot.fatal_error = true;
        snapshot.last_error = error;
    }

    void CloseLocked() noexcept {
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
        pending_video.clear();
        have_pending_video = false;
        snapshot.opened = false;
    }

    [[nodiscard]] bool WritePacketLocked(
        AVStream* stream,
        const std::uint8_t* data,
        std::size_t size,
        std::int64_t pts,
        std::int64_t dts,
        std::int64_t duration,
        AVRational source_time_base,
        bool keyframe,
        std::uint32_t skip_samples,
        std::uint32_t discard_padding) noexcept {
        if (format == nullptr || stream == nullptr || data == nullptr || size == 0U ||
            size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            SetErrorLocked("invalid mux packet input");
            ++snapshot.write_failures;
            return false;
        }

        AVPacket* packet = av_packet_alloc();
        if (packet == nullptr) {
            SetErrorLocked("av_packet_alloc failed");
            ++snapshot.write_failures;
            return false;
        }
        const int allocate_result = av_new_packet(packet, static_cast<int>(size));
        if (allocate_result < 0) {
            SetErrorLocked(
                "av_new_packet failed: " + AvErrorString(allocate_result));
            ++snapshot.write_failures;
            av_packet_free(&packet);
            return false;
        }
        std::memcpy(packet->data, data, size);
        packet->stream_index = stream->index;
        packet->pts = pts;
        packet->dts = dts;
        packet->duration = duration;
        if (keyframe) {
            packet->flags |= AV_PKT_FLAG_KEY;
        }
        av_packet_rescale_ts(packet, source_time_base, stream->time_base);

        if (skip_samples != 0U || discard_padding != 0U) {
            std::uint8_t* side_data = av_packet_new_side_data(
                packet, AV_PKT_DATA_SKIP_SAMPLES, 10);
            if (side_data == nullptr) {
                SetErrorLocked("av_packet_new_side_data(SKIP_SAMPLES) failed");
                ++snapshot.write_failures;
                av_packet_free(&packet);
                return false;
            }
            WriteLe32(side_data, skip_samples);
            WriteLe32(side_data + 4, discard_padding);
            side_data[8] = 0U;
            side_data[9] = 0U;
        }

        const int result = av_interleaved_write_frame(format, packet);
        av_packet_free(&packet);
        if (result < 0) {
            SetErrorLocked(
                "av_interleaved_write_frame failed: " + AvErrorString(result));
            ++snapshot.write_failures;
            return false;
        }
        return true;
    }

    [[nodiscard]] bool CommitVideoLocked() noexcept {
        if (!have_pending_video || pending_video.empty()) {
            SetErrorLocked("attempted to commit empty HEVC access unit");
            ++snapshot.write_failures;
            return false;
        }
        if (last_video_dts_us >= 0 && pending_video_dts_us < last_video_dts_us) {
            SetErrorLocked("video DTS regression before mux");
            ++snapshot.write_failures;
            return false;
        }

        const std::size_t bytes = pending_video.size();
        if (!WritePacketLocked(
                video_stream,
                pending_video.data(),
                pending_video.size(),
                pending_video_pts_us,
                pending_video_dts_us,
                pending_video_duration_us,
                kMicrosecondsTimeBase,
                pending_video_keyframe,
                0U,
                0U)) {
            return false;
        }

        if (snapshot.video_samples_written == 0U) {
            snapshot.first_video_pts_us = pending_video_pts_us;
        }
        snapshot.last_video_pts_us = pending_video_pts_us;
        last_video_dts_us = pending_video_dts_us;
        ++snapshot.video_samples_written;
        snapshot.video_bytes_written += bytes;

        pending_video.clear();
        have_pending_video = false;
        pending_video_keyframe = false;
        return true;
    }
};

FfmpegMp4Muxer::FfmpegMp4Muxer()
    : impl_(std::make_unique<Impl>()) {}

FfmpegMp4Muxer::~FfmpegMp4Muxer() {
    if (impl_ != nullptr) {
        (void)Finalize();
    }
}

void FfmpegMp4Muxer::Initialize(const FfmpegMp4MuxerConfig& config) {
    if (impl_ == nullptr) {
        throw std::runtime_error("FfmpegMp4Muxer implementation is missing");
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->CloseLocked();
    impl_->snapshot = {};

    if (config.path.empty() || config.video_width <= 0 ||
        config.video_height <= 0 || config.video_bit_rate_bps <= 0 ||
        config.video_fps_numerator <= 0 || config.video_fps_denominator <= 0 ||
        config.hevc_annexb_codec_config.empty() || !config.audio.Valid()) {
        throw std::invalid_argument("invalid FFmpeg MP4 muxer config");
    }
    impl_->config = config;

    const unsigned runtime_avformat_major = avformat_version() >> 16U;
    const unsigned runtime_avcodec_major = avcodec_version() >> 16U;
    const unsigned runtime_avutil_major = avutil_version() >> 16U;
    std::cerr << "MP4 mux stage=abi_check"
              << " headers=" << LIBAVFORMAT_VERSION_MAJOR << '/'
              << LIBAVCODEC_VERSION_MAJOR << '/' << LIBAVUTIL_VERSION_MAJOR
              << " runtime=" << runtime_avformat_major << '/'
              << runtime_avcodec_major << '/' << runtime_avutil_major << '\n';
    if (runtime_avformat_major != LIBAVFORMAT_VERSION_MAJOR ||
        runtime_avcodec_major != LIBAVCODEC_VERSION_MAJOR ||
        runtime_avutil_major != LIBAVUTIL_VERSION_MAJOR) {
        throw std::runtime_error(
            "FFmpeg header/runtime major-version mismatch; rebuild against the board Buildroot sysroot");
    }

    std::cerr << "MP4 mux stage=alloc_output_context" << '\n';
    int result = avformat_alloc_output_context2(
        &impl_->format, nullptr, "mp4", config.path.c_str());
    if (result < 0 || impl_->format == nullptr) {
        impl_->CloseLocked();
        throw std::runtime_error(
            "avformat_alloc_output_context2(mp4) failed: " +
            AvErrorString(result));
    }

    std::cerr << "MP4 mux stage=new_video_stream" << '\n';
    impl_->video_stream = avformat_new_stream(impl_->format, nullptr);
    std::cerr << "MP4 mux stage=new_audio_stream" << '\n';
    impl_->audio_stream = avformat_new_stream(impl_->format, nullptr);
    if (impl_->video_stream == nullptr || impl_->audio_stream == nullptr) {
        impl_->CloseLocked();
        throw std::runtime_error("avformat_new_stream failed");
    }
    if (impl_->video_stream->codecpar == nullptr ||
        impl_->audio_stream->codecpar == nullptr) {
        impl_->CloseLocked();
        throw std::runtime_error("FFmpeg stream codecpar is null");
    }

    std::cerr << "MP4 mux stage=video_codecpar" << '\n';
    AVCodecParameters* video = impl_->video_stream->codecpar;
    video->codec_type = AVMEDIA_TYPE_VIDEO;
    video->codec_id = AV_CODEC_ID_HEVC;
    // MPP may repeat parameter sets in-band at random-access pictures. 'hev1'
    // explicitly permits this while still carrying hvcC extradata in MP4.
    video->codec_tag = MKTAG('h', 'e', 'v', '1');
    video->width = config.video_width;
    video->height = config.video_height;
    video->bit_rate = config.video_bit_rate_bps;
    video->format = AV_PIX_FMT_YUV420P;
    if (!CopyExtradata(video, config.hevc_annexb_codec_config)) {
        impl_->CloseLocked();
        throw std::runtime_error("failed to copy HEVC VPS/SPS/PPS extradata");
    }
    impl_->video_stream->time_base = kMicrosecondsTimeBase;
    impl_->video_stream->avg_frame_rate = AVRational{
        config.video_fps_numerator, config.video_fps_denominator};

    std::cerr << "MP4 mux stage=audio_codecpar" << '\n';
    AVCodecParameters* audio = impl_->audio_stream->codecpar;
    audio->codec_type = AVMEDIA_TYPE_AUDIO;
    audio->codec_id = AV_CODEC_ID_AAC;
    audio->codec_tag = 0;
    audio->profile = FF_PROFILE_AAC_LOW;
    audio->sample_rate = static_cast<int>(config.audio.sample_rate_hz);
    audio->channels = static_cast<int>(config.audio.channels);
    audio->channel_layout = config.audio.channels == 1U ?
        AV_CH_LAYOUT_MONO : AV_CH_LAYOUT_STEREO;
    audio->bit_rate = config.audio.bit_rate_bps;
    audio->frame_size = static_cast<int>(config.audio.frame_size_frames);
    audio->initial_padding =
        static_cast<int>(config.audio.initial_padding_frames);
    if (!CopyExtradata(audio, config.audio.codec_config)) {
        impl_->CloseLocked();
        throw std::runtime_error("failed to copy AAC AudioSpecificConfig");
    }
    impl_->audio_stream->time_base = AVRational{
        1, static_cast<int>(config.audio.sample_rate_hz)};

    std::cerr << "MP4 mux stage=avio_open" << '\n';
    if ((impl_->format->oformat->flags & AVFMT_NOFILE) == 0) {
        result = avio_open(&impl_->format->pb, config.path.c_str(), AVIO_FLAG_WRITE);
        if (result < 0) {
            impl_->CloseLocked();
            throw std::runtime_error(
                "avio_open failed: " + AvErrorString(result));
        }
    }

    std::cerr << "MP4 mux stage=write_header" << '\n';
    result = avformat_write_header(impl_->format, nullptr);
    if (result < 0) {
        impl_->CloseLocked();
        throw std::runtime_error(
            "avformat_write_header failed: " + AvErrorString(result));
    }
    impl_->snapshot.opened = true;
    impl_->snapshot.header_written = true;
    std::cerr << "MP4 mux stage=ready" << '\n';
}

bool FfmpegMp4Muxer::Write(const EncodedPacket& packet) noexcept {
    if (impl_ == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->snapshot.opened || impl_->snapshot.finalized ||
        impl_->snapshot.fatal_error) {
        return false;
    }
    if (packet.codec_config) {
        return true;
    }
    if (packet.bytes.empty() || packet.duration_us <= 0) {
        impl_->SetErrorLocked("invalid encoded HEVC packet for mux");
        ++impl_->snapshot.write_failures;
        return false;
    }

    ++impl_->snapshot.video_fragments_received;
    if (!impl_->have_pending_video) {
        impl_->have_pending_video = true;
        impl_->pending_video_pts_us = packet.pts_us;
        impl_->pending_video_dts_us = packet.dts_us;
        impl_->pending_video_duration_us = packet.duration_us;
        impl_->pending_video_keyframe = packet.keyframe;
    } else {
        if (packet.pts_us != impl_->pending_video_pts_us ||
            packet.dts_us != impl_->pending_video_dts_us) {
            impl_->SetErrorLocked(
                "MPP partition packet timestamp changed before end_of_frame");
            ++impl_->snapshot.write_failures;
            return false;
        }
        impl_->pending_video_keyframe =
            impl_->pending_video_keyframe || packet.keyframe;
    }
    impl_->pending_video.insert(
        impl_->pending_video.end(), packet.bytes.begin(), packet.bytes.end());

    if (!packet.end_of_frame) {
        return true;
    }
    return impl_->CommitVideoLocked();
}

void FfmpegMp4Muxer::Flush() noexcept {
    if (impl_ == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->format != nullptr && impl_->format->pb != nullptr) {
        avio_flush(impl_->format->pb);
    }
}

bool FfmpegMp4Muxer::WriteAudio(
    const EncodedAudioPacket& packet) noexcept {
    if (impl_ == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->snapshot.opened || impl_->snapshot.finalized ||
        impl_->snapshot.fatal_error || !packet.Valid()) {
        return false;
    }
    if (impl_->last_audio_dts_ns >= 0 && packet.dts_ns < impl_->last_audio_dts_ns) {
        impl_->SetErrorLocked("audio DTS regression before mux");
        ++impl_->snapshot.write_failures;
        return false;
    }

    if (!impl_->WritePacketLocked(
            impl_->audio_stream,
            packet.data.data(),
            packet.data.size(),
            packet.pts_ns,
            packet.dts_ns,
            packet.duration_ns,
            kNanosecondsTimeBase,
            true,
            packet.skip_samples,
            packet.discard_padding_samples)) {
        return false;
    }

    if (impl_->snapshot.audio_packets_written == 0U) {
        impl_->snapshot.first_audio_pts_ns = packet.pts_ns;
    }
    impl_->snapshot.last_audio_pts_ns = packet.pts_ns;
    impl_->last_audio_dts_ns = packet.dts_ns;
    ++impl_->snapshot.audio_packets_written;
    impl_->snapshot.audio_bytes_written += packet.data.size();
    return true;
}

void FfmpegMp4Muxer::FlushAudio() noexcept {
    Flush();
}

bool FfmpegMp4Muxer::Finalize() noexcept {
    if (impl_ == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->snapshot.finalized) {
        return !impl_->snapshot.fatal_error;
    }
    if (impl_->format == nullptr) {
        return false;
    }
    if (impl_->have_pending_video) {
        impl_->SetErrorLocked("finalize reached with incomplete HEVC access unit");
        ++impl_->snapshot.write_failures;
        impl_->CloseLocked();
        return false;
    }

    const int result = av_write_trailer(impl_->format);
    if (result < 0) {
        impl_->SetErrorLocked(
            "av_write_trailer failed: " + AvErrorString(result));
        ++impl_->snapshot.write_failures;
        impl_->CloseLocked();
        return false;
    }
    if (impl_->format->pb != nullptr) {
        avio_flush(impl_->format->pb);
    }
    impl_->snapshot.finalized = true;
    impl_->CloseLocked();
    // CloseLocked marks opened=false but intentionally preserves the terminal
    // snapshot fields including finalized and all counters.
    return !impl_->snapshot.fatal_error;
}

FfmpegMp4MuxerSnapshot FfmpegMp4Muxer::Snapshot() const {
    if (impl_ == nullptr) {
        return {};
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->snapshot;
}

}  // namespace visionarm
