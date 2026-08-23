#include "media/ffmpeg_mpegts_udp_muxer.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/mem.h>
#include <libavutil/pixfmt.h>
#include <libavutil/version.h>
}

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace visionarm {
namespace {

constexpr AVRational kMicrosecondsTimeBase{1, 1'000'000};
constexpr AVRational kNanosecondsTimeBase{1, 1'000'000'000};

constexpr std::size_t kAdtsHeaderSize = 7U;
constexpr std::size_t kAdtsMaxFrameSize = (1U << 13U) - 1U;

[[nodiscard]] int AdtsSampleRateIndex(std::uint32_t sample_rate_hz) noexcept {
    // ISO/IEC 14496-3 samplingFrequencyIndex table used by ADTS.
    constexpr std::array<std::uint32_t, 13> kRates{
        96000U, 88200U, 64000U, 48000U, 44100U, 32000U, 24000U,
        22050U, 16000U, 12000U, 11025U, 8000U, 7350U};
    for (std::size_t index = 0; index < kRates.size(); ++index) {
        if (kRates[index] == sample_rate_hz) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

[[nodiscard]] bool BuildAdtsFrame(
    const AudioEncoderStreamInfo& audio,
    const std::uint8_t* payload,
    std::size_t payload_size,
    std::vector<std::uint8_t>* framed) {
    if (framed == nullptr || payload == nullptr || payload_size == 0U ||
        audio.codec != EncodedAudioCodec::kAacLc || audio.channels == 0U ||
        audio.channels > 7U) {
        return false;
    }

    const int sample_rate_index = AdtsSampleRateIndex(audio.sample_rate_hz);
    const std::size_t frame_size = payload_size + kAdtsHeaderSize;
    if (sample_rate_index < 0 || frame_size > kAdtsMaxFrameSize) {
        return false;
    }

    // ADTS fixed/variable header, MPEG-4 AAC-LC, no CRC, one raw_data_block.
    // profile stores audioObjectType - 1, therefore AAC-LC (AOT=2) => 1.
    constexpr std::uint8_t kProfileAacLc = 1U;
    const std::uint8_t channel_configuration =
        static_cast<std::uint8_t>(audio.channels);

    framed->resize(frame_size);
    std::uint8_t* header = framed->data();
    header[0] = 0xFFU;
    header[1] = 0xF1U;  // sync, MPEG-4, layer=0, protection_absent=1
    header[2] = static_cast<std::uint8_t>(
        (kProfileAacLc << 6U) |
        (static_cast<std::uint8_t>(sample_rate_index) << 2U) |
        ((channel_configuration >> 2U) & 0x01U));
    header[3] = static_cast<std::uint8_t>(
        ((channel_configuration & 0x03U) << 6U) |
        ((frame_size >> 11U) & 0x03U));
    header[4] = static_cast<std::uint8_t>((frame_size >> 3U) & 0xFFU);
    header[5] = static_cast<std::uint8_t>(
        ((frame_size & 0x07U) << 5U) | 0x1FU);
    header[6] = 0xFCU;  // VBR buffer fullness=0x7ff, raw_data_blocks=0

    std::memcpy(framed->data() + kAdtsHeaderSize, payload, payload_size);
    return true;
}

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

}  // namespace

struct FfmpegMpegTsUdpMuxer::Impl {
    FfmpegMpegTsUdpMuxerConfig config;
    mutable std::mutex mutex;
    FfmpegMpegTsUdpMuxerSnapshot snapshot;

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
        bool keyframe) noexcept {
        if (format == nullptr || stream == nullptr || data == nullptr ||
            size == 0U ||
            size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            SetErrorLocked("invalid MPEG-TS packet input");
            ++snapshot.write_failures;
            return false;
        }

        AVPacket* packet = av_packet_alloc();
        if (packet == nullptr) {
            SetErrorLocked("av_packet_alloc failed");
            ++snapshot.write_failures;
            return false;
        }
        const int allocation_result =
            av_new_packet(packet, static_cast<int>(size));
        if (allocation_result < 0) {
            SetErrorLocked(
                "av_new_packet failed: " + AvErrorString(allocation_result));
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

        // avformat_write_header() may replace the requested stream time base
        // with MPEG-TS' 90 kHz clock. Always rescale against the post-header
        // stream->time_base rather than assuming it stayed unchanged.
        av_packet_rescale_ts(packet, source_time_base, stream->time_base);

        // V8.4 live MPEG-TS/UDP uses a single muxer mutex, so packets already
        // enter libavformat serially. Use av_write_frame() here instead of the
        // generic interleaver: av_interleaved_write_frame() is allowed to hold
        // packets internally waiting for the other stream, which is undesirable
        // during live receiver probing. MPEG-TS tolerates live arrival-order
        // interleaving as long as DTS is monotonic within each stream.
        const int result = av_write_frame(format, packet);
        av_packet_free(&packet);
        if (result < 0) {
            SetErrorLocked(
                "av_write_frame failed: " + AvErrorString(result));
            ++snapshot.write_failures;
            return false;
        }

        // FFmpeg's MPEG-TS muxer buffers small audio access units into a PES
        // payload. Force that muxer-local buffer onto the wire after each AAC
        // packet so a newly-started UDP receiver sees a decodable ADTS frame
        // immediately instead of only seeing PMT stream_type=0x0f. This NULL
        // flush is the documented av_write_frame() mechanism for muxer buffers.
        if (stream == audio_stream) {
            const int flush_result = av_write_frame(format, nullptr);
            if (flush_result < 0) {
                SetErrorLocked(
                    "av_write_frame(mux flush) failed: " +
                    AvErrorString(flush_result));
                ++snapshot.write_failures;
                return false;
            }
            if (format->pb != nullptr) {
                avio_flush(format->pb);
            }
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
            SetErrorLocked("video DTS regression before network mux");
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
                pending_video_keyframe)) {
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

FfmpegMpegTsUdpMuxer::FfmpegMpegTsUdpMuxer()
    : impl_(std::make_unique<Impl>()) {}

FfmpegMpegTsUdpMuxer::~FfmpegMpegTsUdpMuxer() {
    if (impl_ != nullptr) {
        (void)Finalize();
    }
}

void FfmpegMpegTsUdpMuxer::Initialize(
    const FfmpegMpegTsUdpMuxerConfig& config) {
    if (impl_ == nullptr) {
        throw std::runtime_error(
            "FfmpegMpegTsUdpMuxer implementation is missing");
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->CloseLocked();
    impl_->snapshot = {};

    if (config.url.rfind("udp://", 0) != 0 || config.video_width <= 0 ||
        config.video_height <= 0 || config.video_bit_rate_bps <= 0 ||
        config.video_fps_numerator <= 0 || config.video_fps_denominator <= 0 ||
        config.hevc_annexb_codec_config.empty() || !config.audio.Valid() ||
        config.audio.codec != EncodedAudioCodec::kAacLc ||
        config.audio.channels > 7U ||
        AdtsSampleRateIndex(config.audio.sample_rate_hz) < 0) {
        throw std::invalid_argument("invalid FFmpeg MPEG-TS/UDP muxer config");
    }
    impl_->config = config;

    const unsigned runtime_avformat_major = avformat_version() >> 16U;
    const unsigned runtime_avcodec_major = avcodec_version() >> 16U;
    const unsigned runtime_avutil_major = avutil_version() >> 16U;
    std::cerr << "MPEGTS/UDP stage=abi_check"
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

    int result = avformat_alloc_output_context2(
        &impl_->format, nullptr, "mpegts", config.url.c_str());
    if (result < 0 || impl_->format == nullptr) {
        impl_->CloseLocked();
        throw std::runtime_error(
            "avformat_alloc_output_context2(mpegts) failed: " +
            AvErrorString(result));
    }

    impl_->video_stream = avformat_new_stream(impl_->format, nullptr);
    impl_->audio_stream = avformat_new_stream(impl_->format, nullptr);
    if (impl_->video_stream == nullptr || impl_->audio_stream == nullptr ||
        impl_->video_stream->codecpar == nullptr ||
        impl_->audio_stream->codecpar == nullptr) {
        impl_->CloseLocked();
        throw std::runtime_error("avformat_new_stream failed");
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
        impl_->CloseLocked();
        throw std::runtime_error("failed to copy HEVC VPS/SPS/PPS extradata");
    }
    impl_->video_stream->time_base = kMicrosecondsTimeBase;
    impl_->video_stream->avg_frame_rate = AVRational{
        config.video_fps_numerator, config.video_fps_denominator};

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
    // Deliberately do NOT copy AAC AudioSpecificConfig into the outer
    // MPEG-TS stream. V8.4 sends fully framed ADTS AAC access units, and ADTS
    // carries profile/sample-rate/channel configuration in-band. In FFmpeg
    // 4.4, non-empty AAC extradata makes mpegtsenc allocate an internal ADTS
    // sub-muxer for raw AAC. Leaving extradata empty makes our explicit ADTS
    // framing the single source of truth and turns any accidental raw-AAC
    // packet into an immediate mux error instead of a second implicit path.
    impl_->audio_stream->time_base = AVRational{
        1, static_cast<int>(config.audio.sample_rate_hz)};

    if ((impl_->format->oformat->flags & AVFMT_NOFILE) == 0) {
        // Protocol options such as pkt_size, buffer_size and connect are kept in
        // the URL so the exact board-side transport policy is visible in the
        // launch command and can be changed without recompiling.
        result = avio_open2(
            &impl_->format->pb,
            config.url.c_str(),
            AVIO_FLAG_WRITE,
            nullptr,
            nullptr);
        if (result < 0) {
            impl_->CloseLocked();
            throw std::runtime_error(
                "avio_open2(udp) failed: " + AvErrorString(result));
        }
    }

    result = avformat_write_header(impl_->format, nullptr);
    if (result < 0) {
        impl_->CloseLocked();
        throw std::runtime_error(
            "avformat_write_header(mpegts) failed: " + AvErrorString(result));
    }

    // Flush protocol writes promptly for live transport. This complements the
    // explicit MPEG-TS audio-PES flush performed after each AAC packet.
    impl_->format->flags |= AVFMT_FLAG_FLUSH_PACKETS;

    impl_->snapshot.opened = true;
    impl_->snapshot.header_written = true;
    std::cerr << "MPEGTS/UDP stage=ready mode=direct_write_explicit_adts url=" << config.url
              << " video_tb=" << impl_->video_stream->time_base.num << '/'
              << impl_->video_stream->time_base.den
              << " audio_tb=" << impl_->audio_stream->time_base.num << '/'
              << impl_->audio_stream->time_base.den << '\n';
}

bool FfmpegMpegTsUdpMuxer::Write(const EncodedPacket& packet) noexcept {
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
        impl_->SetErrorLocked("invalid encoded HEVC packet for network mux");
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

void FfmpegMpegTsUdpMuxer::Flush() noexcept {
    if (impl_ == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->format != nullptr && impl_->format->pb != nullptr) {
        avio_flush(impl_->format->pb);
    }
}

bool FfmpegMpegTsUdpMuxer::WriteAudio(
    const EncodedAudioPacket& packet) noexcept {
    if (impl_ == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->snapshot.opened || impl_->snapshot.finalized ||
        impl_->snapshot.fatal_error || !packet.Valid()) {
        return false;
    }
    if (impl_->last_audio_dts_ns >= 0 &&
        packet.dts_ns < impl_->last_audio_dts_ns) {
        impl_->SetErrorLocked("audio DTS regression before network mux");
        ++impl_->snapshot.write_failures;
        return false;
    }

    // V8.3 AAC packets intentionally own raw MPEG-4 AAC access units because
    // MP4 needs AudioSpecificConfig + raw access units. MPEG-TS receivers, on
    // the other hand, discover sample rate/channel configuration from ADTS.
    // Frame explicitly here instead of relying on libavformat's internal
    // raw-AAC -> ADTS helper; this keeps the packet contract deterministic
    // across the board's FFmpeg 4.4 Buildroot configuration.
    std::vector<std::uint8_t> adts_frame;
    if (!BuildAdtsFrame(
            impl_->config.audio,
            packet.data.data(),
            packet.data.size(),
            &adts_frame)) {
        impl_->SetErrorLocked("failed to build AAC ADTS frame for MPEG-TS");
        ++impl_->snapshot.write_failures;
        return false;
    }

    if (impl_->snapshot.audio_packets_written == 0U) {
        std::cerr << "MPEGTS/UDP stage=first_audio_adts"
                  << " bytes=" << adts_frame.size()
                  << " header=";
        const std::size_t header_bytes =
            std::min<std::size_t>(kAdtsHeaderSize, adts_frame.size());
        for (std::size_t i = 0; i < header_bytes; ++i) {
            if (i != 0U) {
                std::cerr << ':';
            }
            std::cerr << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<unsigned>(adts_frame[i]);
        }
        std::cerr << std::dec << std::setfill(' ')
                  << " pts_ns=" << packet.pts_ns
                  << " dts_ns=" << packet.dts_ns
                  << " duration_ns=" << packet.duration_ns << '\n';
    }

    if (!impl_->WritePacketLocked(
            impl_->audio_stream,
            adts_frame.data(),
            adts_frame.size(),
            packet.pts_ns,
            packet.dts_ns,
            packet.duration_ns,
            kNanosecondsTimeBase,
            true)) {
        return false;
    }

    if (impl_->snapshot.audio_packets_written == 0U) {
        impl_->snapshot.first_audio_pts_ns = packet.pts_ns;
    }
    impl_->snapshot.last_audio_pts_ns = packet.pts_ns;
    impl_->last_audio_dts_ns = packet.dts_ns;
    ++impl_->snapshot.audio_packets_written;
    impl_->snapshot.audio_bytes_written += adts_frame.size();
    return true;
}

void FfmpegMpegTsUdpMuxer::FlushAudio() noexcept {
    Flush();
}

bool FfmpegMpegTsUdpMuxer::Finalize() noexcept {
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
        impl_->SetErrorLocked(
            "finalize reached with incomplete HEVC access unit");
        ++impl_->snapshot.write_failures;
        impl_->CloseLocked();
        return false;
    }

    const int result = av_write_trailer(impl_->format);
    if (result < 0) {
        impl_->SetErrorLocked(
            "av_write_trailer(mpegts) failed: " + AvErrorString(result));
        ++impl_->snapshot.write_failures;
        impl_->CloseLocked();
        return false;
    }
    if (impl_->format->pb != nullptr) {
        avio_flush(impl_->format->pb);
    }
    impl_->snapshot.finalized = true;
    impl_->CloseLocked();
    return !impl_->snapshot.fatal_error;
}

FfmpegMpegTsUdpMuxerSnapshot FfmpegMpegTsUdpMuxer::Snapshot() const {
    if (impl_ == nullptr) {
        return {};
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->snapshot;
}

}  // namespace visionarm
