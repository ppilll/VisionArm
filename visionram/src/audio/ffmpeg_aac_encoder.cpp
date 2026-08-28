#include "audio/ffmpeg_aac_encoder.h"

#include "media/ffmpeg_log_control.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/channel_layout.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>
}

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace visionarm {
namespace {

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

[[nodiscard]] std::int64_t FramesToNs(
    std::uint64_t frames,
    std::uint32_t sample_rate_hz) {
    if (sample_rate_hz == 0U) {
        throw std::invalid_argument("sample rate must be non-zero");
    }
    if (frames >
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) /
            1'000'000'000ULL) {
        throw std::overflow_error("audio frame count is too large");
    }
    return static_cast<std::int64_t>(
        (frames * 1'000'000'000ULL) /
        static_cast<std::uint64_t>(sample_rate_hz));
}


[[nodiscard]] std::uint32_t ReadLe32(const std::uint8_t* data) noexcept {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

[[nodiscard]] bool SameFormat(
    const AudioStreamFormat& left,
    const AudioStreamFormat& right) noexcept {
    return left.sample_rate_hz == right.sample_rate_hz &&
           left.channels == right.channels &&
           left.sample_format == right.sample_format;
}

[[nodiscard]] bool CodecSupportsSampleRate(
    const AVCodec* codec,
    int sample_rate) noexcept {
    if (codec == nullptr || codec->supported_samplerates == nullptr) {
        return true;
    }
    for (const int* rate = codec->supported_samplerates; *rate != 0; ++rate) {
        if (*rate == sample_rate) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] bool CodecSupportsSampleFormat(
    const AVCodec* codec,
    AVSampleFormat sample_format) noexcept {
    if (codec == nullptr || codec->sample_fmts == nullptr) {
        return true;
    }
    for (const AVSampleFormat* format = codec->sample_fmts;
         *format != AV_SAMPLE_FMT_NONE; ++format) {
        if (*format == sample_format) {
            return true;
        }
    }
    return false;
}

}  // namespace

struct FfmpegAacEncoder::Impl {
    struct PendingChunk {
        TimedAudioChunk chunk;
        std::uint32_t consumed_frames = 0U;
    };

    struct SourceInterval {
        std::int64_t start_tick = 0;
        std::int64_t end_tick = 0;
        std::int64_t source_pts_ns = 0;
        std::uint64_t source_first_sample_frame_index = 0U;
        std::uint32_t source_frame_count = 0U;
        bool discontinuity_before = false;
    };

    FfmpegAacEncoderConfig config;
    AudioEncoderStreamInfo stream_info;
    AudioEncoderSnapshot snapshot;

    const AVCodec* codec = nullptr;
    AVCodecContext* context = nullptr;

    std::deque<PendingChunk> pending;
    std::uint32_t pending_frames = 0U;
    std::deque<SourceInterval> source_intervals;

    bool initialized = false;
    bool drained = false;
    bool have_input = false;
    std::uint64_t expected_next_input_frame_index = 0U;

    void SetError(const std::string& error) noexcept {
        ++snapshot.encode_failures;
        snapshot.last_error = error;
        snapshot.buffered_input_frames = pending_frames;
    }

    void CloseCodec() noexcept {
        if (context != nullptr) {
            avcodec_free_context(&context);
        }
        codec = nullptr;
    }

    [[nodiscard]] bool OpenCodec(std::string* error) {
        CloseCodec();

        codec = avcodec_find_encoder_by_name("aac");
        if (codec == nullptr || codec->id != AV_CODEC_ID_AAC) {
            if (error != nullptr) {
                *error = "FFmpeg native AAC encoder 'aac' was not found";
            }
            return false;
        }

        const int sample_rate = static_cast<int>(config.input_format.sample_rate_hz);
        if (!CodecSupportsSampleRate(codec, sample_rate)) {
            if (error != nullptr) {
                *error = "FFmpeg AAC encoder does not support configured sample rate";
            }
            return false;
        }
        if (!CodecSupportsSampleFormat(codec, AV_SAMPLE_FMT_FLTP)) {
            if (error != nullptr) {
                *error = "FFmpeg AAC encoder does not support FLTP input";
            }
            return false;
        }

        context = avcodec_alloc_context3(codec);
        if (context == nullptr) {
            if (error != nullptr) {
                *error = "avcodec_alloc_context3(aac) failed";
            }
            return false;
        }

        context->bit_rate = config.bit_rate_bps;
        context->sample_rate = sample_rate;
        context->sample_fmt = AV_SAMPLE_FMT_FLTP;
        context->channels = static_cast<int>(config.input_format.channels);
        context->channel_layout =
            config.input_format.channels == 1U ? AV_CH_LAYOUT_MONO :
                                                 AV_CH_LAYOUT_STEREO;
        context->time_base = AVRational{1, sample_rate};
        context->profile = FF_PROFILE_AAC_LOW;

        // Keep AudioSpecificConfig as encoder extradata for the later mux
        // layer. Raw EncodedAudioPacket::data remains an AAC access unit.
        context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

        const int result = avcodec_open2(context, codec, nullptr);
        if (result < 0) {
            if (error != nullptr) {
                *error = "avcodec_open2(aac) failed: " + AvErrorString(result);
            }
            CloseCodec();
            return false;
        }

        if (context->frame_size <= 0) {
            if (error != nullptr) {
                *error = "AAC encoder returned an invalid frame_size";
            }
            CloseCodec();
            return false;
        }
        if (context->extradata == nullptr || context->extradata_size <= 0) {
            if (error != nullptr) {
                *error = "AAC encoder did not provide AudioSpecificConfig extradata";
            }
            CloseCodec();
            return false;
        }

        AudioEncoderStreamInfo info;
        info.codec = EncodedAudioCodec::kAacLc;
        info.sample_rate_hz = config.input_format.sample_rate_hz;
        info.channels = config.input_format.channels;
        info.bit_rate_bps = static_cast<std::int32_t>(context->bit_rate);
        info.frame_size_frames = static_cast<std::uint32_t>(context->frame_size);
        info.initial_padding_frames =
            context->initial_padding > 0 ?
                static_cast<std::uint32_t>(context->initial_padding) : 0U;
        info.codec_config.assign(
            context->extradata,
            context->extradata + context->extradata_size);

        if (!info.Valid()) {
            if (error != nullptr) {
                *error = "AAC encoder returned incomplete stream metadata";
            }
            CloseCodec();
            return false;
        }

        if (!stream_info.codec_config.empty() &&
            stream_info.codec_config != info.codec_config) {
            if (error != nullptr) {
                *error = "AAC codec configuration changed after discontinuity reset";
            }
            CloseCodec();
            return false;
        }
        stream_info = std::move(info);
        drained = false;
        return true;
    }

    [[nodiscard]] bool BuildFrame(
        std::uint32_t source_frames,
        std::uint32_t codec_frames,
        AVFrame** output,
        SourceInterval* source,
        std::string* error) {
        if (output == nullptr || source == nullptr || context == nullptr ||
            source_frames == 0U || codec_frames < source_frames ||
            source_frames > pending_frames || pending.empty()) {
            if (error != nullptr) {
                *error = "invalid AAC frame assembly state";
            }
            return false;
        }

        PendingChunk& first = pending.front();
        const std::uint32_t first_offset = first.consumed_frames;
        source->source_first_sample_frame_index =
            first.chunk.raw.timing.first_sample_frame_index + first_offset;
        source->source_frame_count = source_frames;
        source->source_pts_ns =
            first.chunk.media.pts_ns +
            FramesToNs(first_offset, config.input_format.sample_rate_hz);
        source->discontinuity_before =
            first_offset == 0U && first.chunk.raw.timing.discontinuity_before;
        source->start_tick = av_rescale_q(
            source->source_pts_ns,
            kNanosecondsTimeBase,
            context->time_base);
        source->end_tick =
            source->start_tick + static_cast<std::int64_t>(source_frames);

        AVFrame* frame = av_frame_alloc();
        if (frame == nullptr) {
            if (error != nullptr) {
                *error = "av_frame_alloc failed";
            }
            return false;
        }

        frame->nb_samples = static_cast<int>(codec_frames);
        frame->format = context->sample_fmt;
        frame->sample_rate = context->sample_rate;
        frame->channels = context->channels;
        frame->channel_layout = context->channel_layout;
        frame->pts = source->start_tick;

        const int allocate_result = av_frame_get_buffer(frame, 0);
        if (allocate_result < 0) {
            if (error != nullptr) {
                *error = "av_frame_get_buffer failed: " +
                         AvErrorString(allocate_result);
            }
            av_frame_free(&frame);
            return false;
        }

        const int writable_result = av_frame_make_writable(frame);
        if (writable_result < 0) {
            if (error != nullptr) {
                *error = "av_frame_make_writable failed: " +
                         AvErrorString(writable_result);
            }
            av_frame_free(&frame);
            return false;
        }

        for (std::uint16_t channel = 0U;
             channel < config.input_format.channels; ++channel) {
            auto* plane = reinterpret_cast<float*>(frame->data[channel]);
            if (plane == nullptr) {
                if (error != nullptr) {
                    *error = "AAC FLTP frame plane is null";
                }
                av_frame_free(&frame);
                return false;
            }
            std::fill(plane, plane + codec_frames, 0.0F);
        }

        std::uint32_t destination_frame = 0U;
        std::uint32_t remaining = source_frames;
        for (const PendingChunk& pending_chunk : pending) {
            if (remaining == 0U) {
                break;
            }
            const std::uint32_t available =
                pending_chunk.chunk.raw.timing.frame_count -
                pending_chunk.consumed_frames;
            const std::uint32_t take = std::min(available, remaining);
            const std::uint32_t bytes_per_frame =
                pending_chunk.chunk.raw.format.BytesPerFrame();

            for (std::uint32_t frame_index = 0U;
                 frame_index < take; ++frame_index) {
                const std::uint32_t source_frame =
                    pending_chunk.consumed_frames + frame_index;
                const std::size_t frame_byte_offset =
                    static_cast<std::size_t>(source_frame) * bytes_per_frame;
                for (std::uint16_t channel = 0U;
                     channel < config.input_format.channels; ++channel) {
                    const std::size_t sample_byte_offset =
                        frame_byte_offset +
                        static_cast<std::size_t>(channel) * 2U;
                    const auto& pcm = pending_chunk.chunk.raw.pcm;
                    if (sample_byte_offset + 1U >= pcm.size()) {
                        if (error != nullptr) {
                            *error = "PCM byte range exceeded during AAC conversion";
                        }
                        av_frame_free(&frame);
                        return false;
                    }
                    const std::uint16_t unsigned_sample =
                        static_cast<std::uint16_t>(pcm[sample_byte_offset]) |
                        static_cast<std::uint16_t>(
                            static_cast<std::uint16_t>(pcm[sample_byte_offset + 1U])
                            << 8U);
                    const std::int16_t signed_sample =
                        static_cast<std::int16_t>(unsigned_sample);
                    auto* plane =
                        reinterpret_cast<float*>(frame->data[channel]);
                    plane[destination_frame + frame_index] =
                        static_cast<float>(signed_sample) / 32768.0F;
                }
            }

            destination_frame += take;
            remaining -= take;
        }

        if (remaining != 0U || destination_frame != source_frames) {
            if (error != nullptr) {
                *error = "not enough buffered PCM to assemble AAC frame";
            }
            av_frame_free(&frame);
            return false;
        }

        *output = frame;
        return true;
    }

    void ConsumePending(std::uint32_t frames) {
        if (frames > pending_frames) {
            throw std::logic_error("AAC PCM consume exceeds buffered frames");
        }
        std::uint32_t remaining = frames;
        while (remaining != 0U) {
            PendingChunk& front = pending.front();
            const std::uint32_t available =
                front.chunk.raw.timing.frame_count - front.consumed_frames;
            const std::uint32_t take = std::min(available, remaining);
            front.consumed_frames += take;
            remaining -= take;
            pending_frames -= take;
            if (front.consumed_frames == front.chunk.raw.timing.frame_count) {
                pending.pop_front();
            }
        }
        snapshot.buffered_input_frames = pending_frames;
    }

    [[nodiscard]] bool EmitPacket(
        AVPacket* av_packet,
        std::vector<EncodedAudioPacket>* packets,
        std::string* error) {
        if (av_packet == nullptr || packets == nullptr || context == nullptr) {
            if (error != nullptr) {
                *error = "invalid AAC packet output state";
            }
            return false;
        }
        if (av_packet->data == nullptr || av_packet->size <= 0) {
            // libavcodec may legally emit side-data-only packets. The V8.3
            // audio byte contract has no payload for those, so ignore them.
            return true;
        }
        if (av_packet->pts == AV_NOPTS_VALUE) {
            if (error != nullptr) {
                *error = "AAC encoder emitted a packet without PTS";
            }
            return false;
        }

        const std::int64_t packet_start_tick = av_packet->pts;
        const std::int64_t packet_duration_ticks =
            av_packet->duration > 0 ? av_packet->duration : context->frame_size;
        if (packet_duration_ticks <= 0) {
            if (error != nullptr) {
                *error = "AAC encoder emitted a packet without duration";
            }
            return false;
        }
        const std::int64_t packet_end_tick =
            packet_start_tick + packet_duration_ticks;

        EncodedAudioPacket encoded;
        encoded.codec = EncodedAudioCodec::kAacLc;
        encoded.data.assign(
            av_packet->data,
            av_packet->data + av_packet->size);
        encoded.pts_ns = av_rescale_q(
            av_packet->pts,
            context->time_base,
            kNanosecondsTimeBase);
        const std::int64_t dts =
            av_packet->dts == AV_NOPTS_VALUE ? av_packet->pts : av_packet->dts;
        encoded.dts_ns = av_rescale_q(
            dts,
            context->time_base,
            kNanosecondsTimeBase);
        encoded.duration_ns = av_rescale_q(
            packet_duration_ticks,
            context->time_base,
            kNanosecondsTimeBase);

        std::uint64_t source_frames_total = 0U;
        bool first_source_found = false;
        for (const SourceInterval& interval : source_intervals) {
            if (interval.end_tick <= packet_start_tick) {
                continue;
            }
            if (interval.start_tick >= packet_end_tick) {
                break;
            }

            const std::int64_t overlap_start =
                std::max(interval.start_tick, packet_start_tick);
            const std::int64_t overlap_end =
                std::min(interval.end_tick, packet_end_tick);
            if (overlap_end <= overlap_start) {
                continue;
            }

            const std::uint64_t overlap_frames =
                static_cast<std::uint64_t>(overlap_end - overlap_start);
            if (!first_source_found) {
                const std::uint64_t interval_offset =
                    static_cast<std::uint64_t>(overlap_start - interval.start_tick);
                encoded.has_source_samples = true;
                encoded.source_first_sample_frame_index =
                    interval.source_first_sample_frame_index + interval_offset;
                encoded.source_pts_ns =
                    interval.source_pts_ns +
                    FramesToNs(
                        interval_offset,
                        config.input_format.sample_rate_hz);
                encoded.discontinuity_before =
                    interval.discontinuity_before && interval_offset == 0U;
                first_source_found = true;
            }
            source_frames_total += overlap_frames;
        }

        if (source_frames_total >
            static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
            if (error != nullptr) {
                *error = "AAC packet source frame count overflow";
            }
            return false;
        }
        encoded.source_frame_count =
            static_cast<std::uint32_t>(source_frames_total);
        encoded.contains_encoder_padding =
            source_frames_total < static_cast<std::uint64_t>(packet_duration_ticks);

        int skip_side_data_size = 0;
        const std::uint8_t* skip_side_data = av_packet_get_side_data(
            av_packet, AV_PKT_DATA_SKIP_SAMPLES, &skip_side_data_size);
        if (skip_side_data != nullptr && skip_side_data_size >= 10) {
            encoded.skip_samples = ReadLe32(skip_side_data);
            encoded.discard_padding_samples = ReadLe32(skip_side_data + 4U);
        }
        if (!encoded.Valid()) {
            if (error != nullptr) {
                *error = "constructed EncodedAudioPacket failed contract validation";
            }
            return false;
        }

        packets->push_back(std::move(encoded));
        ++snapshot.emitted_packets;
        snapshot.emitted_bytes += static_cast<std::uint64_t>(av_packet->size);
        if (packets->back().contains_encoder_padding) {
            ++snapshot.encoder_padding_packets;
        }

        while (!source_intervals.empty() &&
               source_intervals.front().end_tick <= packet_end_tick) {
            source_intervals.pop_front();
        }
        return true;
    }

    [[nodiscard]] bool ReceiveAvailable(
        std::vector<EncodedAudioPacket>* packets,
        bool draining,
        std::string* error) {
        AVPacket* packet = av_packet_alloc();
        if (packet == nullptr) {
            if (error != nullptr) {
                *error = "av_packet_alloc failed";
            }
            return false;
        }

        bool ok = true;
        for (;;) {
            const int result = avcodec_receive_packet(context, packet);
            if (result == AVERROR(EAGAIN)) {
                if (draining) {
                    if (error != nullptr) {
                        *error = "AAC encoder returned EAGAIN while draining";
                    }
                    ok = false;
                }
                break;
            }
            if (result == AVERROR_EOF) {
                break;
            }
            if (result < 0) {
                if (error != nullptr) {
                    *error = "avcodec_receive_packet failed: " +
                             AvErrorString(result);
                }
                ok = false;
                break;
            }

            if (!EmitPacket(packet, packets, error)) {
                ok = false;
                av_packet_unref(packet);
                break;
            }
            av_packet_unref(packet);
        }

        av_packet_free(&packet);
        return ok;
    }

    [[nodiscard]] bool SubmitFrame(
        std::uint32_t source_frames,
        std::uint32_t codec_frames,
        std::vector<EncodedAudioPacket>* packets,
        std::string* error) {
        AVFrame* frame = nullptr;
        SourceInterval source;
        if (!BuildFrame(
                source_frames,
                codec_frames,
                &frame,
                &source,
                error)) {
            return false;
        }

        int result = avcodec_send_frame(context, frame);
        if (result == AVERROR(EAGAIN)) {
            if (!ReceiveAvailable(packets, false, error)) {
                av_frame_free(&frame);
                return false;
            }
            result = avcodec_send_frame(context, frame);
        }
        if (result < 0) {
            if (error != nullptr) {
                *error = "avcodec_send_frame failed: " + AvErrorString(result);
            }
            av_frame_free(&frame);
            return false;
        }

        source_intervals.push_back(source);
        ConsumePending(source_frames);
        ++snapshot.submitted_codec_frames;
        av_frame_free(&frame);

        return ReceiveAvailable(packets, false, error);
    }

    [[nodiscard]] bool DrainCodec(
        std::vector<EncodedAudioPacket>* packets,
        std::string* error) {
        const int send_result = avcodec_send_frame(context, nullptr);
        if (send_result < 0 && send_result != AVERROR_EOF) {
            if (error != nullptr) {
                *error = "avcodec_send_frame(NULL) failed: " +
                         AvErrorString(send_result);
            }
            return false;
        }
        if (!ReceiveAvailable(packets, true, error)) {
            return false;
        }
        if (!source_intervals.empty()) {
            if (error != nullptr) {
                *error = "AAC drain completed with source samples not represented by packets";
            }
            return false;
        }
        return true;
    }

    [[nodiscard]] bool DrainAndReopenForDiscontinuity(
        std::vector<EncodedAudioPacket>* packets,
        std::string* error) {
        if (pending_frames != 0U) {
            if (error != nullptr) {
                *error = "audio discontinuity arrived while a partial AAC frame was buffered";
            }
            return false;
        }
        if (!DrainCodec(packets, error)) {
            return false;
        }
        if (!OpenCodec(error)) {
            return false;
        }
        return true;
    }
};

FfmpegAacEncoder::FfmpegAacEncoder()
    : impl_(std::make_unique<Impl>()) {}

FfmpegAacEncoder::~FfmpegAacEncoder() {
    Shutdown();
}

void FfmpegAacEncoder::Initialize(const FfmpegAacEncoderConfig& config) {
    ApplyFfmpegLogLevel();
    Shutdown();
    impl_->snapshot = {};
    impl_->config = config;

    if (!config.input_format.Valid() ||
        config.input_format.sample_format != AudioSampleFormat::kS16LE ||
        (config.input_format.channels != 1U &&
         config.input_format.channels != 2U) ||
        config.bit_rate_bps <= 0) {
        throw std::invalid_argument("invalid FFmpeg AAC encoder config");
    }

    std::string error;
    if (!impl_->OpenCodec(&error)) {
        impl_->CloseCodec();
        throw std::runtime_error(error);
    }

    impl_->initialized = true;
    impl_->drained = false;
    impl_->snapshot.drained = false;
}

void FfmpegAacEncoder::Shutdown() noexcept {
    if (!impl_) {
        return;
    }
    impl_->initialized = false;
    impl_->drained = false;
    impl_->pending.clear();
    impl_->pending_frames = 0U;
    impl_->source_intervals.clear();
    impl_->stream_info = {};
    impl_->have_input = false;
    impl_->expected_next_input_frame_index = 0U;
    impl_->CloseCodec();
}

bool FfmpegAacEncoder::initialized() const noexcept {
    return impl_ != nullptr && impl_->initialized;
}

bool FfmpegAacEncoder::Encode(
    TimedAudioChunk chunk,
    std::vector<EncodedAudioPacket>* packets) noexcept {
    if (packets != nullptr) {
        packets->clear();
    }
    if (impl_ == nullptr || !impl_->initialized || impl_->drained ||
        packets == nullptr) {
        return false;
    }

    try {
        if (!chunk.Valid() ||
            !SameFormat(chunk.raw.format, impl_->config.input_format)) {
            impl_->SetError("invalid or mismatched TimedAudioChunk");
            return false;
        }

        const std::uint64_t first_index =
            chunk.raw.timing.first_sample_frame_index;
        const std::uint32_t frame_count = chunk.raw.timing.frame_count;
        const bool discontinuity = chunk.raw.timing.discontinuity_before;

        if (impl_->have_input && !discontinuity &&
            first_index != impl_->expected_next_input_frame_index) {
            impl_->SetError(
                "non-contiguous PCM sample index without discontinuity flag");
            return false;
        }

        if (impl_->have_input && discontinuity) {
            std::string error;
            if (!impl_->DrainAndReopenForDiscontinuity(packets, &error)) {
                impl_->SetError(error);
                packets->clear();
                return false;
            }
        }

        Impl::PendingChunk pending;
        pending.chunk = std::move(chunk);
        impl_->pending.push_back(std::move(pending));
        impl_->pending_frames += frame_count;
        impl_->snapshot.buffered_input_frames = impl_->pending_frames;

        ++impl_->snapshot.input_chunks;
        impl_->snapshot.input_frames += frame_count;
        impl_->snapshot.input_bytes +=
            static_cast<std::uint64_t>(frame_count) *
            impl_->config.input_format.BytesPerFrame();
        if (discontinuity) {
            ++impl_->snapshot.input_discontinuities;
        }

        impl_->have_input = true;
        impl_->expected_next_input_frame_index = first_index + frame_count;

        const std::uint32_t codec_frame_size =
            impl_->stream_info.frame_size_frames;
        while (impl_->pending_frames >= codec_frame_size) {
            std::string error;
            if (!impl_->SubmitFrame(
                    codec_frame_size,
                    codec_frame_size,
                    packets,
                    &error)) {
                impl_->SetError(error);
                packets->clear();
                return false;
            }
        }
        impl_->snapshot.last_error.clear();
        return true;
    } catch (const std::exception& error) {
        impl_->SetError(error.what());
        packets->clear();
        return false;
    } catch (...) {
        impl_->SetError("unknown FFmpeg AAC encode exception");
        packets->clear();
        return false;
    }
}

bool FfmpegAacEncoder::Flush(
    std::vector<EncodedAudioPacket>* packets) noexcept {
    if (packets != nullptr) {
        packets->clear();
    }
    if (impl_ == nullptr || !impl_->initialized || packets == nullptr) {
        return false;
    }
    if (impl_->drained) {
        return true;
    }

    try {
        if (impl_->pending_frames != 0U) {
            const std::uint32_t source_frames = impl_->pending_frames;
            std::uint32_t codec_frames = source_frames;
            if ((impl_->codec->capabilities & AV_CODEC_CAP_SMALL_LAST_FRAME) == 0) {
                codec_frames = impl_->stream_info.frame_size_frames;
            }
            std::string error;
            if (!impl_->SubmitFrame(
                    source_frames,
                    codec_frames,
                    packets,
                    &error)) {
                impl_->SetError(error);
                packets->clear();
                return false;
            }
        }

        std::string error;
        if (!impl_->DrainCodec(packets, &error)) {
            impl_->SetError(error);
            packets->clear();
            return false;
        }

        impl_->drained = true;
        impl_->snapshot.drained = true;
        impl_->snapshot.buffered_input_frames = 0U;
        impl_->snapshot.last_error.clear();
        return true;
    } catch (const std::exception& error) {
        impl_->SetError(error.what());
        packets->clear();
        return false;
    } catch (...) {
        impl_->SetError("unknown FFmpeg AAC flush exception");
        packets->clear();
        return false;
    }
}

AudioEncoderStreamInfo FfmpegAacEncoder::stream_info() const {
    if (impl_ == nullptr) {
        return {};
    }
    return impl_->stream_info;
}

AudioEncoderSnapshot FfmpegAacEncoder::snapshot() const {
    if (impl_ == nullptr) {
        return {};
    }
    AudioEncoderSnapshot snapshot = impl_->snapshot;
    snapshot.buffered_input_frames = impl_->pending_frames;
    snapshot.drained = impl_->drained;
    return snapshot;
}

}  // namespace visionarm
