#include "audio/ffmpeg_aac_encoder.h"

#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string input_path;
    std::string output_path;
    std::uint32_t sample_rate_hz = 48'000U;
    std::uint16_t channels = 2U;
    std::int32_t bit_rate_bps = 128'000;
    std::uint32_t chunk_frames = 1'024U;
};

[[nodiscard]] std::int64_t FramesToNs(
    std::uint64_t frames,
    std::uint32_t sample_rate_hz) {
    return static_cast<std::int64_t>(
        (frames * 1'000'000'000ULL) /
        static_cast<std::uint64_t>(sample_rate_hz));
}

[[nodiscard]] std::string Hex(const std::vector<std::uint8_t>& bytes) {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const std::uint8_t byte : bytes) {
        stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
}

[[nodiscard]] int AacSampleRateIndex(std::uint32_t sample_rate_hz) {
    constexpr std::array<std::uint32_t, 13> kRates{
        96'000U, 88'200U, 64'000U, 48'000U, 44'100U, 32'000U, 24'000U,
        22'050U, 16'000U, 12'000U, 11'025U, 8'000U, 7'350U};
    for (std::size_t index = 0U; index < kRates.size(); ++index) {
        if (kRates[index] == sample_rate_hz) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

[[nodiscard]] bool WriteAdtsPacket(
    std::ofstream* output,
    const visionarm::AudioEncoderStreamInfo& info,
    const visionarm::EncodedAudioPacket& packet,
    std::string* error) {
    if (output == nullptr || !output->good() || !packet.Valid()) {
        if (error != nullptr) {
            *error = "invalid ADTS output state";
        }
        return false;
    }

    const int frequency_index = AacSampleRateIndex(info.sample_rate_hz);
    if (frequency_index < 0 || info.channels == 0U || info.channels > 7U) {
        if (error != nullptr) {
            *error = "AAC stream cannot be represented by the standalone ADTS writer";
        }
        return false;
    }

    constexpr std::size_t kAdtsHeaderBytes = 7U;
    const std::size_t frame_length = kAdtsHeaderBytes + packet.data.size();
    if (frame_length > 0x1FFFU) {
        if (error != nullptr) {
            *error = "AAC access unit is too large for ADTS frame length field";
        }
        return false;
    }

    // MPEG-4 AAC-LC, no CRC. The production EncodedAudioPacket remains raw
    // AAC; this ADTS wrapper exists only so the standalone probe can be opened
    // directly by ffprobe/ffplay.
    constexpr std::uint8_t kAacLcAdtsProfile = 1U;  // object_type(2) - 1
    const std::uint8_t channel_config = static_cast<std::uint8_t>(info.channels);
    std::array<std::uint8_t, kAdtsHeaderBytes> header{};
    header[0] = 0xFFU;
    header[1] = 0xF1U;
    header[2] = static_cast<std::uint8_t>(
        static_cast<std::uint8_t>(kAacLcAdtsProfile << 6U) |
        static_cast<std::uint8_t>(frequency_index << 2U) |
        static_cast<std::uint8_t>((channel_config >> 2U) & 0x01U));
    header[3] = static_cast<std::uint8_t>(
        static_cast<std::uint8_t>((channel_config & 0x03U) << 6U) |
        static_cast<std::uint8_t>((frame_length >> 11U) & 0x03U));
    header[4] = static_cast<std::uint8_t>((frame_length >> 3U) & 0xFFU);
    header[5] = static_cast<std::uint8_t>(
        static_cast<std::uint8_t>((frame_length & 0x07U) << 5U) | 0x1FU);
    header[6] = 0xFCU;

    output->write(
        reinterpret_cast<const char*>(header.data()),
        static_cast<std::streamsize>(header.size()));
    output->write(
        reinterpret_cast<const char*>(packet.data.data()),
        static_cast<std::streamsize>(packet.data.size()));
    if (!output->good()) {
        if (error != nullptr) {
            *error = "failed to write ADTS output";
        }
        return false;
    }
    return true;
}

void PrintUsage(const char* program) {
    std::cerr
        << "Usage: " << program
        << " <input_s16le.pcm> <output.aac>"
        << " [--sample-rate 48000] [--channels 2]"
        << " [--bitrate 128000] [--chunk-frames 1024]\n";
}

[[nodiscard]] bool ParseUnsigned(
    const std::string& text,
    std::uint64_t* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }
    try {
        std::size_t consumed = 0U;
        const unsigned long long parsed = std::stoull(text, &consumed, 10);
        if (consumed != text.size()) {
            return false;
        }
        *value = static_cast<std::uint64_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

[[nodiscard]] bool ParseOptions(
    int argc,
    char** argv,
    Options* options) {
    if (options == nullptr || argc < 3) {
        return false;
    }
    options->input_path = argv[1];
    options->output_path = argv[2];

    for (int index = 3; index < argc; index += 2) {
        if (index + 1 >= argc) {
            return false;
        }
        const std::string key = argv[index];
        std::uint64_t value = 0U;
        if (!ParseUnsigned(argv[index + 1], &value)) {
            return false;
        }

        if (key == "--sample-rate" &&
            value <= std::numeric_limits<std::uint32_t>::max()) {
            options->sample_rate_hz = static_cast<std::uint32_t>(value);
        } else if (key == "--channels" &&
                   value <= std::numeric_limits<std::uint16_t>::max()) {
            options->channels = static_cast<std::uint16_t>(value);
        } else if (key == "--bitrate" &&
                   value <= static_cast<std::uint64_t>(
                       std::numeric_limits<std::int32_t>::max())) {
            options->bit_rate_bps = static_cast<std::int32_t>(value);
        } else if (key == "--chunk-frames" &&
                   value <= std::numeric_limits<std::uint32_t>::max()) {
            options->chunk_frames = static_cast<std::uint32_t>(value);
        } else {
            return false;
        }
    }

    return options->sample_rate_hz != 0U &&
           (options->channels == 1U || options->channels == 2U) &&
           options->bit_rate_bps > 0 && options->chunk_frames != 0U;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, &options)) {
        PrintUsage(argv[0]);
        return 2;
    }

    try {
        visionarm::FfmpegAacEncoderConfig config;
        config.input_format.sample_rate_hz = options.sample_rate_hz;
        config.input_format.channels = options.channels;
        config.input_format.sample_format = visionarm::AudioSampleFormat::kS16LE;
        config.bit_rate_bps = options.bit_rate_bps;

        visionarm::FfmpegAacEncoder encoder;
        encoder.Initialize(config);
        const visionarm::AudioEncoderStreamInfo info = encoder.stream_info();
        if (!info.Valid()) {
            throw std::runtime_error("encoder stream info is invalid");
        }

        std::ifstream input(options.input_path, std::ios::binary);
        if (!input) {
            throw std::runtime_error("failed to open input PCM: " + options.input_path);
        }
        std::ofstream output(options.output_path, std::ios::binary | std::ios::trunc);
        if (!output) {
            throw std::runtime_error("failed to open output AAC: " + options.output_path);
        }
        const std::string manifest_path = options.output_path + ".packets.tsv";
        std::ofstream manifest(manifest_path, std::ios::trunc);
        if (!manifest) {
            throw std::runtime_error("failed to open packet manifest: " + manifest_path);
        }
        manifest
            << "packet\tpts_ns\tdts_ns\tduration_ns\thas_source"
            << "\tsource_pts_ns\tsource_first_frame\tsource_frames"
            << "\tdiscontinuity_before\tencoder_padding\tpayload_bytes\n";

        std::cout
            << "audio_codec=AAC-LC\n"
            << "sample_rate_hz=" << info.sample_rate_hz << '\n'
            << "channels=" << info.channels << '\n'
            << "bit_rate_bps=" << info.bit_rate_bps << '\n'
            << "codec_frame_size_frames=" << info.frame_size_frames << '\n'
            << "initial_padding_frames=" << info.initial_padding_frames << '\n'
            << "codec_config_hex=" << Hex(info.codec_config) << '\n'
            << "input_chunk_frames=" << options.chunk_frames << '\n';

        const std::uint32_t bytes_per_frame = config.input_format.BytesPerFrame();
        const std::size_t requested_bytes =
            static_cast<std::size_t>(options.chunk_frames) * bytes_per_frame;
        std::vector<std::uint8_t> buffer(requested_bytes);

        std::uint64_t input_frames = 0U;
        std::uint64_t input_bytes = 0U;
        std::uint64_t chunk_sequence = 0U;
        std::uint64_t packet_index = 0U;
        std::uint64_t packet_source_frames = 0U;
        std::uint64_t encoded_payload_bytes = 0U;
        bool have_packet_pts = false;
        std::int64_t previous_packet_pts_ns = 0;
        std::int64_t previous_packet_dts_ns = 0;
        bool have_source_packet = false;
        std::uint64_t previous_source_end_frame = 0U;

        auto consume_packets = [&](const std::vector<visionarm::EncodedAudioPacket>& packets) {
            for (const auto& packet : packets) {
                if (!packet.Valid()) {
                    throw std::runtime_error("encoder emitted invalid packet contract");
                }
                if (have_packet_pts &&
                    (packet.pts_ns <= previous_packet_pts_ns ||
                     packet.dts_ns <= previous_packet_dts_ns)) {
                    throw std::runtime_error("AAC packet PTS/DTS did not increase monotonically");
                }
                have_packet_pts = true;
                previous_packet_pts_ns = packet.pts_ns;
                previous_packet_dts_ns = packet.dts_ns;

                if (packet.has_source_samples) {
                    if (have_source_packet &&
                        packet.source_first_sample_frame_index !=
                            previous_source_end_frame) {
                        throw std::runtime_error(
                            "encoded packet source sample mapping is not contiguous");
                    }
                    have_source_packet = true;
                    previous_source_end_frame =
                        packet.source_first_sample_frame_index +
                        packet.source_frame_count;
                    packet_source_frames += packet.source_frame_count;
                }

                std::string write_error;
                if (!WriteAdtsPacket(&output, info, packet, &write_error)) {
                    throw std::runtime_error(write_error);
                }

                manifest
                    << packet_index << '\t'
                    << packet.pts_ns << '\t'
                    << packet.dts_ns << '\t'
                    << packet.duration_ns << '\t'
                    << (packet.has_source_samples ? 1 : 0) << '\t'
                    << packet.source_pts_ns << '\t'
                    << packet.source_first_sample_frame_index << '\t'
                    << packet.source_frame_count << '\t'
                    << (packet.discontinuity_before ? 1 : 0) << '\t'
                    << (packet.contains_encoder_padding ? 1 : 0) << '\t'
                    << packet.data.size() << '\n';

                encoded_payload_bytes += packet.data.size();
                ++packet_index;
            }
        };

        for (;;) {
            input.read(
                reinterpret_cast<char*>(buffer.data()),
                static_cast<std::streamsize>(buffer.size()));
            const std::streamsize read_bytes = input.gcount();
            if (read_bytes == 0) {
                if (input.bad()) {
                    throw std::runtime_error("failed while reading input PCM");
                }
                break;
            }
            if (read_bytes < 0 ||
                (static_cast<std::uint64_t>(read_bytes) % bytes_per_frame) != 0U) {
                throw std::runtime_error(
                    "input PCM length is not aligned to one interleaved sample frame");
            }

            const std::uint32_t frames = static_cast<std::uint32_t>(
                static_cast<std::uint64_t>(read_bytes) / bytes_per_frame);
            visionarm::TimedAudioChunk chunk;
            chunk.raw.format = config.input_format;
            chunk.raw.timing.chunk_sequence = chunk_sequence;
            chunk.raw.timing.first_sample_frame_index = input_frames;
            chunk.raw.timing.frame_count = frames;
            chunk.raw.pcm.assign(
                buffer.begin(),
                buffer.begin() + static_cast<std::size_t>(read_bytes));
            chunk.media.pts_ns = FramesToNs(input_frames, options.sample_rate_hz);
            chunk.media.duration_ns = FramesToNs(frames, options.sample_rate_hz);
            chunk.media.first_sample_mono_ns = chunk.media.pts_ns;
            chunk.media.observed_first_sample_mono_ns = chunk.media.pts_ns;

            std::vector<visionarm::EncodedAudioPacket> packets;
            if (!encoder.Encode(std::move(chunk), &packets)) {
                const auto snapshot = encoder.snapshot();
                throw std::runtime_error(
                    "AAC Encode failed: " + snapshot.last_error);
            }
            consume_packets(packets);

            input_frames += frames;
            input_bytes += static_cast<std::uint64_t>(read_bytes);
            ++chunk_sequence;

            if (input.eof()) {
                break;
            }
        }

        std::vector<visionarm::EncodedAudioPacket> final_packets;
        if (!encoder.Flush(&final_packets)) {
            const auto snapshot = encoder.snapshot();
            throw std::runtime_error(
                "AAC Flush failed: " + snapshot.last_error);
        }
        consume_packets(final_packets);

        output.flush();
        manifest.flush();
        if (!output.good() || !manifest.good()) {
            throw std::runtime_error("failed to finalize standalone AAC output files");
        }

        if (packet_source_frames != input_frames) {
            std::ostringstream message;
            message
                << "source mapping mismatch: input_frames=" << input_frames
                << " packet_source_frames=" << packet_source_frames;
            throw std::runtime_error(message.str());
        }

        const visionarm::AudioEncoderSnapshot snapshot = encoder.snapshot();
        std::cout
            << "input_frames=" << input_frames << '\n'
            << "input_bytes=" << input_bytes << '\n'
            << "encoded_packets=" << packet_index << '\n'
            << "encoded_payload_bytes=" << encoded_payload_bytes << '\n'
            << "mapped_source_frames=" << packet_source_frames << '\n'
            << "encoder_padding_packets=" << snapshot.encoder_padding_packets << '\n'
            << "buffered_input_frames=" << snapshot.buffered_input_frames << '\n'
            << "encoder_drained=" << (snapshot.drained ? 1 : 0) << '\n'
            << "packet_manifest=" << manifest_path << '\n'
            << "audio_encode_probe=PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "audio_encode_probe=FAIL error=" << error.what() << '\n';
        return 1;
    }
}
