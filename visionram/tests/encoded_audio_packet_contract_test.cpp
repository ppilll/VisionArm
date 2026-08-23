#include "audio/encoded_audio_packet.h"

#include <cassert>
#include <cstdint>
#include <iostream>

int main() {
    visionarm::AudioEncoderStreamInfo info;
    info.sample_rate_hz = 48'000U;
    info.channels = 2U;
    info.bit_rate_bps = 128'000;
    info.frame_size_frames = 1'024U;
    info.initial_padding_frames = 1'024U;
    info.codec_config = {0x11U, 0x90U};
    assert(info.Valid());

    // AAC priming is a valid encoded packet even though no real source PCM
    // sample overlaps it.
    visionarm::EncodedAudioPacket priming;
    priming.data = {0x01U, 0x02U, 0x03U};
    priming.pts_ns = -21'333'333LL;
    priming.dts_ns = priming.pts_ns;
    priming.duration_ns = 21'333'333LL;
    priming.contains_encoder_padding = true;
    assert(priming.Valid());

    visionarm::EncodedAudioPacket audio;
    audio.data = {0x10U, 0x20U};
    audio.pts_ns = 0;
    audio.dts_ns = 0;
    audio.duration_ns = 21'333'333LL;
    audio.has_source_samples = true;
    audio.source_pts_ns = 0;
    audio.source_first_sample_frame_index = 0U;
    audio.source_frame_count = 1'024U;
    assert(audio.Valid());

    audio.source_frame_count = 0U;
    assert(!audio.Valid());

    std::cout << "encoded_audio_packet_contract_test: PASS\n";
    return 0;
}
