#include "audio/audio_chunk.h"

#include <cassert>
#include <cstdint>
#include <iostream>

int main() {
    visionarm::RawAudioChunk chunk;
    chunk.format.sample_rate_hz = 48'000U;
    chunk.format.channels = 2U;
    chunk.format.sample_format = visionarm::AudioSampleFormat::kS16LE;
    chunk.timing.first_sample_frame_index = 96'000U;
    chunk.timing.frame_count = 1'024U;
    chunk.pcm.resize(1'024U * 2U * 2U);

    assert(chunk.format.BytesPerSample() == 2U);
    assert(chunk.format.BytesPerFrame() == 4U);
    assert(chunk.ExpectedBytes() == 4'096U);
    assert(chunk.Valid());
    assert(chunk.NominalDurationNs() == 21'333'333LL);

    chunk.pcm.pop_back();
    assert(!chunk.Valid());

    std::cout << "audio_chunk_contract_test: PASS\n";
    return 0;
}
