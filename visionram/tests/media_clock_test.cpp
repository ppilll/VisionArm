#include "media/media_clock.h"

#include <cassert>
#include <cstdint>
#include <iostream>

namespace {

visionarm::RawAudioChunk MakeChunk(
    std::uint64_t first_index,
    std::uint32_t frames,
    std::int64_t status_ns,
    std::int64_t delay_frames,
    bool discontinuity = false) {
    visionarm::RawAudioChunk chunk;
    chunk.format.sample_rate_hz = 48'000U;
    chunk.format.channels = 2U;
    chunk.format.sample_format = visionarm::AudioSampleFormat::kS16LE;
    chunk.timing.first_sample_frame_index = first_index;
    chunk.timing.frame_count = frames;
    chunk.timing.alsa_status_valid = true;
    chunk.timing.alsa_status_mono_ns = status_ns;
    chunk.timing.delay_frames = delay_frames;
    chunk.timing.discontinuity_before = discontinuity;
    chunk.pcm.resize(
        static_cast<std::size_t>(frames) * chunk.format.BytesPerFrame());
    return chunk;
}

}  // namespace

int main() {
    constexpr std::int64_t kEpochNs = 1'000'000'000LL;
    visionarm::MediaClock clock(kEpochNs);

    assert(clock.VideoPtsNs(1'250'000'000LL) == 250'000'000LL);

    // status=2.000s, after reading 480 frames, 480 more frames remain queued.
    // First sample of the returned chunk is therefore 20ms before status.
    auto first = clock.StampAudio(
        MakeChunk(0U, 480U, 2'000'000'000LL, 480));
    assert(first.media.reanchored);
    assert(first.media.first_sample_mono_ns == 1'980'000'000LL);
    assert(first.media.pts_ns == 980'000'000LL);
    assert(first.media.duration_ns == 10'000'000LL);
    assert(first.media.timing_error_ns == 0);

    // Normal capture advances strictly by sample count. The independent ALSA
    // observation agrees exactly here.
    auto second = clock.StampAudio(
        MakeChunk(480U, 480U, 2'010'000'000LL, 480));
    assert(!second.media.reanchored);
    assert(second.media.pts_ns == 990'000'000LL);
    assert(second.media.duration_ns == 10'000'000LL);
    assert(second.media.timing_error_ns == 0);

    // A 1024-frame interval at 48kHz has a fractional-ns duration. PTS is
    // generated from cumulative sample index so rounding does not accumulate.
    visionarm::MediaClock fractional_clock(kEpochNs);
    auto fractional0 = fractional_clock.StampAudio(
        MakeChunk(0U, 1024U, 2'042'666'666LL, 1024));
    auto fractional1 = fractional_clock.StampAudio(
        MakeChunk(1024U, 1024U, 2'063'999'999LL, 1024));
    assert(fractional1.media.pts_ns ==
           fractional0.media.pts_ns + fractional0.media.duration_ns);

    // XRUN/suspend discontinuity re-anchors to real monotonic time and leaves
    // a forward PTS gap rather than compressing missing physical audio.
    auto after_gap = clock.StampAudio(
        MakeChunk(960U, 480U, 2'500'000'000LL, 480, true));
    assert(after_gap.media.reanchored);
    assert(after_gap.media.pts_ns == 1'480'000'000LL);
    assert(after_gap.media.pts_ns > second.media.pts_ns + second.media.duration_ns);

    const auto snapshot = clock.Snapshot();
    assert(snapshot.audio_chunks_stamped == 3U);
    assert(snapshot.audio_reanchors == 2U);
    assert(snapshot.audio_discontinuities == 1U);
    assert(snapshot.audio_timestamp_failures == 0U);

    std::cout << "media_clock_test: PASS\n";
    return 0;
}
