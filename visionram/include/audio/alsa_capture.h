#pragma once

#include "audio/audio_chunk.h"

#include <cstdint>
#include <memory>
#include <string>

namespace visionarm {

struct AlsaCaptureConfig {
    std::string device = "hw:1,0";
    std::uint32_t sample_rate_hz = 48'000U;
    std::uint16_t channels = 2U;
    AudioSampleFormat sample_format = AudioSampleFormat::kS16LE;
    std::uint32_t period_frames = 1'024U;
    std::uint32_t buffer_frames = 4'096U;

    // V8.3 product path should not silently run with a different rate/channel/
    // period/buffer than the media pipeline was configured for.
    bool require_exact_hw_params = true;
};

struct AlsaCaptureInfo {
    AudioStreamFormat format;
    std::uint32_t period_frames = 0U;
    std::uint32_t buffer_frames = 0U;
    std::string device;
    std::string timestamp_type;
};

struct AlsaCaptureSnapshot {
    std::uint64_t chunks = 0U;
    std::uint64_t delivered_frames = 0U;
    std::uint64_t delivered_bytes = 0U;
    std::uint64_t xrun_count = 0U;
    std::uint64_t suspend_count = 0U;
    std::uint64_t recovery_count = 0U;
    std::uint64_t short_read_count = 0U;
    std::uint64_t eagain_count = 0U;
    std::uint64_t status_error_count = 0U;
};

// Synchronous ALSA PCM producer extracted from the frozen V2A test program.
//
// Threading contract:
// - Open(), Read() and Close() are intended to be called by the owning audio
//   capture thread.
// - Snapshot() may be called from another thread.
// - Read() owns no Camera/DMA-BUF resources. RawAudioChunk owns its PCM bytes.
//
// Timestamp contract:
// - CLOCK_MONOTONIC is mandatory for V8.3.
// - Read() does not assign media PTS. It exports sample-frame indices and ALSA
//   timing evidence for the later MediaClock stage.
class AlsaCapture final {
public:
    explicit AlsaCapture(AlsaCaptureConfig config);
    ~AlsaCapture();

    AlsaCapture(const AlsaCapture&) = delete;
    AlsaCapture& operator=(const AlsaCapture&) = delete;
    AlsaCapture(AlsaCapture&&) = delete;
    AlsaCapture& operator=(AlsaCapture&&) = delete;

    void Open();
    void Close() noexcept;

    [[nodiscard]] bool IsOpen() const noexcept;
    [[nodiscard]] const AlsaCaptureInfo& Info() const;

    // Blocks until one PCM chunk is available. XRUN/suspend recovery is
    // performed internally. The first chunk after recovery is marked with
    // timing.discontinuity_before=true.
    //
    // Returns false only if the blocking ALSA read was interrupted by EINTR;
    // the caller can check its own shutdown flag and call Read() again if the
    // interruption was unrelated to shutdown.
    bool Read(RawAudioChunk* chunk);

    [[nodiscard]] AlsaCaptureSnapshot Snapshot() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace visionarm
