#pragma once

#include <atomic>
#include <cstdint>

namespace visionarm {

struct CaptureBufferBrokerSnapshot;
struct H265FileSinkSnapshot;
struct PipelineStatsSnapshot;
struct RgaPreprocessorSnapshot;
struct TargetStateMachineSnapshot;
struct VideoEncoderSnapshot;
struct TimedAudioChunk;
template <typename T>
class BoundedQueue;

}  // namespace visionarm

namespace visionarm::runtime::detail {

extern std::atomic<bool> g_stop;
extern std::atomic<bool> g_reload_log_config;

[[nodiscard]] int RunRuntime(int argc, char** argv);

[[nodiscard]] std::int64_t ReadVmRssKb() noexcept;

struct RssSamples {
    std::int64_t first_kb = -1;
    std::int64_t last_kb = -1;
    std::int64_t minimum_kb = -1;
    std::int64_t maximum_kb = -1;
    std::uint64_t samples = 0U;

    void Add(std::int64_t value) noexcept;
    [[nodiscard]] std::int64_t GrowthKb() const noexcept;
};

#if defined(VISIONARM_HAS_ALSA_AUDIO)
struct AudioTimelineStats {
    std::uint64_t chunks = 0U;
    std::uint64_t frames = 0U;
    std::uint64_t bytes = 0U;
    std::uint64_t reanchors = 0U;
    std::uint64_t discontinuities = 0U;
    std::uint64_t pts_regressions = 0U;
    std::uint64_t continuous_pts_mismatches = 0U;
    std::int64_t first_pts_ns = -1;
    std::int64_t last_pts_ns = -1;
    std::int64_t last_end_pts_ns = -1;
    std::int64_t maximum_forward_gap_ns = 0;
    std::int64_t maximum_abs_timing_error_ns = 0;

    void Consume(const TimedAudioChunk& chunk) noexcept;
};

void DrainAudioQueue(
    BoundedQueue<TimedAudioChunk>* queue,
    AudioTimelineStats* stats);
#endif

[[nodiscard]] double RatePerSecond(
    std::uint64_t count, double seconds) noexcept;

[[nodiscard]] bool EvaluateRuntimePass(
    bool completed_requested_duration,
    bool terminated_by_signal,
    const PipelineStatsSnapshot& stats,
    const CaptureBufferBrokerSnapshot& broker_stats,
    const VideoEncoderSnapshot& encoder_stats,
    const H265FileSinkSnapshot& sink_stats,
    const RgaPreprocessorSnapshot& rga_stats,
    const TargetStateMachineSnapshot& state_stats,
    bool control_ok,
    bool auxiliary_media_runtime_fault,
    bool audio_ok,
    bool audio_encode_ok,
    bool local_av_mux_ok,
    bool network_mux_ok,
    bool telemetry_ok,
    bool queues_ok,
    bool throughput_ok,
    bool timing_ok,
    bool rss_ok) noexcept;

}  // namespace visionarm::runtime::detail
