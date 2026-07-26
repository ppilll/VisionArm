#pragma once

#include "wav_writer.h"

#include <alsa/asoundlib.h>

#include <csignal>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

struct AudioCaptureConfig {
    std::string device;
    uint32_t rate = 0;
    uint32_t channels = 0;
    std::string format;

    snd_pcm_uframes_t period_frames = 0;
    snd_pcm_uframes_t buffer_frames = 0;

    double duration_seconds = 60.0;

    std::string output_path =
        "reports/audio/capture.wav";

    std::string csv_path =
        "logs/v2a_audio/audio_frames.csv";

    bool no_save = false;

    std::string timestamp_type = "monotonic";

    double stats_interval_seconds = 5.0;
};

class AlsaCapture {
public:
    explicit AlsaCapture(AudioCaptureConfig config);
    ~AlsaCapture();

    AlsaCapture(const AlsaCapture&) = delete;
    AlsaCapture& operator=(const AlsaCapture&) = delete;

    void setStopFlag(volatile sig_atomic_t* stop_flag);
    void run();

private:
    struct Statistics {
        uint64_t chunks = 0;
        uint64_t total_frames = 0;
        uint64_t total_bytes = 0;

        uint64_t xrun_count = 0;
        uint64_t recovery_count = 0;
        uint64_t suspend_count = 0;
        uint64_t short_read_count = 0;
        uint64_t read_error_count = 0;
        uint64_t eagain_count = 0;
        uint64_t status_error_count = 0;

        int64_t session_start_mono_ns = 0;
        int64_t session_end_mono_ns = 0;
    };

    struct MetricAccumulator {
        uint64_t count = 0;
        int minimum = 32767;
        int maximum = -32768;
        int peak_abs = 0;
        uint64_t clipping = 0;
        long double sum = 0.0L;
        long double sum_squares = 0.0L;
    };

    struct ChunkMetrics {
        bool valid = false;
        double peak = 0.0;
        double rms = 0.0;
    };

    struct StatusSnapshot {
        bool valid = false;
        snd_htimestamp_t current {};
        snd_htimestamp_t trigger {};
        snd_pcm_uframes_t avail = 0;
        snd_pcm_sframes_t delay = 0;
    };

    AudioCaptureConfig config_;

    snd_pcm_t* pcm_ = nullptr;
    snd_pcm_status_t* status_ = nullptr;

    snd_pcm_format_t requested_format_ =
        SND_PCM_FORMAT_UNKNOWN;

    snd_pcm_format_t actual_format_ =
        SND_PCM_FORMAT_UNKNOWN;

    uint32_t actual_rate_ = 0;
    uint32_t actual_channels_ = 0;

    snd_pcm_uframes_t actual_period_frames_ = 0;
    snd_pcm_uframes_t actual_buffer_frames_ = 0;

    std::size_t frame_bytes_ = 0;

    snd_pcm_tstamp_type_t requested_tstamp_type_ =
        SND_PCM_TSTAMP_TYPE_MONOTONIC;

    snd_pcm_tstamp_type_t actual_tstamp_type_ =
        SND_PCM_TSTAMP_TYPE_MONOTONIC;

    clockid_t selected_clock_id_ = CLOCK_MONOTONIC;

    std::ofstream csv_;
    std::ofstream event_csv_;
    std::ofstream raw_output_;

    std::unique_ptr<WavWriter> wav_writer_;

    Statistics stats_;
    MetricAccumulator global_metrics_;
    std::vector<MetricAccumulator> channel_metrics_;

    volatile sig_atomic_t* stop_flag_ = nullptr;

    void openPcm();
    void configureHardware();
    void configureSoftware();
    void openOutputs();
    void captureLoop();

    void writePayload(const void* data,
                      std::size_t bytes);

    ChunkMetrics updateMetrics(
        const void* data,
        snd_pcm_uframes_t frames);

    StatusSnapshot readStatus();

    void writeDataCsv(
        uint64_t chunk_index,
        snd_pcm_uframes_t frames_requested,
        snd_pcm_uframes_t frames_read,
        std::size_t bytes_read,
        int64_t app_mono_before_ns,
        int64_t app_mono_after_ns,
        int64_t selected_before_ns,
        int64_t selected_after_ns,
        const StatusSnapshot& status,
        const ChunkMetrics& metrics);

    void writeEvent(const std::string& event_type,
                    int error_code,
                    int64_t event_mono_ns);

    void recoverFrom(int error_code,
                     const std::string& event_type,
                     int64_t event_mono_ns);

    void printSummary() const;

    bool stopRequested() const;

    static int64_t clockNowNs(clockid_t clock_id);

    static void checkAlsa(int result,
                          const std::string& operation);

    static std::string tstampTypeName(
        snd_pcm_tstamp_type_t type);
};
