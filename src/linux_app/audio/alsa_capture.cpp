#include "alsa_capture.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

void ensureParentDirectory(const std::string& path) {
    const std::filesystem::path file_path(path);
    const auto parent = file_path.parent_path();

    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
}

}  // namespace

AlsaCapture::AlsaCapture(AudioCaptureConfig config)
    : config_(std::move(config)) {}

AlsaCapture::~AlsaCapture() {
    try {
        if (wav_writer_) {
            wav_writer_->close();
        }
    } catch (...) {
    }

    if (raw_output_.is_open()) {
        raw_output_.close();
    }

    if (csv_.is_open()) {
        csv_.close();
    }

    if (event_csv_.is_open()) {
        event_csv_.close();
    }

    if (status_ != nullptr) {
        snd_pcm_status_free(status_);
        status_ = nullptr;
    }

    if (pcm_ != nullptr) {
        snd_pcm_drop(pcm_);
        snd_pcm_close(pcm_);
        pcm_ = nullptr;
    }
}

void AlsaCapture::setStopFlag(
    volatile sig_atomic_t* stop_flag) {
    stop_flag_ = stop_flag;
}

bool AlsaCapture::stopRequested() const {
    return stop_flag_ != nullptr &&
           *stop_flag_ != 0;
}

int64_t AlsaCapture::clockNowNs(clockid_t clock_id) {
    timespec now {};

    if (::clock_gettime(clock_id, &now) != 0) {
        throw std::runtime_error(
            "clock_gettime failed");
    }

    return static_cast<int64_t>(now.tv_sec) *
               1'000'000'000LL +
           static_cast<int64_t>(now.tv_nsec);
}

void AlsaCapture::checkAlsa(
    int result,
    const std::string& operation) {
    if (result < 0) {
        throw std::runtime_error(
            operation + ": " +
            snd_strerror(result) +
            " (" + std::to_string(result) + ")");
    }
}

std::string AlsaCapture::tstampTypeName(
    snd_pcm_tstamp_type_t type) {
    switch (type) {
        case SND_PCM_TSTAMP_TYPE_GETTIMEOFDAY:
            return "GETTIMEOFDAY";

        case SND_PCM_TSTAMP_TYPE_MONOTONIC:
            return "MONOTONIC";

        case SND_PCM_TSTAMP_TYPE_MONOTONIC_RAW:
            return "MONOTONIC_RAW";

        default:
            return "UNKNOWN";
    }
}

void AlsaCapture::openPcm() {
    if (config_.device.rfind("hw:", 0) != 0) {
        throw std::invalid_argument(
            "V2A formal capture requires an hw: ALSA PCM. "
            "Do not use default/plughw for native capability validation.");
    }

    checkAlsa(
        snd_pcm_open(&pcm_,
                     config_.device.c_str(),
                     SND_PCM_STREAM_CAPTURE,
                     0),
        "snd_pcm_open(" + config_.device + ")");

    checkAlsa(
        snd_pcm_status_malloc(&status_),
        "snd_pcm_status_malloc");
}

void AlsaCapture::configureHardware() {
    snd_pcm_hw_params_t* hw_params = nullptr;
    snd_pcm_hw_params_alloca(&hw_params);

    checkAlsa(
        snd_pcm_hw_params_any(pcm_, hw_params),
        "snd_pcm_hw_params_any");

    checkAlsa(
        snd_pcm_hw_params_set_access(
            pcm_,
            hw_params,
            SND_PCM_ACCESS_RW_INTERLEAVED),
        "snd_pcm_hw_params_set_access");

    requested_format_ =
        snd_pcm_format_value(config_.format.c_str());

    if (requested_format_ == SND_PCM_FORMAT_UNKNOWN) {
        throw std::invalid_argument(
            "Unknown ALSA format: " + config_.format);
    }

    checkAlsa(
        snd_pcm_hw_params_set_format(
            pcm_,
            hw_params,
            requested_format_),
        "snd_pcm_hw_params_set_format");

    unsigned int channels = config_.channels;

    checkAlsa(
        snd_pcm_hw_params_set_channels_near(
            pcm_,
            hw_params,
            &channels),
        "snd_pcm_hw_params_set_channels_near");

    unsigned int rate = config_.rate;
    int rate_dir = 0;

    checkAlsa(
        snd_pcm_hw_params_set_rate_near(
            pcm_,
            hw_params,
            &rate,
            &rate_dir),
        "snd_pcm_hw_params_set_rate_near");

    snd_pcm_uframes_t period_frames =
        config_.period_frames;

    int period_dir = 0;

    checkAlsa(
        snd_pcm_hw_params_set_period_size_near(
            pcm_,
            hw_params,
            &period_frames,
            &period_dir),
        "snd_pcm_hw_params_set_period_size_near");

    snd_pcm_uframes_t buffer_frames =
        config_.buffer_frames;

    checkAlsa(
        snd_pcm_hw_params_set_buffer_size_near(
            pcm_,
            hw_params,
            &buffer_frames),
        "snd_pcm_hw_params_set_buffer_size_near");

    checkAlsa(
        snd_pcm_hw_params(pcm_, hw_params),
        "snd_pcm_hw_params");

    checkAlsa(
        snd_pcm_hw_params_get_format(
            hw_params,
            &actual_format_),
        "snd_pcm_hw_params_get_format");

    unsigned int actual_channels = 0;

    checkAlsa(
        snd_pcm_hw_params_get_channels(
            hw_params,
            &actual_channels),
        "snd_pcm_hw_params_get_channels");

    actual_channels_ = actual_channels;

    unsigned int actual_rate = 0;
    rate_dir = 0;

    checkAlsa(
        snd_pcm_hw_params_get_rate(
            hw_params,
            &actual_rate,
            &rate_dir),
        "snd_pcm_hw_params_get_rate");

    actual_rate_ = actual_rate;

    period_dir = 0;

    checkAlsa(
        snd_pcm_hw_params_get_period_size(
            hw_params,
            &actual_period_frames_,
            &period_dir),
        "snd_pcm_hw_params_get_period_size");

    checkAlsa(
        snd_pcm_hw_params_get_buffer_size(
            hw_params,
            &actual_buffer_frames_),
        "snd_pcm_hw_params_get_buffer_size");

    const snd_pcm_sframes_t one_frame_bytes =
        snd_pcm_frames_to_bytes(pcm_, 1);

    if (one_frame_bytes <= 0) {
        throw std::runtime_error(
            "Unable to determine PCM frame size");
    }

    frame_bytes_ =
        static_cast<std::size_t>(one_frame_bytes);

    channel_metrics_.resize(actual_channels_);

    std::cout
        << "alsa_lib_version="
        << snd_asoundlib_version() << '\n'
        << "requested_format="
        << config_.format << '\n'
        << "actual_format="
        << snd_pcm_format_name(actual_format_) << '\n'
        << "requested_rate="
        << config_.rate << '\n'
        << "actual_rate="
        << actual_rate_ << '\n'
        << "requested_channels="
        << config_.channels << '\n'
        << "actual_channels="
        << actual_channels_ << '\n'
        << "requested_period_frames="
        << config_.period_frames << '\n'
        << "actual_period_frames="
        << actual_period_frames_ << '\n'
        << "requested_buffer_frames="
        << config_.buffer_frames << '\n'
        << "actual_buffer_frames="
        << actual_buffer_frames_ << '\n'
        << "frame_bytes="
        << frame_bytes_ << '\n';

    if (actual_rate_ != config_.rate ||
        actual_channels_ != config_.channels ||
        actual_period_frames_ != config_.period_frames ||
        actual_buffer_frames_ != config_.buffer_frames) {
        std::cerr
            << "WARNING: ALSA adjusted one or more "
               "requested hardware parameters\n";
    }
}

void AlsaCapture::configureSoftware() {
    snd_pcm_sw_params_t* sw_params = nullptr;
    snd_pcm_sw_params_alloca(&sw_params);

    checkAlsa(
        snd_pcm_sw_params_current(
            pcm_,
            sw_params),
        "snd_pcm_sw_params_current");

    checkAlsa(
        snd_pcm_sw_params_set_avail_min(
            pcm_,
            sw_params,
            actual_period_frames_),
        "snd_pcm_sw_params_set_avail_min");

    checkAlsa(
        snd_pcm_sw_params_set_tstamp_mode(
            pcm_,
            sw_params,
            SND_PCM_TSTAMP_ENABLE),
        "snd_pcm_sw_params_set_tstamp_mode");

    if (config_.timestamp_type == "monotonic") {
        requested_tstamp_type_ =
            SND_PCM_TSTAMP_TYPE_MONOTONIC;

        selected_clock_id_ =
            CLOCK_MONOTONIC;
    } else if (
        config_.timestamp_type == "monotonic-raw") {
        requested_tstamp_type_ =
            SND_PCM_TSTAMP_TYPE_MONOTONIC_RAW;

        selected_clock_id_ =
            CLOCK_MONOTONIC_RAW;
    } else {
        throw std::invalid_argument(
            "--timestamp-type must be "
            "monotonic or monotonic-raw");
    }

    const int tstamp_result =
        snd_pcm_sw_params_set_tstamp_type(
            pcm_,
            sw_params,
            requested_tstamp_type_);

    if (tstamp_result < 0) {
        throw std::runtime_error(
            "Requested ALSA timestamp type is unsupported: " +
            tstampTypeName(requested_tstamp_type_) +
            ". No silent fallback is allowed. ALSA error: " +
            std::string(snd_strerror(tstamp_result)));
    }

    checkAlsa(
        snd_pcm_sw_params(pcm_, sw_params),
        "snd_pcm_sw_params");

    checkAlsa(
        snd_pcm_sw_params_current(
            pcm_,
            sw_params),
        "snd_pcm_sw_params_current confirmation");

    checkAlsa(
        snd_pcm_sw_params_get_tstamp_type(
            sw_params,
            &actual_tstamp_type_),
        "snd_pcm_sw_params_get_tstamp_type");

    if (actual_tstamp_type_ !=
        requested_tstamp_type_) {
        throw std::runtime_error(
            "ALSA timestamp type differs from request. "
            "No silent fallback is allowed.");
    }

    checkAlsa(
        snd_pcm_prepare(pcm_),
        "snd_pcm_prepare");

    std::cout
        << "requested_timestamp_type="
        << tstampTypeName(requested_tstamp_type_)
        << '\n'
        << "actual_timestamp_type="
        << tstampTypeName(actual_tstamp_type_)
        << '\n';
}

void AlsaCapture::openOutputs() {
    ensureParentDirectory(config_.csv_path);

    csv_.open(config_.csv_path,
              std::ios::out |
              std::ios::trunc);

    if (!csv_) {
        throw std::runtime_error(
            "Cannot open CSV: " + config_.csv_path);
    }

    const std::string event_path =
        config_.csv_path + ".events.csv";

    event_csv_.open(event_path,
                    std::ios::out |
                    std::ios::trunc);

    if (!event_csv_) {
        throw std::runtime_error(
            "Cannot open event CSV: " + event_path);
    }

    csv_
        << "chunk_index,"
        << "frames_requested,"
        << "frames_read,"
        << "cumulative_frames,"
        << "bytes_read,"
        << "app_monotonic_before_ns,"
        << "app_monotonic_after_ns,"
        << "app_selected_clock_before_ns,"
        << "app_selected_clock_after_ns,"
        << "selected_clock_type,"
        << "alsa_timestamp_sec,"
        << "alsa_timestamp_nsec,"
        << "trigger_timestamp_sec,"
        << "trigger_timestamp_nsec,"
        << "avail_frames,"
        << "delay_frames,"
        << "xrun_count,"
        << "recovery_count,"
        << "short_read_count,"
        << "read_error_count,"
        << "chunk_peak_s16,"
        << "chunk_rms_s16\n";

    event_csv_
        << "event_index,"
        << "event_type,"
        << "event_monotonic_ns,"
        << "error_code,"
        << "error_text,"
        << "xrun_count,"
        << "recovery_count,"
        << "cumulative_frames\n";

    if (config_.no_save) {
        return;
    }

    ensureParentDirectory(config_.output_path);

    const std::filesystem::path output_path(
        config_.output_path);

    std::string extension =
        output_path.extension().string();

    std::transform(
        extension.begin(),
        extension.end(),
        extension.begin(),
        [](unsigned char value) {
            return static_cast<char>(
                std::tolower(value));
        });

    if (extension == ".wav") {
        if (actual_format_ !=
            SND_PCM_FORMAT_S16_LE) {
            throw std::runtime_error(
                "The V2A WAV writer currently supports "
                "S16_LE only. Use a .raw output for this "
                "hardware format.");
        }

        wav_writer_ =
            std::make_unique<WavWriter>(
                config_.output_path,
                actual_rate_,
                static_cast<uint16_t>(
                    actual_channels_),
                16);
    } else {
        raw_output_.open(
            config_.output_path,
            std::ios::binary |
            std::ios::trunc);

        if (!raw_output_) {
            throw std::runtime_error(
                "Cannot open raw output: " +
                config_.output_path);
        }
    }
}

void AlsaCapture::writePayload(
    const void* data,
    std::size_t bytes) {
    if (config_.no_save) {
        return;
    }

    if (wav_writer_) {
        wav_writer_->write(data, bytes);
        return;
    }

    raw_output_.write(
        static_cast<const char*>(data),
        static_cast<std::streamsize>(bytes));

    if (!raw_output_) {
        throw std::runtime_error(
            "Failed while writing raw PCM");
    }
}

AlsaCapture::ChunkMetrics
AlsaCapture::updateMetrics(
    const void* data,
    snd_pcm_uframes_t frames) {
    ChunkMetrics result;

    if (actual_format_ !=
        SND_PCM_FORMAT_S16_LE) {
        return result;
    }

    const auto* samples =
        static_cast<const int16_t*>(data);

    const uint64_t sample_count =
        static_cast<uint64_t>(frames) *
        actual_channels_;

    long double chunk_squares = 0.0L;
    int chunk_peak = 0;

    for (uint64_t index = 0;
         index < sample_count;
         ++index) {
        const int value =
            static_cast<int>(samples[index]);

        const int absolute =
            value == -32768
                ? 32768
                : std::abs(value);

        global_metrics_.count++;
        global_metrics_.minimum =
            std::min(global_metrics_.minimum,
                     value);
        global_metrics_.maximum =
            std::max(global_metrics_.maximum,
                     value);
        global_metrics_.peak_abs =
            std::max(global_metrics_.peak_abs,
                     absolute);
        global_metrics_.sum += value;
        global_metrics_.sum_squares +=
            static_cast<long double>(value) *
            static_cast<long double>(value);

        if (value == -32768 ||
            value == 32767) {
            global_metrics_.clipping++;
        }

        const uint32_t channel =
            static_cast<uint32_t>(
                index % actual_channels_);

        auto& channel_metric =
            channel_metrics_[channel];

        channel_metric.count++;
        channel_metric.minimum =
            std::min(channel_metric.minimum,
                     value);
        channel_metric.maximum =
            std::max(channel_metric.maximum,
                     value);
        channel_metric.peak_abs =
            std::max(channel_metric.peak_abs,
                     absolute);
        channel_metric.sum += value;
        channel_metric.sum_squares +=
            static_cast<long double>(value) *
            static_cast<long double>(value);

        if (value == -32768 ||
            value == 32767) {
            channel_metric.clipping++;
        }

        chunk_peak =
            std::max(chunk_peak, absolute);

        chunk_squares +=
            static_cast<long double>(value) *
            static_cast<long double>(value);
    }

    if (sample_count != 0) {
        result.valid = true;
        result.peak =
            static_cast<double>(chunk_peak);

        result.rms =
            std::sqrt(
                static_cast<double>(
                    chunk_squares /
                    static_cast<long double>(
                        sample_count)));
    }

    return result;
}

AlsaCapture::StatusSnapshot
AlsaCapture::readStatus() {
    StatusSnapshot snapshot;

    const int result =
        snd_pcm_status(pcm_, status_);

    if (result < 0) {
        ++stats_.status_error_count;

        writeEvent(
            "STATUS_ERROR",
            result,
            clockNowNs(CLOCK_MONOTONIC));

        return snapshot;
    }

    snapshot.valid = true;

    snd_pcm_status_get_htstamp(
        status_,
        &snapshot.current);

    snd_pcm_status_get_trigger_htstamp(
        status_,
        &snapshot.trigger);

    snapshot.avail =
        snd_pcm_status_get_avail(status_);

    snapshot.delay =
        snd_pcm_status_get_delay(status_);

    return snapshot;
}

void AlsaCapture::writeDataCsv(
    uint64_t chunk_index,
    snd_pcm_uframes_t frames_requested,
    snd_pcm_uframes_t frames_read,
    std::size_t bytes_read,
    int64_t app_mono_before_ns,
    int64_t app_mono_after_ns,
    int64_t selected_before_ns,
    int64_t selected_after_ns,
    const StatusSnapshot& status,
    const ChunkMetrics& metrics) {
    csv_
        << chunk_index << ','
        << frames_requested << ','
        << frames_read << ','
        << stats_.total_frames << ','
        << bytes_read << ','
        << app_mono_before_ns << ','
        << app_mono_after_ns << ','
        << selected_before_ns << ','
        << selected_after_ns << ','
        << tstampTypeName(
               actual_tstamp_type_)
        << ',';

    if (status.valid) {
        csv_
            << status.current.tv_sec << ','
            << status.current.tv_nsec << ','
            << status.trigger.tv_sec << ','
            << status.trigger.tv_nsec << ','
            << status.avail << ','
            << status.delay << ',';
    } else {
        csv_
            << "NA,NA,NA,NA,NA,NA,";
    }

    csv_
        << stats_.xrun_count << ','
        << stats_.recovery_count << ','
        << stats_.short_read_count << ','
        << stats_.read_error_count << ',';

    if (metrics.valid) {
        csv_
            << std::fixed
            << std::setprecision(6)
            << metrics.peak << ','
            << metrics.rms;
    } else {
        csv_ << "NA,NA";
    }

    csv_ << '\n';

    if (!csv_) {
        throw std::runtime_error(
            "Failed while writing audio CSV");
    }
}

void AlsaCapture::writeEvent(
    const std::string& event_type,
    int error_code,
    int64_t event_mono_ns) {
    static uint64_t event_index = 0;

    event_csv_
        << event_index++ << ','
        << event_type << ','
        << event_mono_ns << ','
        << error_code << ','
        << '"'
        << (error_code < 0
                ? snd_strerror(error_code)
                : "none")
        << '"' << ','
        << stats_.xrun_count << ','
        << stats_.recovery_count << ','
        << stats_.total_frames << '\n';

    event_csv_.flush();
}

void AlsaCapture::recoverFrom(
    int error_code,
    const std::string& event_type,
    int64_t event_mono_ns) {
    const int recovery =
        snd_pcm_recover(
            pcm_,
            error_code,
            1);

    if (recovery < 0) {
        writeEvent(
            event_type + "_RECOVERY_FAILED",
            recovery,
            event_mono_ns);

        throw std::runtime_error(
            event_type +
            " recovery failed: " +
            snd_strerror(recovery));
    }

    ++stats_.recovery_count;

    writeEvent(
        event_type,
        error_code,
        event_mono_ns);
}

void AlsaCapture::captureLoop() {
    const std::size_t buffer_bytes =
        static_cast<std::size_t>(
            actual_period_frames_) *
        frame_bytes_;

    std::vector<unsigned char> buffer(
        buffer_bytes);

    stats_.session_start_mono_ns =
        clockNowNs(CLOCK_MONOTONIC);

    int64_t last_stats_ns =
        stats_.session_start_mono_ns;

    while (!stopRequested()) {
        const int64_t current_mono_ns =
            clockNowNs(CLOCK_MONOTONIC);

        const double elapsed =
            static_cast<double>(
                current_mono_ns -
                stats_.session_start_mono_ns) /
            1'000'000'000.0;

        if (elapsed >=
            config_.duration_seconds) {
            break;
        }

        const int64_t app_mono_before_ns =
            clockNowNs(CLOCK_MONOTONIC);

        const int64_t selected_before_ns =
            clockNowNs(selected_clock_id_);

        const snd_pcm_sframes_t read_result =
            snd_pcm_readi(
                pcm_,
                buffer.data(),
                actual_period_frames_);

        const int64_t selected_after_ns =
            clockNowNs(selected_clock_id_);

        const int64_t app_mono_after_ns =
            clockNowNs(CLOCK_MONOTONIC);

        if (read_result < 0) {
            if (read_result == -EINTR) {
                if (stopRequested()) {
                    break;
                }

                continue;
            }

            if (read_result == -EAGAIN) {
                ++stats_.eagain_count;
                continue;
            }

            ++stats_.read_error_count;

            if (read_result == -EPIPE) {
                ++stats_.xrun_count;

                recoverFrom(
                    static_cast<int>(read_result),
                    "XRUN_EPIPE",
                    app_mono_after_ns);

                continue;
            }

            if (read_result == -ESTRPIPE) {
                ++stats_.suspend_count;

                recoverFrom(
                    static_cast<int>(read_result),
                    "SUSPEND_ESTRPIPE",
                    app_mono_after_ns);

                continue;
            }

            writeEvent(
                "READ_ERROR",
                static_cast<int>(read_result),
                app_mono_after_ns);

            throw std::runtime_error(
                "snd_pcm_readi: " +
                std::string(
                    snd_strerror(
                        static_cast<int>(
                            read_result))));
        }

        if (read_result == 0) {
            continue;
        }

        const auto frames_read =
            static_cast<snd_pcm_uframes_t>(
                read_result);

        if (frames_read <
            actual_period_frames_) {
            ++stats_.short_read_count;
        }

        const std::size_t bytes_read =
            static_cast<std::size_t>(
                frames_read) *
            frame_bytes_;

        writePayload(
            buffer.data(),
            bytes_read);

        const ChunkMetrics metrics =
            updateMetrics(
                buffer.data(),
                frames_read);

        stats_.total_frames +=
            frames_read;

        stats_.total_bytes +=
            bytes_read;

        const StatusSnapshot status =
            readStatus();

        writeDataCsv(
            stats_.chunks,
            actual_period_frames_,
            frames_read,
            bytes_read,
            app_mono_before_ns,
            app_mono_after_ns,
            selected_before_ns,
            selected_after_ns,
            status,
            metrics);

        ++stats_.chunks;

        if (config_.stats_interval_seconds > 0.0) {
            const double interval =
                static_cast<double>(
                    app_mono_after_ns -
                    last_stats_ns) /
                1'000'000'000.0;

            if (interval >=
                config_.stats_interval_seconds) {
                const double nominal =
                    static_cast<double>(
                        stats_.total_frames) /
                    static_cast<double>(
                        actual_rate_);

                const double actual_elapsed =
                    static_cast<double>(
                        app_mono_after_ns -
                        stats_.session_start_mono_ns) /
                    1'000'000'000.0;

                std::cout
                    << "progress_elapsed_s="
                    << std::fixed
                    << std::setprecision(3)
                    << actual_elapsed
                    << " total_frames="
                    << stats_.total_frames
                    << " nominal_audio_s="
                    << nominal
                    << " xrun="
                    << stats_.xrun_count
                    << " recovery="
                    << stats_.recovery_count
                    << '\n';

                last_stats_ns =
                    app_mono_after_ns;
            }
        }
    }

    stats_.session_end_mono_ns =
        clockNowNs(CLOCK_MONOTONIC);

    snd_pcm_drop(pcm_);

    if (wav_writer_) {
        wav_writer_->close();
    }

    if (raw_output_.is_open()) {
        raw_output_.flush();
    }

    csv_.flush();
    event_csv_.flush();
}

void AlsaCapture::printSummary() const {
    const double elapsed_seconds =
        static_cast<double>(
            stats_.session_end_mono_ns -
            stats_.session_start_mono_ns) /
        1'000'000'000.0;

    const double nominal_audio_seconds =
        actual_rate_ != 0
            ? static_cast<double>(
                  stats_.total_frames) /
              static_cast<double>(
                  actual_rate_)
            : 0.0;

    const double drift_seconds =
        nominal_audio_seconds -
        elapsed_seconds;

    const double drift_ppm =
        elapsed_seconds > 0.0
            ? drift_seconds /
                  elapsed_seconds *
                  1'000'000.0
            : std::numeric_limits<double>::
                  quiet_NaN();

    std::cout
        << "\n=== V2A CAPTURE SUMMARY ===\n"
        << "actual_format="
        << snd_pcm_format_name(actual_format_)
        << '\n'
        << "actual_rate="
        << actual_rate_ << '\n'
        << "actual_channels="
        << actual_channels_ << '\n'
        << "actual_period_frames="
        << actual_period_frames_ << '\n'
        << "actual_buffer_frames="
        << actual_buffer_frames_ << '\n'
        << "period_duration_ms="
        << std::fixed
        << std::setprecision(6)
        << (1000.0 *
            static_cast<double>(
                actual_period_frames_) /
            static_cast<double>(
                actual_rate_))
        << '\n'
        << "buffer_duration_ms="
        << (1000.0 *
            static_cast<double>(
                actual_buffer_frames_) /
            static_cast<double>(
                actual_rate_))
        << '\n'
        << "timestamp_type="
        << tstampTypeName(
               actual_tstamp_type_)
        << '\n'
        << "chunks="
        << stats_.chunks << '\n'
        << "total_frames="
        << stats_.total_frames << '\n'
        << "total_samples="
        << stats_.total_frames *
               actual_channels_
        << '\n'
        << "total_bytes="
        << stats_.total_bytes << '\n'
        << "xrun_count="
        << stats_.xrun_count << '\n'
        << "recovery_count="
        << stats_.recovery_count << '\n'
        << "suspend_count="
        << stats_.suspend_count << '\n'
        << "short_read_count="
        << stats_.short_read_count << '\n'
        << "read_error_count="
        << stats_.read_error_count << '\n'
        << "status_error_count="
        << stats_.status_error_count << '\n'
        << "nominal_audio_duration_s="
        << nominal_audio_seconds << '\n'
        << "monotonic_elapsed_s="
        << elapsed_seconds << '\n'
        << "drift_s="
        << drift_seconds << '\n'
        << "drift_ppm="
        << drift_ppm << '\n';

    if (global_metrics_.count != 0) {
        const long double mean =
            global_metrics_.sum /
            static_cast<long double>(
                global_metrics_.count);

        const long double rms =
            std::sqrt(
                global_metrics_.sum_squares /
                static_cast<long double>(
                    global_metrics_.count));

        std::cout
            << "sample_min="
            << global_metrics_.minimum << '\n'
            << "sample_max="
            << global_metrics_.maximum << '\n'
            << "peak_abs_s16="
            << global_metrics_.peak_abs << '\n'
            << "rms_s16="
            << static_cast<double>(rms) << '\n'
            << "dc_offset_s16="
            << static_cast<double>(mean) << '\n'
            << "clipping_samples="
            << global_metrics_.clipping << '\n';

        for (std::size_t channel = 0;
             channel <
             channel_metrics_.size();
             ++channel) {
            const auto& metric =
                channel_metrics_[channel];

            if (metric.count == 0) {
                continue;
            }

            const long double channel_rms =
                std::sqrt(
                    metric.sum_squares /
                    static_cast<long double>(
                        metric.count));

            std::cout
                << "channel_"
                << channel
                << "_peak_abs_s16="
                << metric.peak_abs
                << '\n'
                << "channel_"
                << channel
                << "_rms_s16="
                << static_cast<double>(
                       channel_rms)
                << '\n'
                << "channel_"
                << channel
                << "_clipping_samples="
                << metric.clipping
                << '\n';
        }
    } else {
        std::cout
            << "signal_metrics=NA "
               "(implemented for S16_LE)\n";
    }

    if (actual_tstamp_type_ ==
        SND_PCM_TSTAMP_TYPE_MONOTONIC) {
        std::cout
            << "v2_clock_candidate="
               "CLOCK_MONOTONIC\n";
    } else {
        std::cout
            << "v2_clock_candidate="
               "NOT_DIRECTLY_COMPATIBLE_WITH_"
               "V2_DQ_MONOTONIC\n";
    }
}

void AlsaCapture::run() {
    openPcm();
    configureHardware();
    configureSoftware();
    openOutputs();
    captureLoop();
    printSummary();
}
