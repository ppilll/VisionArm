#include "v4l2_capture.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>

namespace {

std::runtime_error systemError(const std::string& operation) {
    return std::runtime_error(
        operation + ": " + std::strerror(errno) +
        " (errno=" + std::to_string(errno) + ")");
}

double mean(const std::vector<double>& values) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double sum =
        std::accumulate(values.begin(), values.end(), 0.0);
    return sum / static_cast<double>(values.size());
}

double populationStdDev(const std::vector<double>& values) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double average = mean(values);
    double squared_sum = 0.0;

    for (const double value : values) {
        const double difference = value - average;
        squared_sum += difference * difference;
    }

    return std::sqrt(squared_sum /
                     static_cast<double>(values.size()));
}

void printDoubleOrNA(const char* key, double value) {
    std::cout << key << "=";
    if (std::isnan(value)) {
        std::cout << "NA\n";
    } else {
        std::cout << std::fixed << std::setprecision(6)
                  << value << '\n';
    }
}

}  // namespace

V4L2Capture::V4L2Capture(CaptureConfig config)
    : config_(std::move(config)) {}

V4L2Capture::~V4L2Capture() {
    releaseResources();
}

void V4L2Capture::setStopFlag(volatile sig_atomic_t* stop_flag) {
    stop_flag_ = stop_flag;
}

bool V4L2Capture::stopRequested() const {
    return stop_flag_ != nullptr && *stop_flag_ != 0;
}

int V4L2Capture::xioctl(int fd,
                        unsigned long request,
                        void* argument) {
    int result;

    do {
        result = ::ioctl(fd, request, argument);
    } while (result == -1 && errno == EINTR);

    return result;
}

uint32_t V4L2Capture::fourccFromString(
    const std::string& text) {
    if (text.size() != 4) {
        throw std::invalid_argument(
            "FOURCC must contain exactly four characters");
    }

    return v4l2_fourcc(text[0], text[1], text[2], text[3]);
}

std::string V4L2Capture::fourccToString(uint32_t value) {
    std::string result(4, '\0');
    result[0] = static_cast<char>(value & 0xffU);
    result[1] = static_cast<char>((value >> 8U) & 0xffU);
    result[2] = static_cast<char>((value >> 16U) & 0xffU);
    result[3] = static_cast<char>((value >> 24U) & 0xffU);

    for (char& character : result) {
        if (character < 32 || character > 126) {
            character = '?';
        }
    }

    return result;
}

std::string V4L2Capture::timestampType(uint32_t flags) {
    const uint32_t type = flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;

    switch (type) {
        case V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
            return "MONOTONIC";
        case V4L2_BUF_FLAG_TIMESTAMP_COPY:
            return "COPY";
        case V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

std::string V4L2Capture::timestampSource(uint32_t flags) {
    const uint32_t source =
        flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

    switch (source) {
        case V4L2_BUF_FLAG_TSTAMP_SRC_SOE:
            return "SOE";
        case V4L2_BUF_FLAG_TSTAMP_SRC_EOF:
        default:
            return "EOF_OR_UNKNOWN";
    }
}

int64_t V4L2Capture::timevalToNs(const timeval& value) {
    return static_cast<int64_t>(value.tv_sec) *
               1'000'000'000LL +
           static_cast<int64_t>(value.tv_usec) * 1'000LL;
}

int64_t V4L2Capture::monotonicNowNs() {
    timespec now{};

    if (::clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        throw systemError("clock_gettime(CLOCK_MONOTONIC)");
    }

    return static_cast<int64_t>(now.tv_sec) *
               1'000'000'000LL +
           static_cast<int64_t>(now.tv_nsec);
}

std::string V4L2Capture::frameExtension(uint32_t fourcc) {
    if (fourcc == V4L2_PIX_FMT_MJPEG ||
        fourcc == V4L2_PIX_FMT_JPEG) {
        return ".jpg";
    }

    if (fourcc == V4L2_PIX_FMT_YUYV) {
        return ".yuyv";
    }

    if (fourcc == V4L2_PIX_FMT_UYVY) {
        return ".uyvy";
    }

    if (fourcc == V4L2_PIX_FMT_NV12) {
        return ".nv12";
    }

    if (fourcc == V4L2_PIX_FMT_NV21) {
        return ".nv21";
    }

#ifdef V4L2_PIX_FMT_NV12M
    if (fourcc == V4L2_PIX_FMT_NV12M) {
        return ".nv12m";
    }
#endif

#ifdef V4L2_PIX_FMT_NV21M
    if (fourcc == V4L2_PIX_FMT_NV21M) {
        return ".nv21m";
    }
#endif

    return ".raw";
}

void V4L2Capture::openDevice() {
    struct stat status {};

    if (::stat(config_.device.c_str(), &status) != 0) {
        throw systemError("stat(" + config_.device + ")");
    }

    if (!S_ISCHR(status.st_mode)) {
        throw std::runtime_error(
            config_.device + " is not a character device");
    }

    int flags = O_RDWR | O_CLOEXEC;
    if (config_.nonblock) {
        flags |= O_NONBLOCK;
    }

    fd_ = ::open(config_.device.c_str(), flags);
    if (fd_ < 0) {
        throw systemError("open(" + config_.device + ")");
    }
}

void V4L2Capture::queryCapabilities() {
    v4l2_capability capability {};

    if (xioctl(fd_, VIDIOC_QUERYCAP, &capability) != 0) {
        throw systemError("VIDIOC_QUERYCAP");
    }

    const uint32_t device_caps =
        (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
            ? capability.device_caps
            : capability.capabilities;

    const bool supports_single =
        (device_caps & V4L2_CAP_VIDEO_CAPTURE) != 0U;
    const bool supports_multi =
        (device_caps &
         V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0U;
    const bool supports_streaming =
        (device_caps & V4L2_CAP_STREAMING) != 0U;

    std::cout << "driver="
              << reinterpret_cast<const char*>(capability.driver)
              << '\n';
    std::cout << "card="
              << reinterpret_cast<const char*>(capability.card)
              << '\n';
    std::cout << "bus_info="
              << reinterpret_cast<const char*>(capability.bus_info)
              << '\n';
    std::cout << "capabilities=0x"
              << std::hex << capability.capabilities << '\n';
    std::cout << "device_caps=0x"
              << device_caps << std::dec << '\n';

    if (!supports_single && !supports_multi) {
        throw std::runtime_error(
            "Selected node has neither VIDEO_CAPTURE nor "
            "VIDEO_CAPTURE_MPLANE capability");
    }

    if (!supports_streaming) {
        throw std::runtime_error(
            "Selected node does not report V4L2_CAP_STREAMING");
    }

    /*
     * Prefer the single-planar API only when the node advertises it.
     * Otherwise use the mandatory multi-planar API.
     */
    if (supports_single) {
        multiplanar_ = false;
        buffer_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    } else {
        multiplanar_ = true;
        buffer_type_ =
            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    }

    std::cout << "capture_api="
              << (multiplanar_ ? "MULTI_PLANAR"
                               : "SINGLE_PLANAR")
              << '\n';
}

void V4L2Capture::configureFormat() {
    const uint32_t requested_fourcc =
        fourccFromString(config_.fourcc);

    v4l2_format current {};
    current.type = buffer_type_;

    if (xioctl(fd_, VIDIOC_G_FMT, &current) != 0) {
        throw systemError("VIDIOC_G_FMT before S_FMT");
    }

    v4l2_format requested {};
    requested.type = buffer_type_;

    if (multiplanar_) {
        requested.fmt.pix_mp.width = config_.width;
        requested.fmt.pix_mp.height = config_.height;
        requested.fmt.pix_mp.pixelformat = requested_fourcc;
        requested.fmt.pix_mp.field = V4L2_FIELD_ANY;
    } else {
        requested.fmt.pix.width = config_.width;
        requested.fmt.pix.height = config_.height;
        requested.fmt.pix.pixelformat = requested_fourcc;
        requested.fmt.pix.field = V4L2_FIELD_ANY;
    }

    if (xioctl(fd_, VIDIOC_S_FMT, &requested) != 0) {
        throw systemError("VIDIOC_S_FMT");
    }

    v4l2_format accepted {};
    accepted.type = buffer_type_;

    if (xioctl(fd_, VIDIOC_G_FMT, &accepted) != 0) {
        throw systemError("VIDIOC_G_FMT after S_FMT");
    }

    bytes_per_line_.clear();
    size_image_.clear();

    if (multiplanar_) {
        actual_width_ = accepted.fmt.pix_mp.width;
        actual_height_ = accepted.fmt.pix_mp.height;
        actual_fourcc_ = accepted.fmt.pix_mp.pixelformat;
        plane_count_ = accepted.fmt.pix_mp.num_planes;

        if (plane_count_ == 0 ||
            plane_count_ > VIDEO_MAX_PLANES) {
            throw std::runtime_error(
                "Driver returned an invalid plane count");
        }

        for (uint32_t plane = 0;
             plane < plane_count_;
             ++plane) {
            bytes_per_line_.push_back(
                accepted.fmt.pix_mp.plane_fmt[plane]
                    .bytesperline);
            size_image_.push_back(
                accepted.fmt.pix_mp.plane_fmt[plane]
                    .sizeimage);
        }
    } else {
        actual_width_ = accepted.fmt.pix.width;
        actual_height_ = accepted.fmt.pix.height;
        actual_fourcc_ = accepted.fmt.pix.pixelformat;
        plane_count_ = 1;
        bytes_per_line_.push_back(
            accepted.fmt.pix.bytesperline);
        size_image_.push_back(
            accepted.fmt.pix.sizeimage);
    }

    std::cout << "actual_width=" << actual_width_ << '\n';
    std::cout << "actual_height=" << actual_height_ << '\n';
    std::cout << "actual_format="
              << fourccToString(actual_fourcc_) << '\n';
    std::cout << "plane_count=" << plane_count_ << '\n';

    for (uint32_t plane = 0;
         plane < plane_count_;
         ++plane) {
        std::cout << "plane_" << plane
                  << "_bytesperline="
                  << bytes_per_line_[plane] << '\n';
        std::cout << "plane_" << plane
                  << "_sizeimage="
                  << size_image_[plane] << '\n';
    }

    if (actual_width_ != config_.width ||
        actual_height_ != config_.height ||
        actual_fourcc_ != requested_fourcc) {
        std::cerr
            << "WARNING: driver adjusted the requested format\n";
    }
}

void V4L2Capture::configureFrameRate() {
    v4l2_streamparm parameters {};
    parameters.type = buffer_type_;

    if (xioctl(fd_, VIDIOC_G_PARM, &parameters) != 0) {
        std::cerr
            << "WARNING: VIDIOC_G_PARM before S_PARM failed: "
            << std::strerror(errno) << '\n';
    }

    parameters = {};
    parameters.type = buffer_type_;
    parameters.parm.capture.timeperframe.numerator = 1;
    parameters.parm.capture.timeperframe.denominator =
        config_.fps;

    if (xioctl(fd_, VIDIOC_S_PARM, &parameters) != 0) {
        std::cerr
            << "WARNING: VIDIOC_S_PARM failed or is unsupported: "
            << std::strerror(errno) << '\n';
    }

    parameters = {};
    parameters.type = buffer_type_;

    if (xioctl(fd_, VIDIOC_G_PARM, &parameters) != 0) {
        std::cerr
            << "WARNING: VIDIOC_G_PARM confirmation failed: "
            << std::strerror(errno) << '\n';
        actual_fps_known_ = false;
        return;
    }

    const auto numerator =
        parameters.parm.capture.timeperframe.numerator;
    const auto denominator =
        parameters.parm.capture.timeperframe.denominator;

    if (numerator != 0U && denominator != 0U) {
        actual_fps_parameter_ =
            static_cast<double>(denominator) /
            static_cast<double>(numerator);
        actual_fps_known_ = true;
    }
}

void V4L2Capture::initializeMmap() {
    v4l2_requestbuffers request {};
    request.count = config_.buffer_count;
    request.type = buffer_type_;
    request.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_REQBUFS, &request) != 0) {
        throw systemError("VIDIOC_REQBUFS");
    }

    if (request.count < 2) {
        throw std::runtime_error(
            "Driver allocated fewer than two mmap buffers");
    }

    buffers_.resize(request.count);

    for (uint32_t index = 0;
         index < request.count;
         ++index) {
        v4l2_buffer buffer {};
        v4l2_plane planes[VIDEO_MAX_PLANES] {};

        buffer.type = buffer_type_;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;

        if (multiplanar_) {
            buffer.m.planes = planes;
            buffer.length = VIDEO_MAX_PLANES;
        }

        if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) != 0) {
            throw systemError("VIDIOC_QUERYBUF");
        }

        const uint32_t mapped_plane_count =
            multiplanar_ ? buffer.length : 1U;

        if (mapped_plane_count < plane_count_) {
            throw std::runtime_error(
                "VIDIOC_QUERYBUF returned fewer planes "
                "than the negotiated format");
        }

        buffers_[index].planes.resize(plane_count_);

        for (uint32_t plane = 0;
             plane < plane_count_;
             ++plane) {
            std::size_t length = 0;
            off_t offset = 0;

            if (multiplanar_) {
                length = planes[plane].length;
                offset = static_cast<off_t>(
                    planes[plane].m.mem_offset);
            } else {
                length = buffer.length;
                offset =
                    static_cast<off_t>(buffer.m.offset);
            }

            void* address =
                ::mmap(nullptr,
                       length,
                       PROT_READ | PROT_WRITE,
                       MAP_SHARED,
                       fd_,
                       offset);

            if (address == MAP_FAILED) {
                throw systemError("mmap");
            }

            buffers_[index].planes[plane].start =
                address;
            buffers_[index].planes[plane].length =
                length;
        }
    }
}

void V4L2Capture::queueAllBuffers() {
    for (uint32_t index = 0;
         index < buffers_.size();
         ++index) {
        v4l2_buffer buffer {};
        v4l2_plane planes[VIDEO_MAX_PLANES] {};

        buffer.type = buffer_type_;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;

        if (multiplanar_) {
            buffer.m.planes = planes;
            buffer.length = plane_count_;
        }

        if (xioctl(fd_, VIDIOC_QBUF, &buffer) != 0) {
            throw systemError("initial VIDIOC_QBUF");
        }
    }
}

void V4L2Capture::startStreaming() {
    queueAllBuffers();

    v4l2_buf_type type = buffer_type_;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) != 0) {
        throw systemError("VIDIOC_STREAMON");
    }

    streaming_ = true;
}

void V4L2Capture::saveFrame(
    uint64_t frame_index,
    const v4l2_buffer& buffer,
    const v4l2_plane* planes) {
    std::filesystem::create_directories(config_.output_dir);

    const auto filename =
        std::filesystem::path(config_.output_dir) /
        ("frame_" +
         [&]() {
             std::ostringstream text;
             text << std::setw(6) << std::setfill('0')
                  << frame_index;
             return text.str();
         }() +
         frameExtension(actual_fourcc_));

    std::ofstream output(filename,
                         std::ios::binary |
                         std::ios::trunc);

    if (!output) {
        throw std::runtime_error(
            "Cannot open frame output: " +
            filename.string());
    }

    if (!multiplanar_) {
        const std::size_t payload =
            std::min<std::size_t>(
                buffer.bytesused,
                buffers_[buffer.index].planes[0].length);

        output.write(
            static_cast<const char*>(
                buffers_[buffer.index].planes[0].start),
            static_cast<std::streamsize>(payload));
    } else {
        for (uint32_t plane = 0;
             plane < plane_count_;
             ++plane) {
            const std::size_t mapping_length =
                buffers_[buffer.index].planes[plane]
                    .length;

            const std::size_t data_offset =
                std::min<std::size_t>(
                    planes[plane].data_offset,
                    mapping_length);

            const std::size_t bytes_used =
                std::min<std::size_t>(
                    planes[plane].bytesused,
                    mapping_length);

            const std::size_t payload =
                bytes_used > data_offset
                    ? bytes_used - data_offset
                    : 0;

            const auto* start =
                static_cast<const char*>(
                    buffers_[buffer.index]
                        .planes[plane]
                        .start) +
                data_offset;

            output.write(
                start,
                static_cast<std::streamsize>(payload));
        }
    }

    if (!output.good()) {
        throw std::runtime_error(
            "Failed while writing frame: " +
            filename.string());
    }

    std::cout << "saved_frame=" << filename << '\n';
}

void V4L2Capture::captureLoop() {
    const auto csv_parent =
        std::filesystem::path(config_.csv_path)
            .parent_path();

    if (!csv_parent.empty()) {
        std::filesystem::create_directories(csv_parent);
    }

    std::ofstream csv(config_.csv_path,
                      std::ios::out |
                      std::ios::trunc);

    if (!csv) {
        throw std::runtime_error(
            "Cannot open CSV file: " + config_.csv_path);
    }

    csv << "frame_index,"
        << "v4l2_sequence,"
        << "bytes_used,"
        << "v4l2_timestamp_sec,"
        << "v4l2_timestamp_usec,"
        << "timestamp_type,"
        << "timestamp_source,"
        << "dq_monotonic_ns,"
        << "sequence_gap,"
        << "poll_wait_us\n";

    bool have_previous_sequence = false;
    uint32_t previous_sequence = 0;

    bool have_previous_timestamp = false;
    int64_t previous_timestamp_ns = 0;

    uint32_t consecutive_timeouts = 0;
    constexpr uint32_t maximum_consecutive_timeouts = 10;

    const int64_t session_start_ns = monotonicNowNs();

    while (stats_.captured_frames <
               config_.frame_count &&
           !stopRequested()) {
        pollfd descriptor {};
        descriptor.fd = fd_;
        descriptor.events = POLLIN | POLLPRI;

        const int64_t poll_start_ns = monotonicNowNs();
        const int poll_result =
            ::poll(&descriptor, 1, config_.timeout_ms);
        const int64_t poll_end_ns = monotonicNowNs();

        const int64_t poll_wait_us =
            (poll_end_ns - poll_start_ns) / 1'000LL;

        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw systemError("poll");
        }

        if (poll_result == 0) {
            ++stats_.poll_timeouts;
            ++consecutive_timeouts;

            std::cerr
                << "WARNING: poll timeout "
                << consecutive_timeouts << "/"
                << maximum_consecutive_timeouts
                << '\n';

            if (consecutive_timeouts >=
                maximum_consecutive_timeouts) {
                throw std::runtime_error(
                    "Too many consecutive poll timeouts");
            }
            continue;
        }

        consecutive_timeouts = 0;

        if ((descriptor.revents &
             (POLLERR | POLLHUP | POLLNVAL)) != 0 &&
            (descriptor.revents &
             (POLLIN | POLLPRI)) == 0) {
            throw std::runtime_error(
                "poll returned an unrecoverable event: revents=" +
                std::to_string(descriptor.revents));
        }

        v4l2_buffer buffer {};
        v4l2_plane planes[VIDEO_MAX_PLANES] {};

        buffer.type = buffer_type_;
        buffer.memory = V4L2_MEMORY_MMAP;

        if (multiplanar_) {
            buffer.m.planes = planes;
            buffer.length = plane_count_;
        }

        if (::ioctl(fd_, VIDIOC_DQBUF, &buffer) != 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }

            ++stats_.dqbuf_errors;

            if (errno == EIO) {
                std::cerr
                    << "WARNING: VIDIOC_DQBUF returned EIO\n";
                continue;
            }

            throw systemError("VIDIOC_DQBUF");
        }

        const int64_t dequeue_monotonic_ns =
            monotonicNowNs();

        if (buffer.index >= buffers_.size()) {
            throw std::runtime_error(
                "Driver returned an invalid buffer index");
        }

        uint64_t total_bytes_used = 0;

        if (multiplanar_) {
            for (uint32_t plane = 0;
                 plane < plane_count_;
                 ++plane) {
                total_bytes_used +=
                    planes[plane].bytesused;
            }
        } else {
            total_bytes_used = buffer.bytesused;
        }

        uint32_t sequence_gap = 0;

        if (have_previous_sequence) {
            const uint32_t sequence_delta =
                buffer.sequence - previous_sequence;

            if (sequence_delta > 1U &&
                sequence_delta <
                    0x80000000U) {
                sequence_gap = sequence_delta - 1U;
                ++stats_.sequence_gap_events;
                stats_.estimated_dropped_frames +=
                    sequence_gap;
            }
        }

        previous_sequence = buffer.sequence;
        have_previous_sequence = true;

        const int64_t timestamp_ns =
            timevalToNs(buffer.timestamp);

        if (have_previous_timestamp) {
            if (timestamp_ns < previous_timestamp_ns) {
                ++stats_.timestamp_regressions;
            } else {
                const double delta_ms =
                    static_cast<double>(
                        timestamp_ns -
                        previous_timestamp_ns) /
                    1'000'000.0;
                stats_.timestamp_deltas_ms.push_back(
                    delta_ms);
            }
        }

        previous_timestamp_ns = timestamp_ns;
        have_previous_timestamp = true;

        if ((buffer.flags & V4L2_BUF_FLAG_ERROR) != 0U) {
            ++stats_.buffer_error_flags;
        }

        csv << stats_.captured_frames << ','
            << buffer.sequence << ','
            << total_bytes_used << ','
            << buffer.timestamp.tv_sec << ','
            << buffer.timestamp.tv_usec << ','
            << timestampType(buffer.flags) << ','
            << timestampSource(buffer.flags) << ','
            << dequeue_monotonic_ns << ','
            << sequence_gap << ','
            << poll_wait_us << '\n';

        if (stats_.captured_frames <
            config_.save_first) {
            saveFrame(stats_.captured_frames,
                      buffer,
                      multiplanar_ ? planes : nullptr);
        }

        stats_.bytes_used_sum +=
            static_cast<long double>(total_bytes_used);

        if (xioctl(fd_, VIDIOC_QBUF, &buffer) != 0) {
            throw systemError("VIDIOC_QBUF after DQBUF");
        }

        ++stats_.captured_frames;
    }

    const int64_t session_end_ns = monotonicNowNs();
    stats_.elapsed_seconds =
        static_cast<double>(
            session_end_ns - session_start_ns) /
        1'000'000'000.0;

    csv.flush();
    if (!csv.good()) {
        throw std::runtime_error(
            "Failed while flushing CSV output");
    }
}

void V4L2Capture::stopStreaming() noexcept {
    if (!streaming_ || fd_ < 0) {
        return;
    }

    v4l2_buf_type type = buffer_type_;

    if (::ioctl(fd_, VIDIOC_STREAMOFF, &type) != 0) {
        std::cerr
            << "WARNING: VIDIOC_STREAMOFF failed: "
            << std::strerror(errno) << '\n';
    }

    streaming_ = false;
}

void V4L2Capture::releaseResources() noexcept {
    stopStreaming();

    for (auto& buffer : buffers_) {
        for (auto& plane : buffer.planes) {
            if (plane.start != nullptr &&
                plane.start != MAP_FAILED &&
                plane.length > 0) {
                if (::munmap(plane.start,
                             plane.length) != 0) {
                    std::cerr
                        << "WARNING: munmap failed: "
                        << std::strerror(errno) << '\n';
                }
            }

            plane.start = nullptr;
            plane.length = 0;
        }
    }

    buffers_.clear();

    if (fd_ >= 0) {
        if (::close(fd_) != 0) {
            std::cerr
                << "WARNING: close failed: "
                << std::strerror(errno) << '\n';
        }
        fd_ = -1;
    }
}

void V4L2Capture::printSummary() const {
    const double measured_fps =
        stats_.elapsed_seconds > 0.0
            ? static_cast<double>(
                  stats_.captured_frames) /
                  stats_.elapsed_seconds
            : std::numeric_limits<double>::quiet_NaN();

    const double bytes_used_mean =
        stats_.captured_frames > 0
            ? static_cast<double>(
                  stats_.bytes_used_sum /
                  static_cast<long double>(
                      stats_.captured_frames))
            : std::numeric_limits<double>::quiet_NaN();

    const double delta_min =
        stats_.timestamp_deltas_ms.empty()
            ? std::numeric_limits<double>::quiet_NaN()
            : *std::min_element(
                  stats_.timestamp_deltas_ms.begin(),
                  stats_.timestamp_deltas_ms.end());

    const double delta_max =
        stats_.timestamp_deltas_ms.empty()
            ? std::numeric_limits<double>::quiet_NaN()
            : *std::max_element(
                  stats_.timestamp_deltas_ms.begin(),
                  stats_.timestamp_deltas_ms.end());

    const double delta_mean =
        mean(stats_.timestamp_deltas_ms);

    const double jitter =
        populationStdDev(
            stats_.timestamp_deltas_ms);

    std::cout << "\n=== V2_CAPTURE_SUMMARY ===\n";
    std::cout << "requested_format="
              << config_.fourcc << '\n';
    std::cout << "actual_format="
              << fourccToString(actual_fourcc_) << '\n';
    std::cout << "requested_width="
              << config_.width << '\n';
    std::cout << "actual_width="
              << actual_width_ << '\n';
    std::cout << "requested_height="
              << config_.height << '\n';
    std::cout << "actual_height="
              << actual_height_ << '\n';
    std::cout << "requested_fps="
              << config_.fps << '\n';

    if (actual_fps_known_) {
        printDoubleOrNA("actual_fps_parameter",
                        actual_fps_parameter_);
    } else {
        std::cout
            << "actual_fps_parameter=UNKNOWN\n";
    }

    std::cout << "capture_api="
              << (multiplanar_ ? "MULTI_PLANAR"
                               : "SINGLE_PLANAR")
              << '\n';
    std::cout << "buffer_count="
              << buffers_.size() << '\n';
    std::cout << "captured_frames="
              << stats_.captured_frames << '\n';

    printDoubleOrNA("elapsed_seconds",
                    stats_.elapsed_seconds);
    printDoubleOrNA("measured_fps",
                    measured_fps);

    std::cout << "sequence_gaps="
              << stats_.sequence_gap_events << '\n';
    std::cout << "estimated_dropped_frames="
              << stats_.estimated_dropped_frames << '\n';
    std::cout << "poll_timeouts="
              << stats_.poll_timeouts << '\n';
    std::cout << "dqbuf_errors="
              << stats_.dqbuf_errors << '\n';
    std::cout << "buffer_error_flags="
              << stats_.buffer_error_flags << '\n';
    std::cout << "timestamp_regressions="
              << stats_.timestamp_regressions << '\n';

    printDoubleOrNA("timestamp_delta_min_ms",
                    delta_min);
    printDoubleOrNA("timestamp_delta_max_ms",
                    delta_max);
    printDoubleOrNA("timestamp_delta_mean_ms",
                    delta_mean);
    printDoubleOrNA("timestamp_jitter_ms",
                    jitter);
    printDoubleOrNA("bytes_used_mean",
                    bytes_used_mean);

    std::cout << "csv_path="
              << config_.csv_path << '\n';
    std::cout << "=== END_V2_CAPTURE_SUMMARY ===\n";
}

void V4L2Capture::run() {
    openDevice();
    queryCapabilities();
    configureFormat();
    configureFrameRate();
    initializeMmap();
    startStreaming();

    try {
        captureLoop();
        stopStreaming();
        printSummary();
    } catch (...) {
        stopStreaming();
        throw;
    }
}
