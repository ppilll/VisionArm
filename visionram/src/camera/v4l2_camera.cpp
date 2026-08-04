#include "camera/v4l2_camera.h"

#include "common/monotonic_clock.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

namespace visionarm {
namespace {

[[nodiscard]] std::runtime_error SystemError(const std::string& operation) {
    return std::runtime_error(
        operation + ": " + std::strerror(errno) +
        " (errno=" + std::to_string(errno) + ")");
}

}  // namespace

uint32_t FourccFromString(const std::string& text) {
    if (text.size() != 4U) {
        throw std::invalid_argument(
            "FOURCC must contain exactly four characters");
    }
    return v4l2_fourcc(text[0], text[1], text[2], text[3]);
}

std::string FourccToString(uint32_t value) {
    std::string result(4, '\0');
    result[0] = static_cast<char>(value & 0xffU);
    result[1] = static_cast<char>((value >> 8U) & 0xffU);
    result[2] = static_cast<char>((value >> 16U) & 0xffU);
    result[3] = static_cast<char>((value >> 24U) & 0xffU);
    return result;
}

V4L2Camera::V4L2Camera(V4L2CameraConfig config)
    : config_(std::move(config)) {}

V4L2Camera::~V4L2Camera() {
    Stop();
}

int V4L2Camera::Xioctl(
    int fd,
    unsigned long request,
    void* argument) {

    int result = 0;
    do {
        result = ::ioctl(fd, request, argument);
    } while (result == -1 && errno == EINTR);
    return result;
}

void V4L2Camera::Open() {
    if (opened_) {
        throw std::logic_error("V4L2Camera is already open");
    }
    if (config_.device.empty() || config_.width == 0U ||
        config_.height == 0U || config_.pixel_format == 0U ||
        config_.fps == 0U || config_.buffer_count < 2U ||
        config_.timeout_ms <= 0) {
        throw std::invalid_argument("invalid V4L2 camera configuration");
    }

    try {
        OpenDevice();
        CreateWakeEvent();
        QueryCapabilities();
        ConfigureFormat();
        ConfigureFrameRate();
        InitializeMmapAndExport();
        opened_ = true;
    } catch (...) {
        ReleaseResources();
        throw;
    }
}

void V4L2Camera::OpenDevice() {
    struct stat status{};
    if (::stat(config_.device.c_str(), &status) != 0) {
        throw SystemError("stat(" + config_.device + ")");
    }
    if (!S_ISCHR(status.st_mode)) {
        throw std::runtime_error(config_.device + " is not a character device");
    }

    int flags = O_RDWR | O_CLOEXEC;
    if (config_.nonblocking) {
        flags |= O_NONBLOCK;
    }

    fd_ = ::open(config_.device.c_str(), flags);
    if (fd_ < 0) {
        throw SystemError("open(" + config_.device + ")");
    }
}

void V4L2Camera::CreateWakeEvent() {
    wake_event_fd_ = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (wake_event_fd_ < 0) {
        throw SystemError("eventfd");
    }
}

void V4L2Camera::QueryCapabilities() {
    v4l2_capability capability{};
    if (Xioctl(fd_, VIDIOC_QUERYCAP, &capability) != 0) {
        throw SystemError("VIDIOC_QUERYCAP");
    }

    const uint32_t device_caps =
        (capability.capabilities & V4L2_CAP_DEVICE_CAPS) != 0U
            ? capability.device_caps
            : capability.capabilities;

    const bool supports_single =
        (device_caps & V4L2_CAP_VIDEO_CAPTURE) != 0U;
    const bool supports_multi =
        (device_caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0U;
    const bool supports_streaming =
        (device_caps & V4L2_CAP_STREAMING) != 0U;

    if ((!supports_single && !supports_multi) || !supports_streaming) {
        throw std::runtime_error(
            "device does not support streaming video capture");
    }

    if (config_.prefer_multiplanar && supports_multi) {
        multiplanar_ = true;
    } else if (supports_single) {
        multiplanar_ = false;
    } else {
        multiplanar_ = true;
    }

    buffer_type_ = multiplanar_
        ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
        : V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

void V4L2Camera::ConfigureFormat() {
    v4l2_format requested{};
    requested.type = buffer_type_;

    if (multiplanar_) {
        requested.fmt.pix_mp.width = config_.width;
        requested.fmt.pix_mp.height = config_.height;
        requested.fmt.pix_mp.pixelformat = config_.pixel_format;
        requested.fmt.pix_mp.field = V4L2_FIELD_ANY;
    } else {
        requested.fmt.pix.width = config_.width;
        requested.fmt.pix.height = config_.height;
        requested.fmt.pix.pixelformat = config_.pixel_format;
        requested.fmt.pix.field = V4L2_FIELD_ANY;
    }

    if (Xioctl(fd_, VIDIOC_S_FMT, &requested) != 0) {
        throw SystemError("VIDIOC_S_FMT");
    }

    v4l2_format accepted{};
    accepted.type = buffer_type_;
    if (Xioctl(fd_, VIDIOC_G_FMT, &accepted) != 0) {
        throw SystemError("VIDIOC_G_FMT");
    }

    format_ = {};
    format_.multiplanar = multiplanar_;

    if (multiplanar_) {
        format_.width = accepted.fmt.pix_mp.width;
        format_.height = accepted.fmt.pix_mp.height;
        format_.pixel_format = accepted.fmt.pix_mp.pixelformat;
        plane_count_ = accepted.fmt.pix_mp.num_planes;
        format_.colorspace = accepted.fmt.pix_mp.colorspace;
        format_.ycbcr_encoding = accepted.fmt.pix_mp.ycbcr_enc;
        format_.quantization = accepted.fmt.pix_mp.quantization;
        format_.transfer_function = accepted.fmt.pix_mp.xfer_func;

        if (plane_count_ == 0U ||
            plane_count_ > static_cast<uint32_t>(kMaxFramePlanes) ||
            plane_count_ > VIDEO_MAX_PLANES) {
            throw std::runtime_error("driver returned unsupported plane count");
        }

        for (uint32_t plane = 0U; plane < plane_count_; ++plane) {
            format_.bytes_per_line.push_back(
                accepted.fmt.pix_mp.plane_fmt[plane].bytesperline);
            format_.size_image.push_back(
                accepted.fmt.pix_mp.plane_fmt[plane].sizeimage);
        }
    } else {
        format_.width = accepted.fmt.pix.width;
        format_.height = accepted.fmt.pix.height;
        format_.pixel_format = accepted.fmt.pix.pixelformat;
        plane_count_ = 1U;
        format_.colorspace = accepted.fmt.pix.colorspace;
        format_.ycbcr_encoding = accepted.fmt.pix.ycbcr_enc;
        format_.quantization = accepted.fmt.pix.quantization;
        format_.transfer_function = accepted.fmt.pix.xfer_func;
        format_.bytes_per_line.push_back(accepted.fmt.pix.bytesperline);
        format_.size_image.push_back(accepted.fmt.pix.sizeimage);
    }

    format_.plane_count = plane_count_;
}

void V4L2Camera::ConfigureFrameRate() {
    v4l2_streamparm parameters{};
    parameters.type = buffer_type_;
    parameters.parm.capture.timeperframe.numerator = 1U;
    parameters.parm.capture.timeperframe.denominator = config_.fps;
    (void)Xioctl(fd_, VIDIOC_S_PARM, &parameters);

    parameters = {};
    parameters.type = buffer_type_;
    if (Xioctl(fd_, VIDIOC_G_PARM, &parameters) != 0) {
        return;
    }

    const uint32_t numerator =
        parameters.parm.capture.timeperframe.numerator;
    const uint32_t denominator =
        parameters.parm.capture.timeperframe.denominator;

    if (numerator != 0U && denominator != 0U) {
        format_.fps = static_cast<double>(denominator) /
                      static_cast<double>(numerator);
        format_.fps_known = true;
    }
}

void V4L2Camera::InitializeMmapAndExport() {
    v4l2_requestbuffers request{};
    request.count = config_.buffer_count;
    request.type = buffer_type_;
    request.memory = V4L2_MEMORY_MMAP;

    if (Xioctl(fd_, VIDIOC_REQBUFS, &request) != 0) {
        throw SystemError("VIDIOC_REQBUFS(V4L2_MEMORY_MMAP)");
    }
    if (request.count < 2U) {
        throw std::runtime_error("driver allocated fewer than two buffers");
    }

    buffers_.resize(request.count);

    for (uint32_t index = 0U; index < request.count; ++index) {
        v4l2_buffer buffer{};
        v4l2_plane planes[VIDEO_MAX_PLANES]{};
        buffer.type = buffer_type_;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;

        if (multiplanar_) {
            buffer.m.planes = planes;
            buffer.length = VIDEO_MAX_PLANES;
        }

        if (Xioctl(fd_, VIDIOC_QUERYBUF, &buffer) != 0) {
            throw SystemError("VIDIOC_QUERYBUF");
        }

        const uint32_t returned_plane_count =
            multiplanar_ ? buffer.length : 1U;
        if (returned_plane_count < plane_count_) {
            throw std::runtime_error("QUERYBUF returned too few planes");
        }

        BufferMapping& mapping = buffers_[index];
        mapping.planes.resize(plane_count_);
        mapping.state = BufferState::AVAILABLE;
        mapping.active_key_valid = false;

        for (uint32_t plane = 0U; plane < plane_count_; ++plane) {
            const std::size_t length = multiplanar_
                ? static_cast<std::size_t>(planes[plane].length)
                : static_cast<std::size_t>(buffer.length);
            const off_t offset = multiplanar_
                ? static_cast<off_t>(planes[plane].m.mem_offset)
                : static_cast<off_t>(buffer.m.offset);

            void* address = ::mmap(
                nullptr,
                length,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd_,
                offset);
            if (address == MAP_FAILED) {
                throw SystemError("mmap");
            }

            // Record mmap ownership immediately so Open() failure cleanup can
            // unmap it even if the following EXPBUF operation fails.
            PlaneMapping& plane_mapping = mapping.planes[plane];
            plane_mapping.start = address;
            plane_mapping.length = length;

            if (config_.export_dmabuf) {
                v4l2_exportbuffer export_buffer{};
                export_buffer.type = buffer_type_;
                export_buffer.index = index;
                export_buffer.plane = multiplanar_ ? plane : 0U;
                export_buffer.flags = O_CLOEXEC | O_RDWR;

                if (Xioctl(fd_, VIDIOC_EXPBUF, &export_buffer) != 0) {
                    if (config_.require_dmabuf_export) {
                        throw SystemError("VIDIOC_EXPBUF");
                    }
                } else {
                    plane_mapping.dma_fd = export_buffer.fd;
                }
            }
        }
    }
}

bool V4L2Camera::QueueBufferLocked(uint32_t buffer_index) noexcept {
    if (fd_ < 0 || buffer_index >= buffers_.size()) {
        return false;
    }

    BufferMapping& mapping = buffers_[buffer_index];
    if (mapping.state == BufferState::QUEUED) {
        return false;
    }

    v4l2_buffer buffer{};
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    buffer.type = buffer_type_;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = buffer_index;

    if (multiplanar_) {
        buffer.m.planes = planes;
        buffer.length = plane_count_;
    }

    if (Xioctl(fd_, VIDIOC_QBUF, &buffer) != 0) {
        return false;
    }

    if (mapping.state == BufferState::DEQUEUED) {
        outstanding_buffers_.fetch_sub(1U, std::memory_order_acq_rel);
    }
    mapping.state = BufferState::QUEUED;
    mapping.active_key_valid = false;
    mapping.active_key = {};
    return true;
}

void V4L2Camera::QueueAllBuffers() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    for (uint32_t index = 0U;
         index < static_cast<uint32_t>(buffers_.size());
         ++index) {
        if (!QueueBufferLocked(index)) {
            throw SystemError("initial VIDIOC_QBUF");
        }
    }
}

void V4L2Camera::Start() {
    if (!opened_) {
        throw std::logic_error("V4L2Camera must be opened before Start");
    }
    if (streaming_) {
        throw std::logic_error("V4L2Camera is already streaming");
    }

    stop_requested_.store(false, std::memory_order_release);
    outstanding_buffers_.store(0U, std::memory_order_release);
    DrainWakeEvent();
    QueueAllBuffers();

    v4l2_buf_type type = buffer_type_;
    if (Xioctl(fd_, VIDIOC_STREAMON, &type) != 0) {
        throw SystemError("VIDIOC_STREAMON");
    }

    streaming_ = true;
    capture_session_id_ = next_capture_session_id_++;
    next_frame_id_ = 0U;
}

void V4L2Camera::Wake() noexcept {
    if (wake_event_fd_ >= 0) {
        const uint64_t one = 1U;
        const ssize_t ignored = ::write(wake_event_fd_, &one, sizeof(one));
        (void)ignored;
    }
}

void V4L2Camera::RequestStop() noexcept {
    stop_requested_.store(true, std::memory_order_release);
    Wake();
}

void V4L2Camera::DrainWakeEvent() noexcept {
    if (wake_event_fd_ < 0) {
        return;
    }
    uint64_t value = 0U;
    while (::read(wake_event_fd_, &value, sizeof(value)) > 0) {
    }
}

bool V4L2Camera::RequeueByIndexForInternalError(
    uint32_t buffer_index) noexcept {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!streaming_ || buffer_index >= buffers_.size() ||
        buffers_[buffer_index].state != BufferState::DEQUEUED) {
        return false;
    }
    return QueueBufferLocked(buffer_index);
}

bool V4L2Camera::Requeue(const RequeueRequest& request) noexcept {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const uint32_t buffer_index = request.key.buffer_index;
    if (!streaming_ || buffer_index >= buffers_.size()) {
        return false;
    }

    const BufferMapping& mapping = buffers_[buffer_index];
    if (mapping.state != BufferState::DEQUEUED ||
        !mapping.active_key_valid ||
        mapping.active_key != request.key ||
        request.identity.capture_session_id != request.key.capture_session_id ||
        request.identity.frame_id != request.key.frame_id) {
        return false;
    }

    return QueueBufferLocked(buffer_index);
}

CaptureFrameView V4L2Camera::BuildFrameView(
    const v4l2_buffer& buffer,
    const v4l2_plane* planes,
    int64_t dequeue_timestamp_ns) {

    if (buffer.index >= buffers_.size()) {
        throw std::runtime_error("driver returned invalid buffer index");
    }

    const BufferMapping& mapping = buffers_[buffer.index];

    CaptureFrameView frame;
    frame.identity.capture_session_id = capture_session_id_;
    frame.identity.frame_id = next_frame_id_++;
    frame.identity.v4l2_sequence = buffer.sequence;
    frame.identity.dequeue_timestamp_ns = dequeue_timestamp_ns;
    frame.identity.driver_timestamp_ns = TimevalToNs(
        buffer.timestamp.tv_sec,
        buffer.timestamp.tv_usec);

    if ((buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
        V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
        frame.identity.capture_timestamp_ns =
            frame.identity.driver_timestamp_ns;
        frame.identity.timestamp_origin = TimestampOrigin::V4L2_MONOTONIC;
    } else {
        frame.identity.capture_timestamp_ns = dequeue_timestamp_ns;
        frame.identity.timestamp_origin =
            TimestampOrigin::DEQUEUE_MONOTONIC_FALLBACK;
    }

    frame.buffer_index = buffer.index;
    frame.width = static_cast<int>(format_.width);
    frame.height = static_cast<int>(format_.height);
    frame.pixel_format = format_.pixel_format;
    frame.buffer_flags = buffer.flags;
    frame.plane_count = plane_count_;

    for (uint32_t plane = 0U; plane < plane_count_; ++plane) {
        const PlaneMapping& plane_mapping = mapping.planes.at(plane);
        std::size_t data_offset = 0U;
        std::size_t bytes_used = 0U;

        if (multiplanar_) {
            data_offset = std::min<std::size_t>(
                planes[plane].data_offset,
                plane_mapping.length);
            const std::size_t reported_bytes = std::min<std::size_t>(
                planes[plane].bytesused,
                plane_mapping.length);
            bytes_used = reported_bytes > data_offset
                ? reported_bytes - data_offset
                : 0U;
        } else {
            bytes_used = std::min<std::size_t>(
                buffer.bytesused,
                plane_mapping.length);
        }

        auto* valid_address = static_cast<uint8_t*>(plane_mapping.start) +
                              data_offset;

        frame.planes[plane] = CapturePlaneView{
            valid_address,
            plane_mapping.dma_fd,
            data_offset,
            bytes_used,
            plane_mapping.length,
            format_.bytes_per_line.at(plane),
            format_.size_image.at(plane),
        };
    }

    return frame;
}

CaptureResult V4L2Camera::Capture(CaptureFrameView* frame) {
    if (frame == nullptr) {
        throw std::invalid_argument("Capture received null frame");
    }
    if (!streaming_) {
        throw std::logic_error("V4L2Camera is not streaming");
    }
    if (stop_requested_.load(std::memory_order_acquire)) {
        return CaptureResult::STOPPED;
    }

    pollfd descriptors[2]{};
    descriptors[0].fd = fd_;
    descriptors[0].events = POLLIN | POLLPRI;
    descriptors[1].fd = wake_event_fd_;
    descriptors[1].events = POLLIN;

    const int poll_result = ::poll(descriptors, 2, config_.timeout_ms);
    if (poll_result < 0) {
        if (errno == EINTR) {
            return stop_requested_.load(std::memory_order_acquire)
                ? CaptureResult::STOPPED
                : CaptureResult::TIMEOUT;
        }
        throw SystemError("poll");
    }
    if (poll_result == 0) {
        return CaptureResult::TIMEOUT;
    }
    if ((descriptors[1].revents & POLLIN) != 0) {
        DrainWakeEvent();
        return stop_requested_.load(std::memory_order_acquire)
            ? CaptureResult::STOPPED
            : CaptureResult::WAKE;
    }
    if ((descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 &&
        (descriptors[0].revents & (POLLIN | POLLPRI)) == 0) {
        throw std::runtime_error("camera poll returned an unrecoverable event");
    }

    v4l2_buffer buffer{};
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    buffer.type = buffer_type_;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (multiplanar_) {
        buffer.m.planes = planes;
        buffer.length = plane_count_;
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (::ioctl(fd_, VIDIOC_DQBUF, &buffer) != 0) {
            if (errno == EAGAIN || errno == EINTR) {
                return CaptureResult::TIMEOUT;
            }
            if (errno == EIO) {
                return CaptureResult::DROPPED;
            }
            throw SystemError("VIDIOC_DQBUF");
        }

        if (buffer.index >= buffers_.size() ||
            buffers_[buffer.index].state != BufferState::QUEUED) {
            throw std::runtime_error("invalid V4L2 buffer state after DQBUF");
        }

        buffers_[buffer.index].state = BufferState::DEQUEUED;
        buffers_[buffer.index].active_key_valid = false;
        outstanding_buffers_.fetch_add(1U, std::memory_order_acq_rel);
    }

    if ((buffer.flags & V4L2_BUF_FLAG_ERROR) != 0U) {
        (void)RequeueByIndexForInternalError(buffer.index);
        return CaptureResult::DROPPED;
    }

    try {
        const int64_t dequeue_timestamp_ns = MonotonicNowNs();
        *frame = BuildFrameView(
            buffer,
            multiplanar_ ? planes : nullptr,
            dequeue_timestamp_ns);

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            BufferMapping& mapping = buffers_.at(buffer.index);
            if (mapping.state != BufferState::DEQUEUED) {
                throw std::runtime_error(
                    "buffer state changed while building frame view");
            }
            mapping.active_key = MakeCaptureBufferKey(*frame);
            mapping.active_key_valid = true;
        }
        return CaptureResult::FRAME;
    } catch (...) {
        (void)RequeueByIndexForInternalError(buffer.index);
        throw;
    }
}

V4L2DmabufInventory V4L2Camera::GetDmabufInventory() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!opened_) {
        throw std::logic_error("camera must be open before inventory query");
    }

    V4L2DmabufInventory inventory;
    inventory.width = format_.width;
    inventory.height = format_.height;
    inventory.pixel_format = format_.pixel_format;
    inventory.multiplanar_api = format_.multiplanar;
    inventory.plane_count = format_.plane_count;
    inventory.colorspace = format_.colorspace;
    inventory.ycbcr_encoding = format_.ycbcr_encoding;
    inventory.quantization = format_.quantization;
    inventory.transfer_function = format_.transfer_function;
    inventory.buffers.reserve(buffers_.size());

    for (uint32_t buffer_index = 0U;
         buffer_index < static_cast<uint32_t>(buffers_.size());
         ++buffer_index) {
        V4L2DmabufBufferInventory buffer_inventory;
        buffer_inventory.buffer_index = buffer_index;
        const BufferMapping& mapping = buffers_[buffer_index];
        buffer_inventory.planes.reserve(mapping.planes.size());

        for (uint32_t plane_index = 0U;
             plane_index < static_cast<uint32_t>(mapping.planes.size());
             ++plane_index) {
            const PlaneMapping& mapping_plane = mapping.planes[plane_index];
            V4L2DmabufPlaneInventory plane;
            plane.buffer_index = buffer_index;
            plane.plane_index = plane_index;
            plane.dma_fd = mapping_plane.dma_fd;
            plane.mmap_length = mapping_plane.length;
            plane.stride = format_.bytes_per_line.at(plane_index);
            plane.size_image = format_.size_image.at(plane_index);

            if (plane.dma_fd >= 0) {
                const int fd_flags = ::fcntl(plane.dma_fd, F_GETFD);
                plane.fd_cloexec =
                    fd_flags >= 0 && (fd_flags & FD_CLOEXEC) != 0;

                const off_t fd_size = ::lseek(plane.dma_fd, 0, SEEK_END);
                if (fd_size >= 0) {
                    plane.size_query_ok = true;
                    plane.fd_size = static_cast<uint64_t>(fd_size);
                    (void)::lseek(plane.dma_fd, 0, SEEK_SET);
                }

                struct stat fd_stat{};
                if (::fstat(plane.dma_fd, &fd_stat) == 0) {
                    plane.device_id = static_cast<uint64_t>(fd_stat.st_dev);
                    plane.inode = static_cast<uint64_t>(fd_stat.st_ino);
                }
            }
            buffer_inventory.planes.push_back(plane);
        }
        inventory.buffers.push_back(std::move(buffer_inventory));
    }

    return inventory;
}

void V4L2Camera::StopStreaming() noexcept {
    if (!streaming_ || fd_ < 0) {
        streaming_ = false;
        return;
    }

    v4l2_buf_type type = buffer_type_;
    (void)::ioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;

    std::lock_guard<std::mutex> lock(queue_mutex_);
    for (BufferMapping& buffer : buffers_) {
        buffer.state = BufferState::AVAILABLE;
        buffer.active_key_valid = false;
        buffer.active_key = {};
    }
    outstanding_buffers_.store(0U, std::memory_order_release);
}

void V4L2Camera::ReleaseResources() noexcept {
    StopStreaming();

    for (BufferMapping& buffer : buffers_) {
        for (PlaneMapping& plane : buffer.planes) {
            if (plane.dma_fd >= 0) {
                (void)::close(plane.dma_fd);
                plane.dma_fd = -1;
            }
            if (plane.start != nullptr && plane.start != MAP_FAILED) {
                (void)::munmap(plane.start, plane.length);
            }
            plane = {};
        }
        buffer = {};
    }
    buffers_.clear();

    if (fd_ >= 0) {
        (void)::close(fd_);
        fd_ = -1;
    }
    if (wake_event_fd_ >= 0) {
        (void)::close(wake_event_fd_);
        wake_event_fd_ = -1;
    }

    opened_ = false;
    plane_count_ = 1U;
    format_ = {};
}

void V4L2Camera::Stop() noexcept {
    RequestStop();
    ReleaseResources();
}

}  // namespace visionarm
