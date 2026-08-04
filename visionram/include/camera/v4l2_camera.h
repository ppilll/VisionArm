#pragma once

#include "camera/camera_source.h"
#include "camera/v4l2_dmabuf_contract.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <linux/videodev2.h>
#include <mutex>
#include <string>
#include <vector>

namespace visionarm {

struct V4L2CameraConfig {
    std::string device;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixel_format = 0;
    uint32_t fps = 0;
    uint32_t buffer_count = 6;
    int timeout_ms = 2000;
    bool nonblocking = true;
    bool prefer_multiplanar = false;
    bool export_dmabuf = true;
    bool require_dmabuf_export = true;
};

struct CameraFormat {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixel_format = 0;
    bool multiplanar = false;
    uint32_t plane_count = 0;
    std::vector<uint32_t> bytes_per_line;
    std::vector<uint32_t> size_image;

    uint32_t colorspace = 0;
    uint32_t ycbcr_encoding = 0;
    uint32_t quantization = 0;
    uint32_t transfer_function = 0;

    bool fps_known = false;
    double fps = 0.0;
};

[[nodiscard]] uint32_t FourccFromString(const std::string& text);
[[nodiscard]] std::string FourccToString(uint32_t value);

class V4L2Camera final : public ICameraSource {
public:
    explicit V4L2Camera(V4L2CameraConfig config);
    ~V4L2Camera() override;

    V4L2Camera(const V4L2Camera&) = delete;
    V4L2Camera& operator=(const V4L2Camera&) = delete;

    void Open() override;
    void Start() override;
    CaptureResult Capture(CaptureFrameView* frame) override;
    bool Requeue(const RequeueRequest& request) noexcept override;
    void Wake() noexcept override;
    void RequestStop() noexcept override;
    void Stop() noexcept override;

    [[nodiscard]] std::size_t buffer_count() const noexcept override {
        return buffers_.size();
    }

    [[nodiscard]] const CameraFormat& format() const noexcept {
        return format_;
    }

    [[nodiscard]] uint32_t outstanding_buffers() const noexcept override {
        return outstanding_buffers_.load(std::memory_order_acquire);
    }

    [[nodiscard]] V4L2DmabufInventory GetDmabufInventory() const;

private:
    enum class BufferState {
        AVAILABLE,
        QUEUED,
        DEQUEUED,
    };

    struct PlaneMapping {
        void* start = nullptr;
        std::size_t length = 0;
        int dma_fd = -1;
    };

    struct BufferMapping {
        std::vector<PlaneMapping> planes;
        BufferState state = BufferState::AVAILABLE;
        bool active_key_valid = false;
        CaptureBufferKey active_key;
    };

    void OpenDevice();
    void CreateWakeEvent();
    void QueryCapabilities();
    void ConfigureFormat();
    void ConfigureFrameRate();
    void InitializeMmapAndExport();
    void QueueAllBuffers();
    void StopStreaming() noexcept;
    void ReleaseResources() noexcept;
    void DrainWakeEvent() noexcept;

    [[nodiscard]] CaptureFrameView BuildFrameView(
        const v4l2_buffer& buffer,
        const v4l2_plane* planes,
        int64_t dequeue_timestamp_ns);

    bool QueueBufferLocked(uint32_t buffer_index) noexcept;
    bool RequeueByIndexForInternalError(uint32_t buffer_index) noexcept;

    static int Xioctl(int fd, unsigned long request, void* argument);

    V4L2CameraConfig config_;
    CameraFormat format_;

    int fd_ = -1;
    int wake_event_fd_ = -1;
    bool opened_ = false;
    bool streaming_ = false;
    bool multiplanar_ = false;
    v4l2_buf_type buffer_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint32_t plane_count_ = 1;
    std::vector<BufferMapping> buffers_;

    mutable std::mutex queue_mutex_;

    std::atomic<bool> stop_requested_{false};
    std::atomic<uint32_t> outstanding_buffers_{0};
    uint64_t next_capture_session_id_ = 1U;
    uint64_t capture_session_id_ = 0U;
    uint64_t next_frame_id_ = 0U;
};

}  // namespace visionarm
