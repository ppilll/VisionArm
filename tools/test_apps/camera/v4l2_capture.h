#pragma once

#include <csignal>
#include <cstddef>
#include <cstdint>
#include <linux/videodev2.h>
#include <string>
#include <vector>

struct CaptureConfig {
    std::string device;
    uint32_t width = 0;
    uint32_t height = 0;
    std::string fourcc;
    uint32_t fps = 0;
    uint32_t buffer_count = 4;
    uint64_t frame_count = 300;
    int timeout_ms = 2000;
    std::string output_dir = "reports/images/v2";
    uint32_t save_first = 1;
    std::string csv_path = "logs/v2_camera/frames.csv";
    bool nonblock = false;
};

class V4L2Capture {
public:
    explicit V4L2Capture(CaptureConfig config);
    ~V4L2Capture();

    V4L2Capture(const V4L2Capture&) = delete;
    V4L2Capture& operator=(const V4L2Capture&) = delete;

    void setStopFlag(volatile sig_atomic_t* stop_flag);
    void run();

private:
    struct PlaneMapping {
        void* start = nullptr;
        std::size_t length = 0;
    };

    struct BufferMapping {
        std::vector<PlaneMapping> planes;
    };

    struct Statistics {
        uint64_t captured_frames = 0;
        uint64_t sequence_gap_events = 0;
        uint64_t estimated_dropped_frames = 0;
        uint64_t poll_timeouts = 0;
        uint64_t dqbuf_errors = 0;
        uint64_t buffer_error_flags = 0;
        uint64_t timestamp_regressions = 0;
        long double bytes_used_sum = 0.0L;
        std::vector<double> timestamp_deltas_ms;
        double elapsed_seconds = 0.0;
    };

    CaptureConfig config_;
    int fd_ = -1;
    bool streaming_ = false;
    bool multiplanar_ = false;
    v4l2_buf_type buffer_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint32_t plane_count_ = 1;
    uint32_t actual_width_ = 0;
    uint32_t actual_height_ = 0;
    uint32_t actual_fourcc_ = 0;
    std::vector<uint32_t> bytes_per_line_;
    std::vector<uint32_t> size_image_;
    bool actual_fps_known_ = false;
    double actual_fps_parameter_ = 0.0;
    std::vector<BufferMapping> buffers_;
    Statistics stats_;
    volatile sig_atomic_t* stop_flag_ = nullptr;

    void openDevice();
    void queryCapabilities();
    void configureFormat();
    void configureFrameRate();
    void initializeMmap();
    void queueAllBuffers();
    void startStreaming();
    void captureLoop();
    void stopStreaming() noexcept;
    void releaseResources() noexcept;
    void printSummary() const;

    void saveFrame(uint64_t frame_index,
                   const v4l2_buffer& buffer,
                   const v4l2_plane* planes);

    bool stopRequested() const;

    static int xioctl(int fd, unsigned long request, void* argument);
    static uint32_t fourccFromString(const std::string& text);
    static std::string fourccToString(uint32_t value);
    static std::string timestampType(uint32_t flags);
    static std::string timestampSource(uint32_t flags);
    static int64_t timevalToNs(const timeval& value);
    static int64_t monotonicNowNs();
    static std::string frameExtension(uint32_t fourcc);
};