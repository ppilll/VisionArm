#pragma once

#include "video/video_types.h"

#include <cstdint>
#include <fstream>
#include <string>

namespace visionarm {

struct Nv12MppLayout {
    int width = 0;
    int height = 0;
    int horizontal_stride = 0;
    int vertical_stride = 0;
    uint64_t minimum_bytes = 0U;
};

// Derives the physical NV12 vertical stride from the frozen V4L2 contract.
[[nodiscard]] bool DeriveNv12MppLayout(
    const CaptureFrameView& frame,
    int configured_vertical_stride,
    Nv12MppLayout* layout) noexcept;

struct H265FileSinkSnapshot {
    uint64_t packets_written = 0U;
    uint64_t bytes_written = 0U;
    uint64_t write_failures = 0U;
};

class H265FileSink final : public IEncodedPacketSink {
public:
    explicit H265FileSink(std::string path);
    ~H265FileSink() override;

    H265FileSink(const H265FileSink&) = delete;
    H265FileSink& operator=(const H265FileSink&) = delete;

    [[nodiscard]] bool Write(const EncodedPacket& packet) noexcept override;
    void Flush() noexcept override;

    [[nodiscard]] bool opened() const noexcept { return stream_.is_open(); }
    [[nodiscard]] H265FileSinkSnapshot snapshot() const noexcept {
        return snapshot_;
    }

private:
    std::ofstream stream_;
    H265FileSinkSnapshot snapshot_;
};

}  // namespace visionarm
