#pragma once

#include "video/nv12_mpp_layout.h"
#include "video/video_encoder.h"

#include <cstddef>
#include <cstdint>
#include <vector>

#include <rk_mpi.h>
#include <mpp_buffer.h>
#include <mpp_frame.h>

namespace visionarm {

struct MppH265EncoderConfig {
    int width = 0;
    int height = 0;
    int horizontal_stride = 0;

    // Must come from the frozen R2 Camera layout. The board probe derives it
    // from the negotiated stride/size_image and passes it explicitly.
    int vertical_stride = 0;

    int fps_numerator = 30;
    int fps_denominator = 1;
    int bitrate_bps = 4'000'000;
    int gop_length = 60;
    int qp_min = 10;
    int qp_max = 51;
    int qp_min_i = 10;
    int qp_max_i = 51;

    std::size_t max_source_buffers = 0U;
    std::size_t packet_buffer_bytes = 0U;
};

class MppH265Encoder final : public IVideoEncoder {
public:
    MppH265Encoder() = default;
    ~MppH265Encoder() override;

    MppH265Encoder(const MppH265Encoder&) = delete;
    MppH265Encoder& operator=(const MppH265Encoder&) = delete;

    void Initialize(const MppH265EncoderConfig& config);
    void Shutdown() noexcept;

    [[nodiscard]] bool initialized() const noexcept override {
        return initialized_;
    }

    [[nodiscard]] bool Encode(
        const CaptureFrameView& frame,
        std::vector<EncodedPacket>* packets) noexcept override;

    [[nodiscard]] std::vector<EncodedPacket>
    CodecConfigPackets() const override {
        return codec_config_packets_;
    }

    [[nodiscard]] VideoEncoderSnapshot snapshot()
        const noexcept override {
        return snapshot_;
    }

private:
    struct ImportedSource {
        int fd = -1;
        std::size_t size = 0U;
        MppBuffer buffer = nullptr;
    };

    [[nodiscard]] bool ConfigureEncoder() noexcept;
    [[nodiscard]] bool BuildCodecHeader() noexcept;
    [[nodiscard]] bool EnsureSourceImported(
        const CaptureFrameView& frame,
        MppBuffer* buffer) noexcept;
    [[nodiscard]] bool EncodeOne(
        const CaptureFrameView& frame,
        MppBuffer source,
        std::vector<EncodedPacket>* packets) noexcept;

    MppH265EncoderConfig config_;
    MppCtx context_ = nullptr;
    MppApi* mpi_ = nullptr;
    MppBufferGroup packet_group_ = nullptr;
    MppBuffer packet_buffer_ = nullptr;
    std::vector<ImportedSource> imported_sources_;
    std::vector<EncodedPacket> codec_config_packets_;
    VideoEncoderSnapshot snapshot_;
    bool initialized_ = false;
};

}  // namespace visionarm
