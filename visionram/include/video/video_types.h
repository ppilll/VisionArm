#pragma once

#include "camera/capture_buffer.h"
#include "common/pipeline_types.h"

#include <cstdint>
#include <vector>

namespace visionarm {

struct EncodedPacket {
    FrameIdentity identity;

    // Media PTS in microseconds from the common monotonic media epoch.
    // The original absolute capture timestamp remains in identity.
    int64_t pts_us = 0;
    int64_t dts_us = 0;
    int64_t duration_us = 0;
    bool keyframe = false;
    bool codec_config = false;
    bool end_of_frame = true;
    bool end_of_stream = false;
    std::vector<uint8_t> bytes;
};

struct VideoEncoderSnapshot {
    uint64_t submitted_frames = 0U;
    uint64_t encoded_frames = 0U;
    uint64_t encode_failures = 0U;
    uint64_t emitted_packets = 0U;
    uint64_t emitted_bytes = 0U;
    uint64_t imported_source_buffers = 0U;
    uint64_t source_buffer_reimports = 0U;
    uint64_t codec_config_packets = 0U;
};

class IEncodedPacketSink {
public:
    virtual ~IEncodedPacketSink() = default;
    IEncodedPacketSink(const IEncodedPacketSink&) = delete;
    IEncodedPacketSink& operator=(const IEncodedPacketSink&) = delete;
    [[nodiscard]] virtual bool Write(const EncodedPacket& packet) noexcept = 0;
    virtual void Flush() noexcept = 0;

protected:
    IEncodedPacketSink() = default;
};

class IVideoEncoder {
public:
    virtual ~IVideoEncoder() = default;
    IVideoEncoder(const IVideoEncoder&) = delete;
    IVideoEncoder& operator=(const IVideoEncoder&) = delete;
    [[nodiscard]] virtual bool initialized() const noexcept = 0;
    // Encode returns owning packet bytes, so the caller can release the
    // Camera FrameLease before mux, file, or network delivery.
    [[nodiscard]] virtual bool Encode(const CaptureFrameView& frame,
                                      std::vector<EncodedPacket>* packets) noexcept = 0;
    [[nodiscard]] virtual std::vector<EncodedPacket> CodecConfigPackets() const = 0;
    [[nodiscard]] virtual VideoEncoderSnapshot snapshot() const noexcept = 0;

protected:
    IVideoEncoder() = default;
};

}  // namespace visionarm
