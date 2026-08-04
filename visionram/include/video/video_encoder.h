#pragma once

#include "camera/capture_buffer_contract.h"
#include "video/encoded_packet.h"

#include <vector>

namespace visionarm {

class IVideoEncoder {
public:
    virtual ~IVideoEncoder() = default;

    IVideoEncoder(const IVideoEncoder&) = delete;
    IVideoEncoder& operator=(const IVideoEncoder&) = delete;

    [[nodiscard]] virtual bool initialized() const noexcept = 0;

    // The implementation must copy all returned compressed bytes into
    // application-owned EncodedPacket storage before returning. When this
    // function returns, the caller may release the Camera FrameLease.
    [[nodiscard]] virtual bool Encode(
        const CaptureFrameView& frame,
        std::vector<EncodedPacket>* packets) noexcept = 0;

    [[nodiscard]] virtual std::vector<EncodedPacket>
    CodecConfigPackets() const = 0;

    [[nodiscard]] virtual VideoEncoderSnapshot snapshot()
        const noexcept = 0;

protected:
    IVideoEncoder() = default;
};

}  // namespace visionarm
