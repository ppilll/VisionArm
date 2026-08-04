#pragma once

#include "video/encoded_packet.h"

namespace visionarm {

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

}  // namespace visionarm
