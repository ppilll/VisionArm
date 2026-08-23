#pragma once

#include "audio/encoded_audio_packet.h"

namespace visionarm {

class IEncodedAudioPacketSink {
public:
    virtual ~IEncodedAudioPacketSink() = default;

    IEncodedAudioPacketSink(const IEncodedAudioPacketSink&) = delete;
    IEncodedAudioPacketSink& operator=(const IEncodedAudioPacketSink&) = delete;

    [[nodiscard]] virtual bool WriteAudio(
        const EncodedAudioPacket& packet) noexcept = 0;
    virtual void FlushAudio() noexcept = 0;

protected:
    IEncodedAudioPacketSink() = default;
};

}  // namespace visionarm
