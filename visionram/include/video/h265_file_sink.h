#pragma once

#include "video/encoded_packet_sink.h"

#include <cstdint>
#include <fstream>
#include <string>

namespace visionarm {

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
