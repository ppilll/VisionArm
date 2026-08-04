#include "video/h265_file_sink.h"

#include <utility>

namespace visionarm {

H265FileSink::H265FileSink(std::string path)
    : stream_(std::move(path), std::ios::binary | std::ios::trunc) {}

H265FileSink::~H265FileSink() {
    Flush();
}

bool H265FileSink::Write(const EncodedPacket& packet) noexcept {
    if (!stream_.is_open()) {
        ++snapshot_.write_failures;
        return false;
    }
    if (!packet.bytes.empty()) {
        stream_.write(
            reinterpret_cast<const char*>(packet.bytes.data()),
            static_cast<std::streamsize>(packet.bytes.size()));
    }
    if (!stream_) {
        ++snapshot_.write_failures;
        return false;
    }
    ++snapshot_.packets_written;
    snapshot_.bytes_written += packet.bytes.size();
    return true;
}

void H265FileSink::Flush() noexcept {
    if (stream_.is_open()) {
        stream_.flush();
    }
}

}  // namespace visionarm
