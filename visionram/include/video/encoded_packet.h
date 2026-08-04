#pragma once

#include "common/pipeline_types.h"

#include <cstdint>
#include <vector>

namespace visionarm {

struct EncodedPacket {
    FrameIdentity identity;
    int64_t pts_us = 0;
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

}  // namespace visionarm
