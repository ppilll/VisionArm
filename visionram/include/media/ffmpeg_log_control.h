#pragma once

namespace visionarm {

// Maps the unified "ffmpeg" module threshold to libavutil's process-wide
// level. Called when FFmpeg is initialized and after supervisor SIGHUP reload.
void ApplyFfmpegLogLevel() noexcept;

}  // namespace visionarm
