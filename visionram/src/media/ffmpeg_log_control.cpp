#include "media/ffmpeg_log_control.h"

#include "logging/logger.h"

extern "C" {
#include <libavutil/log.h>
}

namespace visionarm {

void ApplyFfmpegLogLevel() noexcept {
    const logging::AsyncLogger& logger = logging::GlobalLogger();
    int level = AV_LOG_QUIET;
    if (logger.IsEnabled(logging::LogLevel::TRACE, "ffmpeg")) {
        level = AV_LOG_TRACE;
    } else if (logger.IsEnabled(logging::LogLevel::DEBUG, "ffmpeg")) {
        level = AV_LOG_DEBUG;
    } else if (logger.IsEnabled(logging::LogLevel::INFO, "ffmpeg")) {
        level = AV_LOG_INFO;
    } else if (logger.IsEnabled(logging::LogLevel::WARN, "ffmpeg")) {
        level = AV_LOG_WARNING;
    } else if (logger.IsEnabled(logging::LogLevel::ERROR, "ffmpeg")) {
        level = AV_LOG_ERROR;
    } else if (logger.IsEnabled(logging::LogLevel::FATAL, "ffmpeg")) {
        level = AV_LOG_FATAL;
    }
    av_log_set_level(level);
}

}  // namespace visionarm
