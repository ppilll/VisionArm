#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace visionarm::logging {

enum class LogLevel : std::uint8_t {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    FATAL = 5,
    OFF = 6,
};

[[nodiscard]] const char* LogLevelName(LogLevel level) noexcept;
[[nodiscard]] bool ParseLogLevel(std::string_view text,
                                 LogLevel* level) noexcept;

struct ModuleLogLevel {
    std::string module;
    LogLevel level = LogLevel::INFO;
};

struct LoggerConfig {
#if defined(VISIONARM_COMPILETIME_LOG_LEVEL)
    LogLevel default_level =
        static_cast<LogLevel>(VISIONARM_COMPILETIME_LOG_LEVEL);
#else
    LogLevel default_level = LogLevel::INFO;
#endif
    std::vector<ModuleLogLevel> module_levels;
    std::size_t queue_capacity = 1'024U;
    std::size_t critical_queue_capacity = 32U;
};

struct LoggerSnapshot {
    bool running = false;
    std::size_t queue_capacity = 0U;
    std::size_t critical_queue_capacity = 0U;
    std::size_t current_size = 0U;
    std::size_t high_watermark = 0U;
    std::uint64_t accepted = 0U;
    std::uint64_t emitted = 0U;
    std::uint64_t dropped = 0U;
    std::uint64_t dropped_contention = 0U;
    std::uint64_t dropped_overflow = 0U;
    std::uint64_t dropped_critical = 0U;
    std::uint64_t sink_failures = 0U;
};

class AsyncLogger final {
public:
    explicit AsyncLogger(LoggerConfig config = {},
                         std::ostream* sink = nullptr);
    ~AsyncLogger();

    AsyncLogger(const AsyncLogger&) = delete;
    AsyncLogger& operator=(const AsyncLogger&) = delete;

    [[nodiscard]] bool IsEnabled(LogLevel level,
                                 std::string_view module) const noexcept;
    void Enqueue(LogLevel level,
                 std::string module,
                 std::string message) noexcept;

    // Reconfiguration is a supervisor/control-plane operation. Queue sizes are
    // immutable after construction so producers never observe a queue swap.
    [[nodiscard]] bool Reconfigure(const LoggerConfig& config,
                                   std::string* error = nullptr) noexcept;
    [[nodiscard]] bool Flush(
        std::chrono::milliseconds timeout = std::chrono::seconds(2)) noexcept;
    void Shutdown() noexcept;
    [[nodiscard]] LoggerSnapshot Snapshot() const noexcept;

    [[nodiscard]] static std::string FormatRecordForTest(
        std::chrono::system_clock::time_point timestamp,
        LogLevel level,
        std::string_view module,
        std::string_view message);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

[[nodiscard]] bool ApplyModuleLogLevel(
    std::string_view specification,
    LoggerConfig* config,
    std::string* error = nullptr) noexcept;

// File syntax is deliberately small and safe for SIGHUP reload:
//   level=INFO
//   module.pipeline=DEBUG
// Blank lines and lines beginning with '#' are ignored. File values override
// the supplied base config; queue capacities remain those of the base config.
[[nodiscard]] bool LoadLoggerConfigFile(
    const std::string& path,
    const LoggerConfig& base,
    LoggerConfig* loaded,
    std::string* error = nullptr) noexcept;

AsyncLogger& GlobalLogger();
[[nodiscard]] bool ConfigureGlobalLogger(
    const LoggerConfig& config,
    std::string* error = nullptr) noexcept;
[[nodiscard]] bool FlushGlobalLogger(
    std::chrono::milliseconds timeout = std::chrono::seconds(2)) noexcept;
void ShutdownGlobalLogger() noexcept;
[[nodiscard]] LoggerSnapshot GlobalLoggerSnapshot() noexcept;

template <typename... Parts>
void Log(LogLevel level, std::string_view module, Parts&&... parts) noexcept {
    AsyncLogger& logger = GlobalLogger();
    if (!logger.IsEnabled(level, module)) {
        return;
    }
    try {
        std::ostringstream message;
        (message << ... << std::forward<Parts>(parts));
        logger.Enqueue(level, std::string(module), message.str());
    } catch (...) {
        // Logging is observational. Allocation/formatting failure must not
        // change Camera, media, inference, Telemetry, or UART behavior.
    }
}

}  // namespace visionarm::logging
