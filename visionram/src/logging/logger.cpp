#include "logging/logger.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cctype>
#include <ctime>
#include <deque>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

namespace visionarm::logging {
namespace {

struct Thresholds {
    LogLevel default_level = LogLevel::INFO;
    std::vector<ModuleLogLevel> modules;
};

struct Record {
    std::chrono::system_clock::time_point timestamp;
    LogLevel level = LogLevel::INFO;
    std::string module;
    std::string message;
};

std::string Trim(std::string value) {
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(),
                std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(),
                value.end());
    return value;
}

bool IsValidModule(std::string_view module) noexcept {
    if (module.empty()) return false;
    return std::all_of(module.begin(), module.end(), [](unsigned char ch) {
        return std::isalnum(ch) || ch == '.' || ch == '_' || ch == '-';
    });
}

std::string Upper(std::string_view text) {
    std::string result(text);
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::toupper(ch));
                   });
    return result;
}

void SetError(std::string* error, std::string message) noexcept {
    if (error == nullptr) return;
    try {
        *error = std::move(message);
    } catch (...) {
    }
}

std::shared_ptr<const Thresholds> MakeThresholds(const LoggerConfig& config) {
    auto thresholds = std::make_shared<Thresholds>();
    thresholds->default_level = config.default_level;
    thresholds->modules = config.module_levels;
    return thresholds;
}

bool IsValidLevel(LogLevel level) noexcept {
    return level >= LogLevel::TRACE && level <= LogLevel::OFF;
}

}  // namespace

const char* LogLevelName(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::TRACE: return "TRACE";
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARN: return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        case LogLevel::OFF: return "OFF";
    }
    return "UNKNOWN";
}

bool ParseLogLevel(std::string_view text, LogLevel* level) noexcept {
    if (level == nullptr) return false;
    try {
        const std::string upper = Upper(Trim(std::string(text)));
        if (upper == "TRACE") *level = LogLevel::TRACE;
        else if (upper == "DEBUG") *level = LogLevel::DEBUG;
        else if (upper == "INFO") *level = LogLevel::INFO;
        else if (upper == "WARN" || upper == "WARNING") {
            *level = LogLevel::WARN;
        } else if (upper == "ERROR") *level = LogLevel::ERROR;
        else if (upper == "FATAL") *level = LogLevel::FATAL;
        else if (upper == "OFF") *level = LogLevel::OFF;
        else return false;
        return true;
    } catch (...) {
        return false;
    }
}

class AsyncLogger::Impl final {
public:
    Impl(LoggerConfig config, std::ostream* sink)
        : queue_capacity_(config.queue_capacity),
          critical_queue_capacity_(config.critical_queue_capacity),
          thresholds_(MakeThresholds(config)),
          sink_(sink == nullptr ? &std::cerr : sink),
          worker_(&Impl::WorkerMain, this) {}

    ~Impl() { Shutdown(); }

    bool IsEnabled(LogLevel level, std::string_view module) const noexcept {
        if (!IsValidLevel(level) || level == LogLevel::OFF) return false;
        const std::shared_ptr<const Thresholds> thresholds =
            std::atomic_load_explicit(&thresholds_, std::memory_order_acquire);
        LogLevel threshold = thresholds->default_level;
        for (const ModuleLogLevel& item : thresholds->modules) {
            if (item.module == module) {
                threshold = item.level;
                break;
            }
        }
        return threshold != LogLevel::OFF && level >= threshold;
    }

    void Enqueue(LogLevel level,
                 std::string module,
                 std::string message) noexcept {
        if (!IsEnabled(level, module) ||
            !running_.load(std::memory_order_acquire)) {
            return;
        }
        const bool critical = level >= LogLevel::ERROR;
        std::unique_lock<std::mutex> lock(queue_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            dropped_.fetch_add(1U, std::memory_order_relaxed);
            dropped_contention_.fetch_add(1U, std::memory_order_relaxed);
            if (critical) {
                dropped_critical_.fetch_add(1U, std::memory_order_relaxed);
            }
            return;
        }

        std::deque<Record>& queue = critical ? critical_queue_ : queue_;
        const std::size_t capacity =
            critical ? critical_queue_capacity_ : queue_capacity_;
        if (queue.size() >= capacity) {
            dropped_.fetch_add(1U, std::memory_order_relaxed);
            dropped_overflow_.fetch_add(1U, std::memory_order_relaxed);
            if (critical) {
                dropped_critical_.fetch_add(1U, std::memory_order_relaxed);
            }
            return;
        }

        try {
            queue.push_back(Record{std::chrono::system_clock::now(), level,
                                   std::move(module), std::move(message)});
        } catch (...) {
            dropped_.fetch_add(1U, std::memory_order_relaxed);
            dropped_overflow_.fetch_add(1U, std::memory_order_relaxed);
            if (critical) {
                dropped_critical_.fetch_add(1U, std::memory_order_relaxed);
            }
            return;
        }
        accepted_.fetch_add(1U, std::memory_order_relaxed);
        const std::size_t size = queue_.size() + critical_queue_.size();
        high_watermark_ = std::max(high_watermark_, size);
        lock.unlock();
        ready_.notify_one();
    }

    bool Reconfigure(const LoggerConfig& config,
                     std::string* error) noexcept {
        if (!IsValidLevel(config.default_level)) {
            SetError(error, "invalid default log level");
            return false;
        }
        if (config.queue_capacity != queue_capacity_ ||
            config.critical_queue_capacity != critical_queue_capacity_) {
            SetError(error, "log queue capacities cannot change at runtime");
            return false;
        }
        for (const ModuleLogLevel& item : config.module_levels) {
            if (!IsValidModule(item.module) || !IsValidLevel(item.level)) {
                SetError(error, "invalid module log override");
                return false;
            }
        }
        try {
            std::shared_ptr<const Thresholds> thresholds =
                MakeThresholds(config);
            std::atomic_store_explicit(&thresholds_, std::move(thresholds),
                                       std::memory_order_release);
            return true;
        } catch (const std::exception& exception) {
            SetError(error, exception.what());
            return false;
        } catch (...) {
            SetError(error, "unknown logger reconfiguration failure");
            return false;
        }
    }

    bool Flush(std::chrono::milliseconds timeout) noexcept {
        try {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            const bool drained = drained_.wait_for(lock, timeout, [this] {
                return queue_.empty() && critical_queue_.empty() && !writing_;
            });
            lock.unlock();
            sink_->flush();
            if (!sink_->good()) {
                sink_failures_.fetch_add(1U, std::memory_order_relaxed);
                return false;
            }
            return drained;
        } catch (...) {
            sink_failures_.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }
    }

    void Shutdown() noexcept {
        bool expected = true;
        if (!running_.compare_exchange_strong(
                expected, false, std::memory_order_acq_rel)) {
            return;
        }
        ready_.notify_all();
        if (worker_.joinable()) worker_.join();
        try {
            sink_->flush();
            if (!sink_->good()) {
                sink_failures_.fetch_add(1U, std::memory_order_relaxed);
            }
        } catch (...) {
            sink_failures_.fetch_add(1U, std::memory_order_relaxed);
        }
    }

    LoggerSnapshot Snapshot() const noexcept {
        LoggerSnapshot snapshot;
        snapshot.running = running_.load(std::memory_order_acquire);
        snapshot.queue_capacity = queue_capacity_;
        snapshot.critical_queue_capacity = critical_queue_capacity_;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            snapshot.current_size = queue_.size() + critical_queue_.size();
            snapshot.high_watermark = high_watermark_;
        }
        snapshot.accepted = accepted_.load(std::memory_order_relaxed);
        snapshot.emitted = emitted_.load(std::memory_order_relaxed);
        snapshot.dropped = dropped_.load(std::memory_order_relaxed);
        snapshot.dropped_contention =
            dropped_contention_.load(std::memory_order_relaxed);
        snapshot.dropped_overflow =
            dropped_overflow_.load(std::memory_order_relaxed);
        snapshot.dropped_critical =
            dropped_critical_.load(std::memory_order_relaxed);
        snapshot.sink_failures = sink_failures_.load(std::memory_order_relaxed);
        return snapshot;
    }

private:
    void WorkerMain() noexcept {
        while (true) {
            Record record;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                ready_.wait(lock, [this] {
                    return !running_.load(std::memory_order_acquire) ||
                           !critical_queue_.empty() || !queue_.empty();
                });
                if (critical_queue_.empty() && queue_.empty() &&
                    !running_.load(std::memory_order_acquire)) {
                    drained_.notify_all();
                    return;
                }
                std::deque<Record>& source =
                    critical_queue_.empty() ? queue_ : critical_queue_;
                record = std::move(source.front());
                source.pop_front();
                writing_ = true;
            }

            try {
                *sink_ << AsyncLogger::FormatRecordForTest(
                    record.timestamp, record.level, record.module,
                    record.message);
                if (!sink_->good()) {
                    sink_failures_.fetch_add(1U, std::memory_order_relaxed);
                }
            } catch (...) {
                sink_failures_.fetch_add(1U, std::memory_order_relaxed);
            }
            emitted_.fetch_add(1U, std::memory_order_relaxed);

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                writing_ = false;
                if (queue_.empty() && critical_queue_.empty()) {
                    drained_.notify_all();
                }
            }
        }
    }

    const std::size_t queue_capacity_;
    const std::size_t critical_queue_capacity_;
    mutable std::mutex queue_mutex_;
    std::condition_variable ready_;
    std::condition_variable drained_;
    std::deque<Record> queue_;
    std::deque<Record> critical_queue_;
    bool writing_ = false;
    std::size_t high_watermark_ = 0U;
    std::shared_ptr<const Thresholds> thresholds_;
    std::ostream* sink_;
    std::atomic<bool> running_{true};
    std::thread worker_;
    std::atomic<std::uint64_t> accepted_{0U};
    std::atomic<std::uint64_t> emitted_{0U};
    std::atomic<std::uint64_t> dropped_{0U};
    std::atomic<std::uint64_t> dropped_contention_{0U};
    std::atomic<std::uint64_t> dropped_overflow_{0U};
    std::atomic<std::uint64_t> dropped_critical_{0U};
    std::atomic<std::uint64_t> sink_failures_{0U};
};

AsyncLogger::AsyncLogger(LoggerConfig config, std::ostream* sink)
    : impl_(std::make_unique<Impl>(std::move(config), sink)) {}

AsyncLogger::~AsyncLogger() = default;

bool AsyncLogger::IsEnabled(LogLevel level,
                            std::string_view module) const noexcept {
    return impl_->IsEnabled(level, module);
}

void AsyncLogger::Enqueue(LogLevel level,
                          std::string module,
                          std::string message) noexcept {
    impl_->Enqueue(level, std::move(module), std::move(message));
}

bool AsyncLogger::Reconfigure(const LoggerConfig& config,
                              std::string* error) noexcept {
    return impl_->Reconfigure(config, error);
}

bool AsyncLogger::Flush(std::chrono::milliseconds timeout) noexcept {
    return impl_->Flush(timeout);
}

void AsyncLogger::Shutdown() noexcept { impl_->Shutdown(); }

LoggerSnapshot AsyncLogger::Snapshot() const noexcept {
    return impl_->Snapshot();
}

std::string AsyncLogger::FormatRecordForTest(
    std::chrono::system_clock::time_point timestamp,
    LogLevel level,
    std::string_view module,
    std::string_view message) {
    const auto milliseconds = std::chrono::duration_cast<
        std::chrono::milliseconds>(timestamp.time_since_epoch());
    const auto whole_seconds = std::chrono::duration_cast<
        std::chrono::seconds>(milliseconds);
    const int fraction = static_cast<int>((milliseconds - whole_seconds).count());
    const std::time_t value = std::chrono::system_clock::to_time_t(timestamp);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &value);
#else
    gmtime_r(&value, &utc);
#endif
    std::ostringstream output;
    output << '[' << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.'
           << std::setw(3) << std::setfill('0') << fraction << "Z] ["
           << LogLevelName(level) << "] [" << module << "] " << message
           << '\n';
    return output.str();
}

bool ApplyModuleLogLevel(std::string_view specification,
                         LoggerConfig* config,
                         std::string* error) noexcept {
    if (config == nullptr) {
        SetError(error, "logger config is null");
        return false;
    }
    try {
        const std::size_t separator = specification.find('=');
        if (separator == std::string_view::npos || separator == 0U ||
            separator + 1U >= specification.size()) {
            SetError(error, "module override must be MODULE=LEVEL");
            return false;
        }
        const std::string module = Trim(std::string(specification.substr(
            0U, separator)));
        LogLevel level = LogLevel::OFF;
        if (!IsValidModule(module) ||
            !ParseLogLevel(specification.substr(separator + 1U), &level)) {
            SetError(error, "invalid module log override: " +
                                std::string(specification));
            return false;
        }
        const auto existing = std::find_if(
            config->module_levels.begin(), config->module_levels.end(),
            [&](const ModuleLogLevel& item) { return item.module == module; });
        if (existing == config->module_levels.end()) {
            config->module_levels.push_back(ModuleLogLevel{module, level});
        } else {
            existing->level = level;
        }
        return true;
    } catch (const std::exception& exception) {
        SetError(error, exception.what());
        return false;
    } catch (...) {
        SetError(error, "unknown module override failure");
        return false;
    }
}

bool LoadLoggerConfigFile(const std::string& path,
                          const LoggerConfig& base,
                          LoggerConfig* loaded,
                          std::string* error) noexcept {
    if (loaded == nullptr) {
        SetError(error, "loaded logger config is null");
        return false;
    }
    try {
        std::ifstream input(path);
        if (!input.is_open()) {
            SetError(error, "cannot open log config: " + path);
            return false;
        }
        LoggerConfig result = base;
        std::string line;
        std::size_t line_number = 0U;
        while (std::getline(input, line)) {
            ++line_number;
            line = Trim(std::move(line));
            if (line.empty() || line.front() == '#') continue;
            const std::size_t separator = line.find('=');
            if (separator == std::string::npos) {
                SetError(error, "invalid log config line " +
                                    std::to_string(line_number));
                return false;
            }
            const std::string key = Trim(line.substr(0U, separator));
            const std::string value = Trim(line.substr(separator + 1U));
            if (key == "level") {
                LogLevel level = LogLevel::OFF;
                if (!ParseLogLevel(value, &level)) {
                    SetError(error, "invalid level on log config line " +
                                        std::to_string(line_number));
                    return false;
                }
                result.default_level = level;
            } else if (key.rfind("module.", 0U) == 0U) {
                if (!ApplyModuleLogLevel(key.substr(7U) + '=' + value,
                                         &result, error)) {
                    return false;
                }
            } else {
                SetError(error, "unknown log config key on line " +
                                    std::to_string(line_number) + ": " + key);
                return false;
            }
        }
        if (!input.eof() && input.fail()) {
            SetError(error, "failed while reading log config: " + path);
            return false;
        }
        *loaded = std::move(result);
        return true;
    } catch (const std::exception& exception) {
        SetError(error, exception.what());
        return false;
    } catch (...) {
        SetError(error, "unknown log config read failure");
        return false;
    }
}

AsyncLogger& GlobalLogger() {
    static AsyncLogger logger;
    return logger;
}

bool ConfigureGlobalLogger(const LoggerConfig& config,
                           std::string* error) noexcept {
    return GlobalLogger().Reconfigure(config, error);
}

bool FlushGlobalLogger(std::chrono::milliseconds timeout) noexcept {
    return GlobalLogger().Flush(timeout);
}

void ShutdownGlobalLogger() noexcept { GlobalLogger().Shutdown(); }

LoggerSnapshot GlobalLoggerSnapshot() noexcept {
    return GlobalLogger().Snapshot();
}

}  // namespace visionarm::logging
