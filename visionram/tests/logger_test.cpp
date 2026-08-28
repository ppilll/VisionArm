#include "logging/logger.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "logger_test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

class BlockingBuffer final : public std::stringbuf {
public:
    void WaitUntilBlocked() {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_cv_.wait(lock, [this] { return entered_; });
    }

    void Release() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            released_ = true;
        }
        release_cv_.notify_all();
    }

protected:
    int_type overflow(int_type ch) override {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            entered_ = true;
            entered_cv_.notify_all();
            release_cv_.wait(lock, [this] { return released_; });
        }
        return std::stringbuf::overflow(ch);
    }

private:
    std::mutex mutex_;
    std::condition_variable entered_cv_;
    std::condition_variable release_cv_;
    bool entered_ = false;
    bool released_ = false;
};

void TestLevelFilterModuleOverrideAndOff() {
    std::ostringstream sink;
    visionarm::logging::LoggerConfig config;
    config.default_level = visionarm::logging::LogLevel::WARN;
    config.module_levels.push_back(
        {"pipeline", visionarm::logging::LogLevel::DEBUG});
    visionarm::logging::AsyncLogger logger(config, &sink);

    Require(!logger.IsEnabled(visionarm::logging::LogLevel::INFO, "camera"),
            "default WARN must filter INFO");
    Require(logger.IsEnabled(visionarm::logging::LogLevel::DEBUG, "pipeline"),
            "module DEBUG override must enable DEBUG");

    config.default_level = visionarm::logging::LogLevel::OFF;
    config.module_levels.clear();
    Require(logger.Reconfigure(config), "OFF reconfigure must succeed");
    Require(!logger.IsEnabled(visionarm::logging::LogLevel::FATAL, "pipeline"),
            "OFF must filter FATAL too");
    logger.Enqueue(visionarm::logging::LogLevel::FATAL, "pipeline", "hidden");
    Require(logger.Flush(), "empty logger must flush");
    Require(sink.str().empty(), "OFF mode must emit nothing");
}

void TestStableFormat() {
    const std::string formatted =
        visionarm::logging::AsyncLogger::FormatRecordForTest(
            std::chrono::system_clock::time_point{std::chrono::milliseconds(7)},
            visionarm::logging::LogLevel::INFO, "camera", "opened");
    Require(formatted ==
                "[1970-01-01T00:00:00.007Z] [INFO] [camera] opened\n",
            "formatted record must match the stable UTC schema");
}

void TestConfigReloadParser() {
    const std::string path = "logger_test_config.tmp";
    {
        std::ofstream output(path, std::ios::trunc);
        output << "# runtime control\n"
               << "level=ERROR\n"
               << "module.media=TRACE\n";
    }
    visionarm::logging::LoggerConfig base;
    base.default_level = visionarm::logging::LogLevel::INFO;
    visionarm::logging::LoggerConfig loaded;
    std::string error;
    Require(visionarm::logging::LoadLoggerConfigFile(
                path, base, &loaded, &error),
            "valid reload file must parse");
    std::remove(path.c_str());
    Require(loaded.default_level == visionarm::logging::LogLevel::ERROR,
            "reload must update the default level");
    Require(loaded.module_levels.size() == 1U &&
                loaded.module_levels.front().module == "media" &&
                loaded.module_levels.front().level ==
                    visionarm::logging::LogLevel::TRACE,
            "reload must update module override");
    std::ostringstream sink;
    visionarm::logging::AsyncLogger logger(base, &sink);
    Require(logger.Reconfigure(loaded, &error),
            "parsed dynamic config must apply without restart");
    Require(!logger.IsEnabled(visionarm::logging::LogLevel::WARN, "camera") &&
                logger.IsEnabled(
                    visionarm::logging::LogLevel::TRACE, "media"),
            "dynamic reload must atomically replace thresholds");
}

void TestConcurrentAccounting() {
    std::ostringstream sink;
    visionarm::logging::LoggerConfig config;
    config.default_level = visionarm::logging::LogLevel::TRACE;
    config.queue_capacity = 256U;
    visionarm::logging::AsyncLogger logger(config, &sink);
    constexpr int kThreads = 4;
    constexpr int kMessages = 500;
    std::vector<std::thread> producers;
    for (int thread = 0; thread < kThreads; ++thread) {
        producers.emplace_back([&logger, thread] {
            for (int index = 0; index < kMessages; ++index) {
                logger.Enqueue(visionarm::logging::LogLevel::DEBUG,
                               "concurrency",
                               std::to_string(thread) + ':' +
                                   std::to_string(index));
            }
        });
    }
    for (std::thread& producer : producers) producer.join();
    Require(logger.Flush(std::chrono::seconds(5)),
            "concurrent records must drain");
    const auto snapshot = logger.Snapshot();
    Require(snapshot.accepted + snapshot.dropped ==
                static_cast<std::uint64_t>(kThreads * kMessages),
            "every concurrent attempt must be accepted or counted as dropped");
    Require(snapshot.emitted == snapshot.accepted,
            "every accepted record must be emitted after flush");
    Require(snapshot.high_watermark <= config.queue_capacity,
            "normal queue high watermark must remain bounded");
}

void TestBoundedOverflowAndCriticalReserve() {
    BlockingBuffer buffer;
    std::ostream sink(&buffer);
    visionarm::logging::LoggerConfig config;
    config.default_level = visionarm::logging::LogLevel::TRACE;
    config.queue_capacity = 1U;
    config.critical_queue_capacity = 1U;
    visionarm::logging::AsyncLogger logger(config, &sink);

    logger.Enqueue(visionarm::logging::LogLevel::INFO, "test", "worker-hold");
    buffer.WaitUntilBlocked();
    logger.Enqueue(visionarm::logging::LogLevel::INFO, "test", "normal-queued");
    logger.Enqueue(visionarm::logging::LogLevel::INFO, "test", "normal-drop");
    logger.Enqueue(visionarm::logging::LogLevel::ERROR, "test", "error-queued");
    logger.Enqueue(visionarm::logging::LogLevel::FATAL, "test", "fatal-drop");
    const auto overloaded = logger.Snapshot();
    Require(overloaded.current_size <= 2U,
            "normal plus critical queues must remain bounded");
    Require(overloaded.dropped_overflow >= 2U,
            "overflowed records must be counted");
    Require(overloaded.dropped_critical >= 1U,
            "critical overflow must have a dedicated counter");
    buffer.Release();
    Require(logger.Flush(), "released logger must drain");
}

}  // namespace

int main() {
    TestLevelFilterModuleOverrideAndOff();
    TestStableFormat();
    TestConfigReloadParser();
    TestConcurrentAccounting();
    TestBoundedOverflowAndCriticalReserve();
    std::cout << "logger_test=PASS\n";
    return EXIT_SUCCESS;
}
