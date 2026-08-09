#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <termios.h>

namespace visionarm::uart {

struct SerialPortConfig {
    std::string device_path;
    uint32_t baud_rate = 115200U;
    bool exclusive = true;
    bool flush_on_open = true;
};

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    [[nodiscard]] bool Open(const SerialPortConfig& config,
                            std::string* error) noexcept;
    void Close() noexcept;

    [[nodiscard]] ssize_t Read(uint8_t* data,
                               std::size_t capacity,
                               std::string* error) noexcept;
    [[nodiscard]] ssize_t Write(const uint8_t* data,
                                std::size_t size,
                                std::string* error) noexcept;

    [[nodiscard]] bool IsOpen() const noexcept { return fd_ >= 0; }
    [[nodiscard]] int Fd() const noexcept { return fd_; }
    [[nodiscard]] const std::string& DevicePath() const noexcept {
        return device_path_;
    }

private:
    [[nodiscard]] static bool BaudToSpeed(uint32_t baud_rate,
                                          speed_t* speed) noexcept;

    int fd_ = -1;
    std::string device_path_;
    bool have_original_termios_ = false;
    termios original_termios_{};
};

}  // namespace visionarm::uart
