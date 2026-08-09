#include "uart/serial_port.h"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace visionarm::uart {
namespace {

void SetErrnoError(std::string* error, const char* operation) noexcept {
    if (error == nullptr) {
        return;
    }
    *error = std::string(operation) + ": " + std::strerror(errno);
}

void SetError(std::string* error, const char* message) noexcept {
    if (error != nullptr) {
        *error = message;
    }
}

}  // namespace

SerialPort::~SerialPort() {
    Close();
}

bool SerialPort::BaudToSpeed(uint32_t baud_rate, speed_t* speed) noexcept {
    if (speed == nullptr) {
        return false;
    }
    switch (baud_rate) {
        case 9600U: *speed = B9600; return true;
        case 19200U: *speed = B19200; return true;
        case 38400U: *speed = B38400; return true;
        case 57600U: *speed = B57600; return true;
        case 115200U: *speed = B115200; return true;
#ifdef B230400
        case 230400U: *speed = B230400; return true;
#endif
#ifdef B460800
        case 460800U: *speed = B460800; return true;
#endif
#ifdef B921600
        case 921600U: *speed = B921600; return true;
#endif
        default: return false;
    }
}

bool SerialPort::Open(const SerialPortConfig& config,
                      std::string* error) noexcept {
    Close();

    if (config.device_path.empty()) {
        SetError(error, "serial device path is empty");
        return false;
    }

    speed_t speed = B0;
    if (!BaudToSpeed(config.baud_rate, &speed)) {
        SetError(error, "unsupported serial baud rate");
        return false;
    }

    const int fd = ::open(config.device_path.c_str(),
                          O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        SetErrnoError(error, "open");
        return false;
    }

    if (config.exclusive && ::ioctl(fd, TIOCEXCL) != 0) {
        SetErrnoError(error, "TIOCEXCL");
        ::close(fd);
        return false;
    }

    termios current{};
    if (::tcgetattr(fd, &current) != 0) {
        SetErrnoError(error, "tcgetattr");
        ::close(fd);
        return false;
    }

    original_termios_ = current;
    have_original_termios_ = true;

    ::cfmakeraw(&current);
    current.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    current.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    current.c_cflag |= CS8;
    current.c_cflag &= static_cast<tcflag_t>(~PARENB);
    current.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
#ifdef CRTSCTS
    current.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    current.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    current.c_cc[VMIN] = 0;
    current.c_cc[VTIME] = 0;

    if (::cfsetispeed(&current, speed) != 0 ||
        ::cfsetospeed(&current, speed) != 0) {
        SetErrnoError(error, "cfsetispeed/cfsetospeed");
        ::close(fd);
        have_original_termios_ = false;
        return false;
    }

    if (::tcsetattr(fd, TCSANOW, &current) != 0) {
        SetErrnoError(error, "tcsetattr");
        ::close(fd);
        have_original_termios_ = false;
        return false;
    }

    if (config.flush_on_open && ::tcflush(fd, TCIOFLUSH) != 0) {
        SetErrnoError(error, "tcflush");
        ::tcsetattr(fd, TCSANOW, &original_termios_);
        ::close(fd);
        have_original_termios_ = false;
        return false;
    }

    fd_ = fd;
    device_path_ = config.device_path;
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

void SerialPort::Close() noexcept {
    if (fd_ < 0) {
        return;
    }
    if (have_original_termios_) {
        (void)::tcsetattr(fd_, TCSANOW, &original_termios_);
    }
    (void)::close(fd_);
    fd_ = -1;
    device_path_.clear();
    have_original_termios_ = false;
    original_termios_ = termios{};
}

ssize_t SerialPort::Read(uint8_t* data,
                         std::size_t capacity,
                         std::string* error) noexcept {
    if (fd_ < 0 || data == nullptr || capacity == 0U) {
        errno = EINVAL;
        SetErrnoError(error, "read arguments");
        return -1;
    }
    const ssize_t result = ::read(fd_, data, capacity);
    if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        SetErrnoError(error, "read");
    } else if (error != nullptr) {
        error->clear();
    }
    return result;
}

ssize_t SerialPort::Write(const uint8_t* data,
                          std::size_t size,
                          std::string* error) noexcept {
    if (fd_ < 0 || data == nullptr || size == 0U) {
        errno = EINVAL;
        SetErrnoError(error, "write arguments");
        return -1;
    }
    const ssize_t result = ::write(fd_, data, size);
    if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        SetErrnoError(error, "write");
    } else if (error != nullptr) {
        error->clear();
    }
    return result;
}

}  // namespace visionarm::uart
