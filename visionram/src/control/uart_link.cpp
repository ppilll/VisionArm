#include "control/uart_link.h"

#include "control/uart_protocol.h"
#include "control/uart_transport.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <mutex>
#include <random>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include <poll.h>
#include <sys/eventfd.h>
#include <sys/random.h>
#include <time.h>
#include <unistd.h>

namespace visionarm::uart {

template <typename T>
class LatestValueMailbox {
public:
    struct PutResult {
        bool accepted = false;
        bool replaced = false;
    };

    PutResult Put(T value) noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool replaced = value_.has_value();
        value_ = std::move(value);
        ++put_count_;
        if (replaced) {
            ++overwrite_count_;
        }
        return {true, replaced};
    }

    [[nodiscard]] std::optional<T> Take() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!value_.has_value()) {
            return std::nullopt;
        }
        std::optional<T> result = std::move(value_);
        value_.reset();
        ++take_count_;
        return result;
    }

    [[nodiscard]] bool HasValue() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return value_.has_value();
    }

    void Clear() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        value_.reset();
    }

    [[nodiscard]] uint64_t PutCount() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return put_count_;
    }

    [[nodiscard]] uint64_t TakeCount() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return take_count_;
    }

    [[nodiscard]] uint64_t OverwriteCount() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return overwrite_count_;
    }

private:
    mutable std::mutex mutex_;
    std::optional<T> value_;
    uint64_t put_count_ = 0U;
    uint64_t take_count_ = 0U;
    uint64_t overwrite_count_ = 0U;
};

namespace {

uint64_t MonotonicNs() noexcept {
    timespec value{};
    if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0U;
    }
    return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(value.tv_nsec);
}

uint64_t MonotonicMs() noexcept {
    return MonotonicNs() / 1000000ULL;
}

uint32_t GenerateNonzeroId() noexcept {
    uint32_t value = 0U;
    const ssize_t bytes = ::getrandom(&value, sizeof(value), GRND_NONBLOCK);
    if (bytes != static_cast<ssize_t>(sizeof(value)) || value == 0U) {
        try {
            std::random_device random;
            value = (static_cast<uint32_t>(random()) << 16U) ^
                    static_cast<uint32_t>(random()) ^
                    static_cast<uint32_t>(MonotonicNs()) ^
                    static_cast<uint32_t>(::getpid());
        } catch (...) {
            value = static_cast<uint32_t>(MonotonicNs()) ^
                    static_cast<uint32_t>(::getpid());
        }
    }
    return value == 0U ? 1U : value;
}

enum class ResponseKind : uint8_t {
    NONE = 0U,
    HELLO_ACK,
    STATUS,
    PONG,
    RELIABLE,
};

struct WaitContext {
    ResponseKind kind = ResponseKind::NONE;
    uint64_t deadline_ms = 0U;
    uint32_t expected_id = 0U;
};

struct TxFrame {
    std::vector<uint8_t> bytes;
    std::size_t offset = 0U;
    MessageType message_type = MessageType::HEARTBEAT;
    ResponseKind response_kind = ResponseKind::NONE;
    uint32_t expected_id = 0U;
    bool valid_control = false;
    bool reliable_retry = false;
};

struct PendingReliableRequest {
    MessageType message_type = MessageType::REMOTE_STOP_REQUEST;
    uint32_t transaction_id = 0U;
    uint16_t reason_code = 0U;
};

}  // namespace

const char* LinkStateName(LinkState state) noexcept {
    switch (state) {
        case LinkState::CLOSED: return "CLOSED";
        case LinkState::OPENING: return "OPENING";
        case LinkState::NEGOTIATING: return "NEGOTIATING";
        case LinkState::READY: return "READY";
        case LinkState::DEGRADED: return "DEGRADED";
        case LinkState::LINK_LOST: return "LINK_LOST";
        case LinkState::STOPPING: return "STOPPING";
        case LinkState::FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

class UartLink::Impl {
public:
    explicit Impl(UartLinkConfig config)
        : config_(std::move(config)),
          parser_(config_.parser_assembly_timeout_ms) {
        snapshot_.local_boot_id = GenerateNonzeroId();
    }

    ~Impl() {
        Stop();
    }

    bool Start(std::string* error) noexcept {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
        if (running_.load()) {
            if (error != nullptr) {
                *error = "UART link is already running";
            }
            return false;
        }
        if (config_.device_path.empty() || config_.max_control_rate_hz == 0U ||
            config_.response_timeout_ms == 0U ||
            config_.reliable_timeout_ms == 0U) {
            if (error != nullptr) {
                *error = "UART link configuration is invalid";
            }
            return false;
        }

        wake_fd_ = ::eventfd(0U, EFD_NONBLOCK | EFD_CLOEXEC);
        if (wake_fd_ < 0) {
            if (error != nullptr) {
                *error = std::string("eventfd: ") + std::strerror(errno);
            }
            return false;
        }

        stop_requested_.store(false);
        running_.store(true);
        try {
            thread_ = std::thread(&Impl::ThreadMain, this);
        } catch (const std::exception& exception) {
            running_.store(false);
            (void)::close(wake_fd_);
            wake_fd_ = -1;
            if (error != nullptr) {
                *error = exception.what();
            }
            return false;
        } catch (...) {
            running_.store(false);
            (void)::close(wake_fd_);
            wake_fd_ = -1;
            if (error != nullptr) {
                *error = "failed to create UART I/O thread";
            }
            return false;
        }

        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    void Stop() noexcept {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
        if (!running_.load()) {
            return;
        }
        SetState(LinkState::STOPPING);
        stop_requested_.store(true);
        Wake();
        if (thread_.joinable()) {
            thread_.join();
        }
        if (wake_fd_ >= 0) {
            (void)::close(wake_fd_);
            wake_fd_ = -1;
        }
        running_.store(false);
        SetState(LinkState::CLOSED);
    }

    bool SubmitLatestControl(const ControlUpdateInput& input) noexcept {
        if (!running_.load()) {
            return false;
        }
        const auto put = control_mailbox_.Put(input);
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.control_accepted;
            if (put.replaced) {
                ++snapshot_.metrics.control_overwritten;
            }
        }
        Wake();
        return put.accepted;
    }

    bool RequestRemoteStop(uint16_t reason_code,
                           uint32_t* transaction_id) noexcept {
        return QueueReliable(MessageType::REMOTE_STOP_REQUEST,
                             reason_code, transaction_id);
    }

    bool RequestClearRemoteStop(uint32_t* transaction_id) noexcept {
        return QueueReliable(MessageType::CLEAR_REMOTE_STOP,
                             0U, transaction_id);
    }

    bool RequestPing(uint32_t* ping_id) noexcept {
        if (!running_.load()) {
            return false;
        }
        std::lock_guard<std::mutex> lock(request_mutex_);
        if (ping_pending_) {
            return false;
        }
        const uint32_t id = NextNonzero(&next_ping_id_);
        ping_pending_ = true;
        pending_ping_id_ = id;
        if (ping_id != nullptr) {
            *ping_id = id;
        }
        Wake();
        return true;
    }

    UartModuleSnapshot GetSnapshot() const noexcept {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return snapshot_;
    }

    UartMetrics GetMetrics() const noexcept {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return snapshot_.metrics;
    }

    LinkState GetLinkState() const noexcept {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return snapshot_.state;
    }

    bool IsRunning() const noexcept {
        return running_.load();
    }

private:
    static uint32_t NextNonzero(uint32_t* value) noexcept {
        ++(*value);
        if (*value == 0U) {
            ++(*value);
        }
        return *value;
    }

    bool QueueReliable(MessageType type,
                       uint16_t reason_code,
                       uint32_t* transaction_id) noexcept {
        if (!running_.load()) {
            return false;
        }
        std::lock_guard<std::mutex> lock(request_mutex_);
        if (reliable_busy_ || pending_reliable_.has_value()) {
            return false;
        }
        const uint32_t id = NextNonzero(&next_transaction_id_);
        pending_reliable_ = PendingReliableRequest{type, id, reason_code};
        reliable_busy_ = true;
        {
            std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex_);
            snapshot_.reliable_busy = true;
            ++snapshot_.metrics.reliable_requests;
            if (type == MessageType::REMOTE_STOP_REQUEST) {
                ++snapshot_.metrics.remote_stop_requests;
            } else if (type == MessageType::CLEAR_REMOTE_STOP) {
                ++snapshot_.metrics.clear_stop_requests;
            }
        }
        if (transaction_id != nullptr) {
            *transaction_id = id;
        }
        Wake();
        return true;
    }

    void Wake() noexcept {
        if (wake_fd_ < 0) {
            return;
        }
        const uint64_t one = 1U;
        const ssize_t result = ::write(wake_fd_, &one, sizeof(one));
        (void)result;
    }

    void DrainWakeFd() noexcept {
        uint64_t value = 0U;
        while (::read(wake_fd_, &value, sizeof(value)) > 0) {
        }
    }

    void SetState(LinkState state) noexcept {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.state = state;
    }

    void SetLastError(std::string error) noexcept {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.last_error = std::move(error);
    }

    void ClearLastError() noexcept {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.last_error.clear();
    }

    uint32_t SenderBootId() const noexcept {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return snapshot_.local_boot_id;
    }

    uint32_t AllocateWireSequence() noexcept {
        const uint32_t sequence = next_wire_sequence_++;
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.next_wire_sequence = next_wire_sequence_;
        return sequence;
    }

    uint32_t SenderUptimeMs(uint64_t now_ms) const noexcept {
        return static_cast<uint32_t>(now_ms - start_ms_);
    }

    void ThreadMain() noexcept {
        start_ms_ = MonotonicMs();
        uint64_t next_open_ms = start_ms_;
        uint64_t next_hello_ms = start_ms_;
        uint64_t next_heartbeat_ms = start_ms_;
        uint64_t next_control_ms = start_ms_;
        uint64_t last_valid_rx_ms = 0U;
        bool have_valid_rx = false;

        while (!stop_requested_.load()) {
            const uint64_t now_ms = MonotonicMs();

            if (!serial_.IsOpen() && now_ms >= next_open_ms) {
                SetState(LinkState::OPENING);
                {
                    std::lock_guard<std::mutex> lock(snapshot_mutex_);
                    ++snapshot_.metrics.open_attempts;
                }
                SerialPortConfig serial_config;
                serial_config.device_path = config_.device_path;
                serial_config.baud_rate = config_.baud_rate;
                serial_config.exclusive = config_.exclusive;
                serial_config.flush_on_open = config_.flush_on_open;
                std::string error;
                if (serial_.Open(serial_config, &error)) {
                    parser_.Reset();
                    peer_sequence_.Reset();
                    tx_.reset();
                    ClearWait();
                    reliable_.Cancel();
                    SetState(LinkState::NEGOTIATING);
                    ClearLastError();
                    next_hello_ms = now_ms;
                    next_heartbeat_ms = now_ms + config_.heartbeat_period_ms;
                    next_control_ms = now_ms;
                    have_valid_rx = false;
                    {
                        std::lock_guard<std::mutex> lock(snapshot_mutex_);
                        ++snapshot_.metrics.open_successes;
                        if (ever_opened_) {
                            ++snapshot_.metrics.reopen_count;
                        }
                        snapshot_.peer_boot_id = 0U;
                        snapshot_.peer_boot_id_valid = false;
                    }
                    ever_opened_ = true;
                } else {
                    SetLastError(error);
                    next_open_ms = now_ms + config_.open_retry_ms;
                }
            }

            if (serial_.IsOpen()) {
                parser_.Tick(now_ms);
                CopyParserStats();
                HandleTimeouts(now_ms, &next_hello_ms, &next_heartbeat_ms,
                               &have_valid_rx, &last_valid_rx_ms);
                Schedule(now_ms, &next_hello_ms, &next_heartbeat_ms,
                         &next_control_ms);
            }

            pollfd descriptors[2]{};
            nfds_t count = 1U;
            descriptors[0].fd = wake_fd_;
            descriptors[0].events = POLLIN;
            if (serial_.IsOpen()) {
                descriptors[1].fd = serial_.Fd();
                descriptors[1].events = POLLIN;
                if (tx_.has_value()) {
                    descriptors[1].events |= POLLOUT;
                }
                count = 2U;
            }

            const int poll_result = ::poll(descriptors, count, 10);
            if (poll_result < 0) {
                if (errno != EINTR) {
                    std::lock_guard<std::mutex> lock(snapshot_mutex_);
                    ++snapshot_.metrics.poll_errors;
                    snapshot_.last_error = std::string("poll: ") +
                                           std::strerror(errno);
                }
                continue;
            }

            if ((descriptors[0].revents & POLLIN) != 0) {
                DrainWakeFd();
            }

            if (!serial_.IsOpen() || count < 2U) {
                continue;
            }

            if ((descriptors[1].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                Reconnect("serial poll disconnect", MonotonicMs(), &next_open_ms);
                continue;
            }

            if ((descriptors[1].revents & POLLIN) != 0) {
                if (!ReadAvailable(MonotonicMs(), &last_valid_rx_ms,
                                   &have_valid_rx)) {
                    Reconnect("serial read failure", MonotonicMs(), &next_open_ms);
                    continue;
                }
            }

            if ((descriptors[1].revents & POLLOUT) != 0 && tx_.has_value()) {
                if (!WriteAvailable(MonotonicMs())) {
                    Reconnect("serial write failure", MonotonicMs(), &next_open_ms);
                    continue;
                }
            }
        }

        serial_.Close();
        tx_.reset();
        wait_ = {};
        reliable_.Cancel();
        {
            std::lock_guard<std::mutex> lock(request_mutex_);
            pending_reliable_.reset();
            reliable_busy_ = false;
            ping_pending_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot_.reliable_busy = false;
            snapshot_.response_window_open = false;
        }
        control_mailbox_.Clear();
    }

    void Reconnect(const char* reason,
                   uint64_t now_ms,
                   uint64_t* next_open_ms) noexcept {
        serial_.Close();
        tx_.reset();
        wait_ = {};
        parser_.Reset();
        peer_sequence_.Reset();
        FailOutstandingReliable();
        SetState(LinkState::LINK_LOST);
        recovery_pending_ = true;
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.link_losses;
            snapshot_.last_error = reason;
            snapshot_.response_window_open = false;
            snapshot_.peer_boot_id_valid = false;
        }
        *next_open_ms = now_ms + config_.open_retry_ms;
    }

    bool ReadAvailable(uint64_t now_ms,
                       uint64_t* last_valid_rx_ms,
                       bool* have_valid_rx) noexcept {
        uint8_t buffer[512]{};
        for (;;) {
            std::string error;
            const ssize_t read_size = serial_.Read(buffer, sizeof(buffer), &error);
            if (read_size > 0) {
                {
                    std::lock_guard<std::mutex> lock(snapshot_mutex_);
                    snapshot_.metrics.rx_bytes +=
                        static_cast<uint64_t>(read_size);
                }
                std::vector<Frame> frames;
                parser_.Feed(buffer, static_cast<std::size_t>(read_size),
                             now_ms, &frames);
                CopyParserStats();
                for (const Frame& frame : frames) {
                    HandleFrame(frame, now_ms, last_valid_rx_ms, have_valid_rx);
                }
                continue;
            }
            if (read_size == 0) {
                return true;
            }
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return true;
            }
            {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                ++snapshot_.metrics.read_errors;
                snapshot_.last_error = error;
            }
            return false;
        }
    }

    bool WriteAvailable(uint64_t now_ms) noexcept {
        while (tx_.has_value() && tx_->offset < tx_->bytes.size()) {
            std::string error;
            const uint8_t* begin = tx_->bytes.data() + tx_->offset;
            const std::size_t remaining = tx_->bytes.size() - tx_->offset;
            const ssize_t written = serial_.Write(begin, remaining, &error);
            if (written > 0) {
                tx_->offset += static_cast<std::size_t>(written);
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                snapshot_.metrics.tx_bytes +=
                    static_cast<uint64_t>(written);
                continue;
            }
            if (written < 0 && errno == EINTR) {
                continue;
            }
            if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                return true;
            }
            {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                ++snapshot_.metrics.write_errors;
                snapshot_.last_error = error;
            }
            return false;
        }

        if (tx_.has_value() && tx_->offset == tx_->bytes.size()) {
            CompleteTransmit(now_ms);
        }
        return true;
    }

    void CompleteTransmit(uint64_t now_ms) noexcept {
        const TxFrame completed = std::move(*tx_);
        tx_.reset();

        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.tx_frames;
            switch (completed.message_type) {
                case MessageType::HELLO:
                    ++snapshot_.metrics.hello_sent;
                    break;
                case MessageType::HEARTBEAT:
                    ++snapshot_.metrics.heartbeat_sent;
                    break;
                case MessageType::PING:
                    ++snapshot_.metrics.ping_sent;
                    break;
                case MessageType::REMOTE_STOP_REQUEST:
                case MessageType::CLEAR_REMOTE_STOP:
                    ++snapshot_.metrics.reliable_sent;
                    if (completed.reliable_retry) {
                        ++snapshot_.metrics.reliable_retries;
                    }
                    break;
                case MessageType::CONTROL_UPDATE:
                    ++snapshot_.metrics.control_sent;
                    if (completed.valid_control) {
                        ++snapshot_.metrics.valid_control_sent;
                    } else {
                        ++snapshot_.metrics.invalid_control_sent;
                    }
                    break;
                case MessageType::HELLO_ACK:
                case MessageType::STATUS:
                case MessageType::ACK:
                case MessageType::NACK:
                case MessageType::PONG:
                    break;
            }
        }

        if (completed.response_kind != ResponseKind::NONE) {
            wait_.kind = completed.response_kind;
            wait_.expected_id = completed.expected_id;
            wait_.deadline_ms = now_ms +
                (completed.response_kind == ResponseKind::RELIABLE
                     ? config_.reliable_timeout_ms
                     : config_.response_timeout_ms);
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot_.response_window_open = true;
            ++snapshot_.metrics.response_windows_opened;
        }
    }

    void HandleFrame(const Frame& frame,
                     uint64_t now_ms,
                     uint64_t* last_valid_rx_ms,
                     bool* have_valid_rx) noexcept {
        Message message;
        std::string error;
        if (!DecodeMessage(frame, &message, &error)) {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.message_decode_errors;
            snapshot_.last_error = error;
            return;
        }

        const bool had_peer_baseline = peer_sequence_.HasBaseline();
        const uint32_t previous_peer_sequence = peer_sequence_.LastSequence();
        const bool boot_changed = had_peer_baseline &&
            peer_sequence_.BootId() != frame.header.sender_boot_id;
        const SequenceRelation relation = peer_sequence_.Observe(
            frame.header.sender_boot_id, frame.header.wire_sequence);
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot_.peer_boot_id = frame.header.sender_boot_id;
            snapshot_.peer_boot_id_valid = true;
            if (boot_changed) {
                ++snapshot_.metrics.peer_boot_changes;
            }
            if (relation == SequenceRelation::DUPLICATE) {
                ++snapshot_.metrics.wire_duplicates;
            } else if (relation == SequenceRelation::OLDER) {
                ++snapshot_.metrics.wire_old_sequences;
            } else if (relation == SequenceRelation::NEWER_GAP &&
                       had_peer_baseline && !boot_changed) {
                const uint32_t delta =
                    frame.header.wire_sequence - previous_peer_sequence;
                snapshot_.metrics.wire_gaps +=
                    static_cast<uint64_t>(delta - 1U);
            }
        }
        if (boot_changed) {
            ClearWait();
            tx_.reset();
            FailOutstandingReliable();
            SetState(LinkState::NEGOTIATING);
        }
        if (relation == SequenceRelation::DUPLICATE ||
            relation == SequenceRelation::OLDER) {
            return;
        }

        *last_valid_rx_ms = now_ms;
        *have_valid_rx = true;

        switch (message.header.message_type) {
            case MessageType::HELLO_ACK:
                HandleHelloAck(std::get<HelloAck>(message.body), now_ms);
                break;
            case MessageType::STATUS:
                HandleStatus(std::get<Status>(message.body), now_ms);
                break;
            case MessageType::PONG:
                HandlePong(std::get<Pong>(message.body), now_ms);
                break;
            case MessageType::ACK:
                HandleAck(std::get<Ack>(message.body), true, now_ms);
                break;
            case MessageType::NACK:
                HandleNack(std::get<Nack>(message.body), now_ms);
                break;
            case MessageType::HELLO:
            case MessageType::HEARTBEAT:
            case MessageType::CONTROL_UPDATE:
            case MessageType::REMOTE_STOP_REQUEST:
            case MessageType::CLEAR_REMOTE_STOP:
            case MessageType::PING:
                CountUnexpectedResponse();
                break;
        }
    }

    void HandleHelloAck(const HelloAck& ack, uint64_t now_ms) noexcept {
        if (wait_.kind != ResponseKind::HELLO_ACK ||
            ack.hello_wire_sequence != wait_.expected_id ||
            ack.accepted == 0U ||
            ack.selected_protocol_version != kProtocolVersion ||
            ack.device_role != config_.expected_mcu_device_role ||
            ack.max_payload < kMaxPayloadSize) {
            CountUnexpectedResponse();
            return;
        }
        ClearWait();
        SetState(LinkState::READY);
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.hello_ack_received;
            if (recovery_pending_) {
                ++snapshot_.metrics.link_recoveries;
            }
            snapshot_.last_error.clear();
        }
        recovery_pending_ = false;
        last_ready_ms_ = now_ms;
    }

    void HandleStatus(const Status& status, uint64_t now_ms) noexcept {
        if (wait_.kind != ResponseKind::STATUS) {
            CountUnexpectedResponse();
            return;
        }
        ClearWait();
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot_.last_status = status;
            ++snapshot_.metrics.status_received;
        }
        SetState(LinkState::READY);
        last_ready_ms_ = now_ms;
    }

    void HandlePong(const Pong& pong, uint64_t now_ms) noexcept {
        if (wait_.kind != ResponseKind::PONG ||
            pong.ping_id != wait_.expected_id) {
            CountUnexpectedResponse();
            return;
        }
        ClearWait();
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.pong_received;
        }
        SetState(LinkState::READY);
        last_ready_ms_ = now_ms;
    }

    void HandleAck(const Ack& ack, bool acknowledged, uint64_t now_ms) noexcept {
        HandleReliableResponse(static_cast<MessageType>(ack.acked_message_type),
                               ack.transaction_id, acknowledged,
                               ack.result_code, ack.detail_code, now_ms);
    }

    void HandleNack(const Nack& nack, uint64_t now_ms) noexcept {
        HandleReliableResponse(static_cast<MessageType>(nack.acked_message_type),
                               nack.transaction_id, false,
                               nack.result_code, nack.detail_code, now_ms);
    }

    void HandleReliableResponse(MessageType type,
                                uint32_t transaction_id,
                                bool acknowledged,
                                uint16_t result_code,
                                uint16_t detail_code,
                                uint64_t now_ms) noexcept {
        if (wait_.kind != ResponseKind::RELIABLE) {
            CountUnexpectedResponse();
            return;
        }
        const ReliableResponse response = reliable_.HandleResponse(
            type, transaction_id, acknowledged, result_code, detail_code);
        if (!response.matched) {
            CountUnexpectedResponse();
            return;
        }
        ClearWait();
        ReleaseReliableBusy();
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            if (acknowledged) {
                ++snapshot_.metrics.reliable_acks;
            } else {
                ++snapshot_.metrics.reliable_nacks;
            }
        }
        SetState(LinkState::READY);
        last_ready_ms_ = now_ms;
    }

    void CountUnexpectedResponse() noexcept {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        ++snapshot_.metrics.unexpected_responses;
    }

    void ClearWait() noexcept {
        wait_ = {};
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.response_window_open = false;
    }

    void HandleTimeouts(uint64_t now_ms,
                        uint64_t* next_hello_ms,
                        uint64_t* next_heartbeat_ms,
                        bool* have_valid_rx,
                        uint64_t* last_valid_rx_ms) noexcept {
        if (wait_.kind != ResponseKind::NONE && now_ms >= wait_.deadline_ms) {
            const ResponseKind expired = wait_.kind;
            ClearWait();
            {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                ++snapshot_.metrics.response_window_timeouts;
                if (expired == ResponseKind::RELIABLE) {
                    ++snapshot_.metrics.reliable_timeouts;
                }
            }
            if (expired == ResponseKind::HELLO_ACK) {
                SetState(LinkState::NEGOTIATING);
                *next_hello_ms = now_ms;
            } else if (expired == ResponseKind::RELIABLE) {
                SetState(LinkState::DEGRADED);
            } else {
                SetState(LinkState::DEGRADED);
                *next_heartbeat_ms = now_ms + config_.heartbeat_period_ms;
            }
        }

        const LinkState state = GetLinkState();
        if ((state == LinkState::READY || state == LinkState::DEGRADED) &&
            *have_valid_rx && now_ms - *last_valid_rx_ms >=
                                  config_.link_watchdog_timeout_ms) {
            SetState(LinkState::LINK_LOST);
            tx_.reset();
            ClearWait();
            FailOutstandingReliable();
            peer_sequence_.Reset();
            *have_valid_rx = false;
            recovery_pending_ = true;
            {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                ++snapshot_.metrics.link_losses;
                snapshot_.peer_boot_id_valid = false;
            }
            *next_hello_ms = now_ms + 1U;
        }
    }

    void Schedule(uint64_t now_ms,
                  uint64_t* next_hello_ms,
                  uint64_t* next_heartbeat_ms,
                  uint64_t* next_control_ms) noexcept {
        if (tx_.has_value() || wait_.kind != ResponseKind::NONE ||
            !serial_.IsOpen()) {
            return;
        }

        const LinkState state = GetLinkState();

        if (state == LinkState::LINK_LOST) {
            if (now_ms >= *next_hello_ms) {
                SetState(LinkState::NEGOTIATING);
            }
            return;
        }

        if (state == LinkState::NEGOTIATING && now_ms >= *next_hello_ms) {
            SendHello(now_ms);
            *next_hello_ms = now_ms + config_.hello_period_ms;
            return;
        }

        if (state != LinkState::READY && state != LinkState::DEGRADED) {
            return;
        }

        const ReliablePollResult reliable_poll = reliable_.Poll(now_ms);
        if (reliable_poll.action == ReliablePollAction::RETRY &&
            reliable_poll.encoded_frame != nullptr) {
            TxFrame frame;
            frame.bytes = *reliable_poll.encoded_frame;
            frame.message_type = outstanding_reliable_type_;
            frame.response_kind = ResponseKind::RELIABLE;
            frame.expected_id = reliable_.TransactionId();
            frame.reliable_retry = true;
            tx_ = std::move(frame);
            return;
        }
        if (reliable_poll.action == ReliablePollAction::FAILED) {
            ReleaseReliableBusy();
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.reliable_retry_exhausted;
            ++snapshot_.metrics.reliable_failures;
        }

        if (!reliable_.IsOutstanding()) {
            std::optional<PendingReliableRequest> request;
            {
                std::lock_guard<std::mutex> lock(request_mutex_);
                if (pending_reliable_.has_value()) {
                    request = pending_reliable_;
                    pending_reliable_.reset();
                }
            }
            if (request.has_value()) {
                SendReliable(*request, now_ms);
                return;
            }
        }

        if (now_ms >= *next_heartbeat_ms) {
            SendHeartbeat(now_ms);
            *next_heartbeat_ms = now_ms + config_.heartbeat_period_ms;
            return;
        }

        uint32_t ping_id = 0U;
        bool send_ping = false;
        {
            std::lock_guard<std::mutex> lock(request_mutex_);
            if (ping_pending_) {
                send_ping = true;
                ping_id = pending_ping_id_;
                ping_pending_ = false;
            }
        }
        if (send_ping) {
            SendPing(ping_id, now_ms);
            return;
        }

        if (now_ms >= *next_control_ms) {
            std::optional<ControlUpdateInput> control = control_mailbox_.Take();
            if (control.has_value()) {
                SendControl(*control, now_ms);
                const uint64_t period = std::max<uint64_t>(
                    1U, 1000U / config_.max_control_rate_hz);
                *next_control_ms = now_ms + period;
            }
        }
    }

    Header MakeHeader(MessageType type, uint64_t now_ms) noexcept {
        Header header;
        header.protocol_version = kProtocolVersion;
        header.message_type = type;
        header.wire_sequence = AllocateWireSequence();
        header.sender_boot_id = SenderBootId();
        header.sender_uptime_ms = SenderUptimeMs(now_ms);
        return header;
    }

    bool StartTx(Message message,
                 ResponseKind response_kind,
                 uint32_t expected_id,
                 bool valid_control,
                 bool reliable_retry = false) noexcept {
        std::vector<uint8_t> encoded;
        std::string error;
        if (!EncodeMessage(message, &encoded, &error)) {
            SetLastError(error);
            return false;
        }
        TxFrame frame;
        frame.bytes = std::move(encoded);
        frame.message_type = message.header.message_type;
        frame.response_kind = response_kind;
        frame.expected_id = expected_id;
        frame.valid_control = valid_control;
        frame.reliable_retry = reliable_retry;
        tx_ = std::move(frame);
        return true;
    }

    void SendHello(uint64_t now_ms) noexcept {
        Hello hello;
        hello.device_role = config_.linux_device_role;
        hello.minimum_protocol_version = kProtocolVersion;
        hello.maximum_protocol_version = kProtocolVersion;
        hello.max_payload = static_cast<uint16_t>(kMaxPayloadSize);
        hello.max_control_rate_hz = static_cast<uint16_t>(
            std::min<uint32_t>(config_.max_control_rate_hz, 65535U));
        hello.capability_bits = config_.capability_bits;
        hello.software_version_major = config_.software_version_major;
        hello.software_version_minor = config_.software_version_minor;
        hello.software_version_patch = config_.software_version_patch;

        Header header = MakeHeader(MessageType::HELLO, now_ms);
        const uint32_t sequence = header.wire_sequence;
        (void)StartTx(Message{header, hello}, ResponseKind::HELLO_ACK,
                      sequence, false);
    }

    void SendHeartbeat(uint64_t now_ms) noexcept {
        Header header = MakeHeader(MessageType::HEARTBEAT, now_ms);
        (void)StartTx(Message{header, Heartbeat{}}, ResponseKind::STATUS,
                      header.wire_sequence, false);
    }

    void SendPing(uint32_t ping_id, uint64_t now_ms) noexcept {
        Header header = MakeHeader(MessageType::PING, now_ms);
        (void)StartTx(Message{header, Ping{ping_id}}, ResponseKind::PONG,
                      ping_id, false);
    }

    void SendReliable(const PendingReliableRequest& request,
                      uint64_t now_ms) noexcept {
        MessageBody body;
        if (request.message_type == MessageType::REMOTE_STOP_REQUEST) {
            body = RemoteStopRequest{request.transaction_id,
                                     request.reason_code};
        } else {
            body = ClearRemoteStop{request.transaction_id};
        }

        Header header = MakeHeader(request.message_type, now_ms);
        Message message{header, body};
        std::vector<uint8_t> encoded;
        std::string error;
        if (!EncodeMessage(message, &encoded, &error)) {
            SetLastError(error);
            ReleaseReliableBusy();
            return;
        }
        if (!reliable_.Begin(request.message_type, request.transaction_id,
                             header.wire_sequence, encoded, now_ms,
                             config_.reliable_timeout_ms,
                             config_.reliable_max_retries)) {
            SetLastError("failed to begin reliable transaction");
            ReleaseReliableBusy();
            return;
        }
        outstanding_reliable_type_ = request.message_type;

        TxFrame frame;
        frame.bytes = std::move(encoded);
        frame.message_type = request.message_type;
        frame.response_kind = ResponseKind::RELIABLE;
        frame.expected_id = request.transaction_id;
        tx_ = std::move(frame);
    }

    void SendControl(const ControlUpdateInput& input,
                     uint64_t now_ms) noexcept {
        bool valid = input.valid;
        const uint64_t now_ns = MonotonicNs();
        uint64_t age_ms = 65535U;
        if (input.capture_monotonic_ns != 0U &&
            now_ns >= input.capture_monotonic_ns) {
            age_ms = (now_ns - input.capture_monotonic_ns) / 1000000ULL;
        } else {
            valid = false;
        }

        const bool too_old = age_ms > config_.control_max_age_ms;
        if (too_old) {
            valid = false;
        }

        ControlUpdate control;
        control.source_capture_session_id = input.source_capture_session_id;
        control.source_frame_id = input.source_frame_id;
        control.source_v4l2_sequence = input.source_v4l2_sequence;
        control.target_state = input.target_state;
        control.capture_age_at_tx_ms = static_cast<uint16_t>(
            std::min<uint64_t>(age_ms, 65535U));

        if (valid) {
            control.control_flags = 0x01U;
            control.dx_px = input.dx_px;
            control.dy_px = input.dy_px;
            control.error_x_q15 = input.error_x_q15;
            control.error_y_q15 = input.error_y_q15;
            control.confidence_u16 = input.confidence_u16;
        }

        if (too_old) {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.control_invalidated_age;
        }

        Header header = MakeHeader(MessageType::CONTROL_UPDATE, now_ms);
        if (StartTx(Message{header, control}, ResponseKind::NONE, 0U, valid)) {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.control_encoded;
        }
    }

    void FailOutstandingReliable() noexcept {
        if (!reliable_.IsOutstanding()) {
            return;
        }
        reliable_.Cancel();
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            ++snapshot_.metrics.reliable_failures;
        }
        ReleaseReliableBusy();
    }

    void ReleaseReliableBusy() noexcept {
        {
            std::lock_guard<std::mutex> lock(request_mutex_);
            reliable_busy_ = false;
        }
        std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex_);
        snapshot_.reliable_busy = false;
    }

    void CopyParserStats() noexcept {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.metrics.parser = parser_.Stats();
        snapshot_.metrics.rx_valid_frames = snapshot_.metrics.parser.valid_frames;
    }

    UartLinkConfig config_;
    SerialPort serial_;
    FrameParser parser_;
    SequenceTracker peer_sequence_;
    ReliableTransaction reliable_;
    MessageType outstanding_reliable_type_ =
        MessageType::REMOTE_STOP_REQUEST;

    LatestValueMailbox<ControlUpdateInput> control_mailbox_;
    std::optional<TxFrame> tx_;
    WaitContext wait_;

    mutable std::mutex lifecycle_mutex_;
    mutable std::mutex snapshot_mutex_;
    mutable std::mutex request_mutex_;
    UartModuleSnapshot snapshot_;
    std::optional<PendingReliableRequest> pending_reliable_;
    bool reliable_busy_ = false;
    bool ping_pending_ = false;
    uint32_t pending_ping_id_ = 0U;
    uint32_t next_transaction_id_ = 0U;
    uint32_t next_ping_id_ = 0U;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread thread_;
    int wake_fd_ = -1;
    uint64_t start_ms_ = 0U;
    uint64_t last_ready_ms_ = 0U;
    uint32_t next_wire_sequence_ = 0U;
    bool ever_opened_ = false;
    bool recovery_pending_ = false;
};

UartLink::UartLink(UartLinkConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

UartLink::~UartLink() = default;

bool UartLink::Start(std::string* error) noexcept {
    return impl_->Start(error);
}

void UartLink::Stop() noexcept {
    impl_->Stop();
}

bool UartLink::SubmitLatestControl(const ControlUpdateInput& input) noexcept {
    return impl_->SubmitLatestControl(input);
}

bool UartLink::RequestRemoteStop(uint16_t reason_code,
                                 uint32_t* transaction_id) noexcept {
    return impl_->RequestRemoteStop(reason_code, transaction_id);
}

bool UartLink::RequestClearRemoteStop(uint32_t* transaction_id) noexcept {
    return impl_->RequestClearRemoteStop(transaction_id);
}

bool UartLink::RequestPing(uint32_t* ping_id) noexcept {
    return impl_->RequestPing(ping_id);
}

UartModuleSnapshot UartLink::GetSnapshot() const noexcept {
    return impl_->GetSnapshot();
}

UartMetrics UartLink::GetMetrics() const noexcept {
    return impl_->GetMetrics();
}

LinkState UartLink::GetLinkState() const noexcept {
    return impl_->GetLinkState();
}

bool UartLink::IsRunning() const noexcept {
    return impl_->IsRunning();
}

}  // namespace visionarm::uart
