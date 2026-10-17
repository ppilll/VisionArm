#include "runtime/runtime_supervisor_client.h"

#include "control/uart_link.h"

#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <initializer_list>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace visionarm::runtime {
namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
constexpr std::size_t kHeaderBytes = 10U;
constexpr std::size_t kLegacyPayloadMax = 65536U;
constexpr std::size_t kHealthPayloadMax = 4096U;
constexpr std::size_t kOutputFramesMax = 2U;
constexpr unsigned kInputFramesPerTurn = 16U;

uint32_t ReadLE(const std::string& bytes, std::size_t at, unsigned count) {
    uint32_t value = 0U;
    for (unsigned i = 0U; i < count; ++i)
        value |= static_cast<uint32_t>(static_cast<unsigned char>(bytes[at + i])) << (8U * i);
    return value;
}

std::string Frame(uint16_t type, uint32_t request, const std::string& payload) {
    if (payload.size() > kHealthPayloadMax) throw std::length_error("managed payload too large");
    std::string bytes;
    auto put = [&](uint32_t value, unsigned count) {
        for (unsigned i = 0U; i < count; ++i)
            bytes.push_back(static_cast<char>(value >> (8U * i)));
    };
    put(static_cast<uint32_t>(payload.size()), 4U);
    put(type, 2U);
    put(request, 4U);
    return bytes + payload;
}

bool ValidUtf8(const std::string& text) {
    std::size_t i = 0U;
    while (i < text.size()) {
        const auto first = static_cast<unsigned char>(text[i++]);
        if (first < 0x80U) continue;
        unsigned count = 0U;
        uint32_t value = 0U, minimum = 0U;
        if (first >= 0xc2U && first <= 0xdfU) { count = 1U; value = first & 0x1fU; minimum = 0x80U; }
        else if (first >= 0xe0U && first <= 0xefU) { count = 2U; value = first & 0x0fU; minimum = 0x800U; }
        else if (first >= 0xf0U && first <= 0xf4U) { count = 3U; value = first & 0x07U; minimum = 0x10000U; }
        else return false;
        if (text.size() - i < count) return false;
        while (count-- != 0U) {
            const auto next = static_cast<unsigned char>(text[i++]);
            if ((next & 0xc0U) != 0x80U) return false;
            value = (value << 6U) | (next & 0x3fU);
        }
        if (value < minimum || value > 0x10ffffU || (value >= 0xd800U && value <= 0xdfffU)) return false;
    }
    return true;
}

std::string Quote(const std::string& text) {
    if (!ValidUtf8(text)) throw std::invalid_argument("managed text is not UTF-8");
    constexpr char hex[] = "0123456789abcdef";
    std::string result = "\"";
    for (unsigned char ch : text) {
        if (ch == '"' || ch == '\\') { result += '\\'; result += static_cast<char>(ch); }
        else if (ch < 0x20U) { result += "\\u00"; result += hex[ch >> 4U]; result += hex[ch & 15U]; }
        else result += static_cast<char>(ch);
    }
    return result + '"';
}

struct Value { std::string text; bool quoted = false; };
using Object = std::map<std::string, Value>;

// Replies and SERVICE_STOP are flat, bounded objects. This parser accepts JSON
// string escaping, including surrogate pairs, but never nested objects/arrays,
// duplicate keys, floating values or unknown fields in a successful reply.
class ReplyParser {
public:
    explicit ReplyParser(const std::string& input) : input_(input) {}
    Object Parse() {
        Object result;
        Space(); Expect('{'); Space();
        if (Take('}')) { Finish(); return result; }
        for (;;) {
            const auto key = String(); Space(); Expect(':'); Space();
            Value value;
            if (Peek() == '"') { value.text = String(); value.quoted = true; }
            else {
                const auto start = at_;
                if (Take('-')) {}
                if (Peek() == '0') ++at_;
                else { if (Peek() < '1' || Peek() > '9') Bad(); while (Peek() >= '0' && Peek() <= '9') ++at_; }
                value.text = input_.substr(start, at_ - start);
            }
            if (result.size() >= 16U || !result.emplace(key, std::move(value)).second) Bad();
            Space(); if (Take('}')) break; Expect(','); Space();
        }
        Finish(); return result;
    }
private:
    [[noreturn]] static void Bad() { throw std::invalid_argument("invalid managed reply"); }
    char Peek() const { return at_ < input_.size() ? input_[at_] : '\0'; }
    bool Take(char ch) { if (Peek() != ch) return false; ++at_; return true; }
    void Expect(char ch) { if (!Take(ch)) Bad(); }
    void Space() { while (Peek() == ' ' || Peek() == '\n' || Peek() == '\r' || Peek() == '\t') ++at_; }
    void Finish() { Space(); if (at_ != input_.size()) Bad(); }
    uint32_t Hex4() {
        uint32_t result = 0U;
        for (unsigned i = 0U; i < 4U; ++i) {
            const char ch = Peek(); ++at_;
            result <<= 4U;
            if (ch >= '0' && ch <= '9') result |= static_cast<uint32_t>(ch - '0');
            else if (ch >= 'a' && ch <= 'f') result |= static_cast<uint32_t>(ch - 'a' + 10);
            else if (ch >= 'A' && ch <= 'F') result |= static_cast<uint32_t>(ch - 'A' + 10);
            else Bad();
        }
        return result;
    }
    static void AppendUtf8(std::string& out, uint32_t value) {
        if (value < 0x80U) out += static_cast<char>(value);
        else if (value < 0x800U) { out += static_cast<char>(0xc0U | (value >> 6U)); out += static_cast<char>(0x80U | (value & 63U)); }
        else if (value < 0x10000U) { out += static_cast<char>(0xe0U | (value >> 12U)); out += static_cast<char>(0x80U | ((value >> 6U) & 63U)); out += static_cast<char>(0x80U | (value & 63U)); }
        else { out += static_cast<char>(0xf0U | (value >> 18U)); out += static_cast<char>(0x80U | ((value >> 12U) & 63U)); out += static_cast<char>(0x80U | ((value >> 6U) & 63U)); out += static_cast<char>(0x80U | (value & 63U)); }
    }
    std::string String() {
        Expect('"'); std::string out;
        while (!Take('"')) {
            char ch = Peek(); if (static_cast<unsigned char>(ch) < 0x20U) Bad(); ++at_;
            if (ch != '\\') { out += ch; continue; }
            ch = Peek(); ++at_;
            switch (ch) {
                case '"': case '\\': case '/': out += ch; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    uint32_t value = Hex4();
                    if (value >= 0xd800U && value <= 0xdbffU) {
                        Expect('\\'); Expect('u'); const uint32_t low = Hex4();
                        if (low < 0xdc00U || low > 0xdfffU) Bad();
                        value = 0x10000U + ((value - 0xd800U) << 10U) + low - 0xdc00U;
                    } else if (value >= 0xdc00U && value <= 0xdfffU) Bad();
                    AppendUtf8(out, value); break;
                }
                default: Bad();
            }
        }
        if (!ValidUtf8(out)) Bad();
        return out;
    }
    const std::string& input_;
    std::size_t at_ = 0U;
};

const std::string& Field(const Object& object, const char* key, bool quoted = true) {
    const auto it = object.find(key);
    if (it == object.end() || it->second.quoted != quoted)
        throw std::invalid_argument("managed reply field missing or wrong type");
    return it->second.text;
}

void Keys(const Object& object, std::initializer_list<const char*> names) {
    if (object.size() != names.size()) throw std::invalid_argument("managed reply shape");
    for (const char* name : names) if (object.count(name) != 1U)
        throw std::invalid_argument("managed reply keys");
}

uint64_t Positive(const std::string& value) {
    uint64_t number = 0U;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), number);
    if (value.empty() || value.front() < '1' || value.front() > '9' ||
        parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || number == 0U)
        throw std::invalid_argument("managed reply numeric identity");
    return number;
}

bool Epoch(const std::string& value) {
    if (value.size() != 32U) return false;
    for (char ch : value) if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f'))) return false;
    return true;
}
}  // namespace

class RuntimeSupervisorClient::Impl {
public:
    Impl(RuntimeSupervisorConfig config, std::atomic<bool>& process_stop)
        : config_(std::move(config)), process_stop_(process_stop) {
        if (config_.service_socket.empty() || config_.service_socket.front() != '/' ||
            config_.service_socket.size() >= sizeof(sockaddr_un{}.sun_path) ||
            config_.service_socket.find('\0') != std::string::npos ||
            config_.service_name.empty() || config_.service_name.size() > 512U ||
            config_.service_name.find('\0') != std::string::npos)
            throw std::invalid_argument("invalid runtime service socket/name");
        (void)Quote(config_.service_name);
    }

    ~Impl() { Stop(); }
    void Start() {
        if (worker_.joinable()) throw std::logic_error("managed worker already started");
        worker_ = std::thread([this] {
            try { Loop(); }
            catch (...) {
                { std::lock_guard<std::mutex> lock(failure_mutex_); failure_ = std::current_exception(); }
                BeginShutdown();
                process_stop_.store(true, std::memory_order_release);
            }
            Disconnect();
        });
    }
    void AttachProjection(uart::UartLink& owner) {
        std::lock_guard<std::mutex> lock(owner_mutex_);
        if (owner_ != nullptr) throw std::logic_error("managed projection already attached");
        owner_ = &owner;
    }
    void BeginShutdown() noexcept {
        std::lock_guard<std::mutex> lock(send_mutex_);
        health_enabled_ = false;
    }
    void Stop() noexcept {
        BeginShutdown(); stop_.store(true, std::memory_order_release);
        if (worker_.joinable()) worker_.join();
    }
    void RethrowWorkerFailure() {
        std::exception_ptr error;
        { std::lock_guard<std::mutex> lock(failure_mutex_); error = failure_; }
        if (error) std::rethrow_exception(error);
    }

private:
    enum class Command { none, bind, report };
    struct Output { std::string bytes; std::size_t offset = 0U; bool health = false; };
    struct Pending {
        Command command = Command::none;
        uint32_t request = 0U;
        uint64_t sequence = 0U;
        bool issue = false;
        uart::UartHealthSample sample;
        Clock::time_point deadline;
    };

    uart::UartHealthProjection ReadOwner() {
        std::lock_guard<std::mutex> lock(owner_mutex_);
        return owner_ ? owner_->ReadHealthProjection() : uart::UartHealthProjection{};
    }
    void AckIssue(uint64_t revision) {
        std::lock_guard<std::mutex> lock(owner_mutex_);
        if (owner_) owner_->AckHealthIssueThrough(revision);
    }
    bool HealthEnabled() {
        std::lock_guard<std::mutex> lock(send_mutex_);
        return health_enabled_ && !process_stop_.load(std::memory_order_acquire);
    }
    uint32_t NextRequest() noexcept {
        ++request_counter_; if (request_counter_ == 0U) ++request_counter_;
        return request_counter_;
    }
    void Disconnect() noexcept {
        if (fd_ >= 0) (void)::close(fd_);
        fd_ = -1; connecting_ = false; bound_ = false;
        output_.clear(); input_.clear(); pending_ = {};
        epoch_.clear(); session_.clear(); sequence_ = 0U;
        last_report_.reset();
        reconnect_at_ = Clock::now() + 1s;
    }
    void Connect(Clock::time_point now) {
        fd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) { reconnect_at_ = now + 1s; return; }
        sockaddr_un address{}; address.sun_family = AF_UNIX;
        std::memcpy(address.sun_path, config_.service_socket.c_str(), config_.service_socket.size() + 1U);
        const auto length = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + config_.service_socket.size() + 1U);
        const int result = ::connect(fd_, reinterpret_cast<const sockaddr*>(&address), length);
        if (result == 0) QueueBind(now);
        else if (errno == EINPROGRESS) { connecting_ = true; connect_deadline_ = now + 1s; }
        else Disconnect(); // AF_UNIX EAGAIN is retried using a fresh socket.
    }
    void QueueBind(Clock::time_point now) {
        const uint32_t request = NextRequest();
        output_.push_back({Frame(11U, request, "{\"version\":1,\"op\":\"bind\",\"service_name\":" + Quote(config_.service_name) + "}"), 0U, true});
        pending_ = {}; pending_.command = Command::bind;
        pending_.request = request; pending_.deadline = now + 1s;
    }
    void QueueReport(Clock::time_point now) {
        // Never reuse cached current after an ACK. The owner may have recorded
        // a newer issue while the previous command was in flight.
        const auto projection = ReadOwner();
        const bool issue = projection.pending_issue.has_value();
        const auto sample = issue ? *projection.pending_issue : projection.current;
        if (!issue && last_report_ && uart::SameHealthSample(sample, *last_report_) && now < report_due_) return;
        if (sequence_ == std::numeric_limits<uint64_t>::max())
            throw std::overflow_error("managed report sequence exhausted");
        const uint64_t sequence = ++sequence_;
        const auto status = uart::HealthStatusName(sample.status);
        const auto reason = uart::HealthReasonName(sample.reason);
        const std::string boot = sample.peer_boot_id ? std::to_string(*sample.peer_boot_id) : "null";
        const std::string payload = "{\"version\":1,\"op\":\"report\",\"runtime_epoch\":" + Quote(epoch_) +
            ",\"session_id\":" + Quote(session_) + ",\"report_seq\":" + Quote(std::to_string(sequence)) +
            ",\"component\":\"mcu_link\",\"status\":" + Quote(status) + ",\"reason_code\":" + Quote(reason) +
            ",\"reason\":" + Quote(reason) + ",\"projection_revision\":" + Quote(std::to_string(sample.projection_revision)) +
            ",\"link_epoch\":" + Quote(std::to_string(sample.link_epoch)) + ",\"peer_boot_id\":" + boot +
            ",\"sample_kind\":" + Quote(issue ? "issue" : "current") + "}";
        const uint32_t request = NextRequest();
        output_.push_back({Frame(11U, request, payload), 0U, true});
        pending_ = {Command::report, request, sequence, issue, sample, now + 1s};
    }
    void QueueHeartbeat(Clock::time_point now) {
        // Independent of UART health, report ACK and report lease. It stays on
        // the bound service socket, using the original type4 JSON shape.
        output_.push_back({Frame(4U, 0U, "{\"service_name\":" + Quote(config_.service_name) +
            ",\"timestamp\":0}"), 0U, false});
        heartbeat_due_ = now + 1s;
    }
    bool Flush() {
        std::lock_guard<std::mutex> lock(send_mutex_);
        while (!output_.empty()) {
            auto& frame = output_.front();
            if (frame.health && (!health_enabled_ || process_stop_.load(std::memory_order_acquire))) {
                // Do not leave a partial command in a stream or await its ACK.
                return false;
            }
            const ssize_t count = ::send(fd_, frame.bytes.data() + frame.offset,
                frame.bytes.size() - frame.offset, MSG_NOSIGNAL | MSG_DONTWAIT);
            if (count > 0) {
                frame.offset += static_cast<std::size_t>(count);
                if (frame.offset == frame.bytes.size()) output_.pop_front();
            } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return true;
            else return false;
        }
        return true;
    }
    bool Handle(uint16_t type, uint32_t request, const std::string& payload) {
        const Object object = ReplyParser(payload).Parse();
        if (type == 5U) {
            Keys(object, {"event", "service_name"});
            if (request != 0U || Field(object, "event") != "SERVICE_STOP" ||
                Field(object, "service_name") != config_.service_name) return false;
            BeginShutdown(); process_stop_.store(true, std::memory_order_release);
            return true;
        }
        if (request == 0U || pending_.command == Command::none || request != pending_.request) return false;
        if (type == 255U) {
            Keys(object, {"error", "code"});
            (void)Field(object, "error");
            const auto& code = Field(object, "code", false);
            if (code == "1001") throw std::logic_error("managed service/component is not authorized");
            return false; // 1002/1003/1004: no ACK, fresh reconnect/rebind in 1s.
        }
        if (type != 11U || Field(object, "version", false) != "1") return false;
        if (pending_.command == Command::bind) {
            Keys(object, {"version", "op", "result", "runtime_epoch", "session_id", "service_name",
                          "instance_generation", "report_interval_ms", "lease_ms"});
            if (Field(object, "op") != "bind" || Field(object, "result") != "OK" ||
                Field(object, "service_name") != config_.service_name ||
                Field(object, "report_interval_ms", false) != "1000" || Field(object, "lease_ms", false) != "3500") return false;
            epoch_ = Field(object, "runtime_epoch"); if (!Epoch(epoch_)) return false;
            session_ = Field(object, "session_id"); (void)Positive(session_);
            (void)Positive(Field(object, "instance_generation")); // server evidence, never self-reported.
            bound_ = true; pending_ = {}; heartbeat_due_ = Clock::now(); report_due_ = Clock::now();
        } else {
            Keys(object, {"version", "op", "result", "session_id", "report_seq", "projection_revision"});
            if (Field(object, "op") != "report" || Field(object, "result") != "APPLIED" ||
                Field(object, "session_id") != session_ ||
                Field(object, "report_seq") != std::to_string(pending_.sequence) ||
                Field(object, "projection_revision") != std::to_string(pending_.sample.projection_revision)) return false;
            if (pending_.issue && HealthEnabled()) AckIssue(pending_.sample.projection_revision);
            last_report_ = pending_.sample;
            const bool was_issue = pending_.issue; pending_ = {};
            // Force a new coherent read after every ACK; only stable current
            // renewals wait a second. New pending issues always take precedence.
            report_due_ = Clock::now() + (was_issue ? 0ms : 1000ms);
        }
        return true;
    }
    bool Read() {
        unsigned frames = 0U;
        for (;;) {
            while (input_.size() >= kHeaderBytes && frames < kInputFramesPerTurn) {
                const uint32_t size = ReadLE(input_, 0U, 4U);
                const auto type = static_cast<uint16_t>(ReadLE(input_, 4U, 2U));
                const uint32_t request = ReadLE(input_, 6U, 4U);
                if (size > kLegacyPayloadMax || ((type == 11U || type == 255U || type == 5U) && size > kHealthPayloadMax)) return false;
                if (type != 11U && type != 255U && type != 5U) return false;
                if (input_.size() < kHeaderBytes + size) break;
                const auto payload = input_.substr(kHeaderBytes, size);
                input_.erase(0U, kHeaderBytes + size); ++frames;
                try { if (!Handle(type, request, payload)) return false; }
                catch (const std::invalid_argument&) { return false; }
            }
            if (frames >= kInputFramesPerTurn || process_stop_.load(std::memory_order_acquire)) return true;
            char buffer[4096];
            const ssize_t count = ::recv(fd_, buffer, sizeof(buffer), MSG_DONTWAIT);
            if (count > 0) {
                if (input_.size() + static_cast<std::size_t>(count) > kHeaderBytes + kLegacyPayloadMax) return false;
                input_.append(buffer, static_cast<std::size_t>(count));
            } else if (count == 0) return false;
            else if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) return true;
            else return false;
        }
    }
    void Loop() {
        while (!stop_.load(std::memory_order_acquire)) {
            if (process_stop_.load(std::memory_order_acquire)) BeginShutdown();
            const auto now = Clock::now();
            const bool enabled = HealthEnabled();
            if (fd_ < 0 && enabled && now >= reconnect_at_) Connect(now);
            if (fd_ >= 0 && connecting_ && now >= connect_deadline_) Disconnect();
            if (fd_ >= 0 && !connecting_) {
                if (!enabled && pending_.command != Command::none) {
                    // Cancel the stop-and-wait exchange without waiting for ACK.
                    // A partly queued frame must be cancelled by closing its socket.
                    if (!output_.empty()) Disconnect();
                    else pending_ = {};
                }
                if (enabled && pending_.command != Command::none && now >= pending_.deadline) Disconnect();
                if (fd_ >= 0 && bound_ && now >= heartbeat_due_ && output_.size() < kOutputFramesMax)
                    QueueHeartbeat(now);
                if (fd_ >= 0 && bound_ && enabled && pending_.command == Command::none && output_.size() < kOutputFramesMax)
                    QueueReport(now);
                if (fd_ >= 0 && !Flush()) Disconnect();
                // Also process already buffered complete frames after the fairness cap.
                if (fd_ >= 0 && !Read()) Disconnect();
            }
            pollfd descriptor{}; descriptor.fd = fd_; descriptor.events = POLLIN;
            if (connecting_ || !output_.empty()) descriptor.events |= POLLOUT;
            const int result = ::poll(&descriptor, 1U, 50); // bounded stop/join, projection check <=100ms.
            if (result < 0) { if (errno != EINTR) Disconnect(); continue; }
            if (result == 0 || fd_ < 0) continue;
            if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) { Disconnect(); continue; }
            if (connecting_ && (descriptor.revents & POLLOUT) != 0) {
                int error = 0; socklen_t length = sizeof(error);
                if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &length) != 0 || error != 0) Disconnect();
                else { connecting_ = false; QueueBind(Clock::now()); }
            }
        }
    }

    RuntimeSupervisorConfig config_;
    std::atomic<bool>& process_stop_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
    std::mutex owner_mutex_, send_mutex_, failure_mutex_;
    uart::UartLink* owner_ = nullptr; // owner declared before this client in composition.
    bool health_enabled_ = true;
    std::exception_ptr failure_;
    int fd_ = -1;
    bool connecting_ = false, bound_ = false;
    std::deque<Output> output_;
    std::string input_, epoch_, session_;
    uint32_t request_counter_ = 0U;
    uint64_t sequence_ = 0U;
    Pending pending_;
    std::optional<uart::UartHealthSample> last_report_;
    Clock::time_point reconnect_at_{}, connect_deadline_{}, heartbeat_due_{}, report_due_{};
};

RuntimeSupervisorClient::RuntimeSupervisorClient(RuntimeSupervisorConfig config,
                                                 std::atomic<bool>& process_stop)
    : impl_(std::make_unique<Impl>(std::move(config), process_stop)) {}
RuntimeSupervisorClient::~RuntimeSupervisorClient() = default;
void RuntimeSupervisorClient::Start() { impl_->Start(); }
void RuntimeSupervisorClient::AttachProjection(uart::UartLink& owner) { impl_->AttachProjection(owner); }
void RuntimeSupervisorClient::BeginShutdown() noexcept { impl_->BeginShutdown(); }
void RuntimeSupervisorClient::Stop() noexcept { impl_->Stop(); }
void RuntimeSupervisorClient::RethrowWorkerFailure() { impl_->RethrowWorkerFailure(); }

}  // namespace visionarm::runtime
