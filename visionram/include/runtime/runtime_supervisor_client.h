#pragma once

#include <atomic>
#include <memory>
#include <string>

namespace visionarm::uart { class UartLink; }

namespace visionarm::runtime {

struct RuntimeSupervisorConfig {
    std::string service_socket;
    std::string service_name;
};

// One foreground-process UDS loop. No UART wire operations or process policy.
class RuntimeSupervisorClient {
public:
    RuntimeSupervisorClient(RuntimeSupervisorConfig config,
                            std::atomic<bool>& process_stop);
    ~RuntimeSupervisorClient();
    RuntimeSupervisorClient(const RuntimeSupervisorClient&) = delete;
    RuntimeSupervisorClient& operator=(const RuntimeSupervisorClient&) = delete;

    void Start();
    void AttachProjection(uart::UartLink& owner);
    void BeginShutdown() noexcept;
    void Stop() noexcept;
    void RethrowWorkerFailure();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace visionarm::runtime
