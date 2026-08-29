#include "runtime/runtime_app.h"

#include "runtime_internal.h"

#include <csignal>

namespace visionarm::runtime::detail {

std::atomic<bool> g_stop{false};
std::atomic<bool> g_reload_log_config{false};

void SignalHandler(int signal_number) {
#if defined(SIGHUP)
    if (signal_number == SIGHUP) {
        g_reload_log_config.store(true, std::memory_order_release);
        return;
    }
#endif
    g_stop.store(true, std::memory_order_release);
}

}  // namespace visionarm::runtime::detail

void visionarm::runtime::InstallSignalHandlers() noexcept {
    std::signal(SIGINT, detail::SignalHandler);
    std::signal(SIGTERM, detail::SignalHandler);
#if defined(SIGHUP)
    std::signal(SIGHUP, detail::SignalHandler);
#endif
}

int visionarm::runtime::RuntimeMain(int argc, char** argv) {
    return detail::RunRuntime(argc, argv);
}
