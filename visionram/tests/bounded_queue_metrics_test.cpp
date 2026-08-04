#include "pipeline/bounded_queue.h"

#include <cstdlib>
#include <iostream>
#include <optional>
#include <stdexcept>

namespace {
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main() {
    try {
        visionarm::BoundedQueue<int> queue(1U);
        std::optional<int> evicted;
        Require(queue.PushLatest(1, &evicted), "first push failed");
        Require(queue.PushLatest(2, &evicted), "second push failed");
        Require(evicted.has_value() && *evicted == 1,
                "latest replacement mismatch");
        queue.Stop();
        int output = 0;
        Require(queue.WaitPop(&output) && output == 2,
                "stopped queue must drain latest item");
        Require(!queue.WaitPop(&output),
                "stopped drained queue must return false");
        const auto snapshot = queue.Snapshot();
        Require(snapshot.pushed == 2U, "push metric mismatch");
        Require(snapshot.popped == 1U, "pop metric mismatch");
        Require(snapshot.replaced_oldest == 1U,
                "replacement metric mismatch");
        Require(snapshot.high_watermark == 1U,
                "high watermark mismatch");
        std::cout << "bounded_queue_metrics_test PASSED\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "bounded_queue_metrics_test FAILED: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
