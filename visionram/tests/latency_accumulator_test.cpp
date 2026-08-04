#include "metrics/latency_accumulator.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}
}  // namespace

int main() {
    try {
        visionarm::LatencyAccumulator accumulator(4U);
        accumulator.Add(1'000'000LL);
        accumulator.Add(2'000'000LL);
        accumulator.Add(3'000'000LL);
        accumulator.Add(4'000'000LL);
        auto snapshot = accumulator.Snapshot();
        Require(snapshot.total_samples == 4U, "sample count mismatch");
        Require(!snapshot.truncated, "unexpected truncation");
        Require(snapshot.mean_ms == 2.5, "mean mismatch");
        Require(snapshot.maximum_ms == 4.0, "maximum mismatch");

        accumulator.Add(10'000'000LL);
        snapshot = accumulator.Snapshot();
        Require(snapshot.total_samples == 5U, "total sample mismatch");
        Require(snapshot.retained_samples == 4U, "retained count mismatch");
        Require(snapshot.truncated, "truncation flag missing");
        Require(snapshot.mean_ms == 4.0, "whole-run mean mismatch");
        Require(snapshot.maximum_ms == 10.0, "whole-run maximum mismatch");

        accumulator.Reset();
        snapshot = accumulator.Snapshot();
        Require(snapshot.total_samples == 0U, "reset failed");

        std::cout << "latency_accumulator_test PASSED\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "latency_accumulator_test FAILED: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
