#include <iomanip>
#include <iostream>

#include "Simulation.h"

int main() {
    std::cout << "Mini UAV Flight Control Simulator (C++17)\n";

    const SimulationConfig config{};
    const SimulationRunner runner;
    const SimulationResult result = runner.run(config);

    for (std::size_t i = 0; i < result.samples.size(); i += 5) {
        const auto& sample = result.samples[i];
        std::cout << std::fixed << std::setprecision(2)
                  << "[CTRL] t=" << sample.timeSeconds << "s"
                  << " target=" << sample.targetAltitude
                  << " trueAlt=" << sample.trueAltitude
                  << " vz=" << sample.verticalVelocity
                  << " cmd=" << sample.controlCommand
                  << " thrust=" << sample.actualThrust
                  << " gustAcc=" << sample.disturbance
                  << '\n';
    }

    const auto& metrics = result.metrics;
    std::cout << "\nPerformance summary\n"
              << "  Final altitude:     " << metrics.finalAltitude << '\n'
              << "  Final error:        " << metrics.finalError << '\n'
              << "  Mean abs error:     " << metrics.meanAbsoluteError << '\n'
              << "  RMS error:          " << metrics.rmsError << '\n'
              << "  Max abs error:      " << metrics.maxAbsoluteError << '\n'
              << "  Overshoot:          " << metrics.overshoot << '\n'
              << "  Settling time (2%): " << metrics.settlingTimeSeconds << " s\n"
              << "Done.\n";
    return 0;
}
