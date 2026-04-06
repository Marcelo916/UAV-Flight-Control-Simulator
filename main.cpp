#include <iomanip>
#include <iostream>
#include <string>

#include "Simulation.h"

int main(int argc, char** argv) {
    std::cout << "Mini UAV Flight Control Simulator (C++17)\n";

    SimulationConfig config{};
    std::string csvPath;

    if (argc >= 2) {
        ScenarioKind scenario = ScenarioKind::Nominal;
        if (!parseScenario(argv[1], scenario)) {
            std::cerr << "Unknown scenario: " << argv[1] << "\n"
                      << "Valid scenarios: nominal, sensor_dropout, stuck_sensor, high_noise_burst, actuator_degradation\n";
            return 1;
        }
        config.scenario = scenario;
    }

    if (argc >= 3) {
        csvPath = argv[2];
    }

    const SimulationRunner runner;
    const SimulationResult result = runner.run(config);

    std::cout << "Scenario: " << scenarioName(config.scenario) << '\n';

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

    if (!csvPath.empty()) {
        if (writeTelemetryCsv(csvPath, result)) {
            std::cout << "Telemetry CSV written to: " << csvPath << '\n';
        } else {
            std::cerr << "Failed to write telemetry CSV to: " << csvPath << '\n';
            return 2;
        }
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
