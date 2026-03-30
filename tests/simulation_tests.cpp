#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Plant.h"
#include "Simulation.h"

namespace {
constexpr double kEps = 1e-6;

bool nearlyEqual(double a, double b, double eps = kEps) {
    return std::fabs(a - b) <= eps;
}

SimulationConfig baselineConfig() {
    SimulationConfig config;
    config.initialAltitude = 980.0;
    config.initialVerticalVelocity = 0.0;
    config.targetAltitude = 1000.0;
    config.controllerGain = 0.05;
    config.gravity = 9.81;
    config.maxThrustAcceleration = 15.0;
    config.actuatorTimeConstant = 0.35;
    config.verticalDamping = 1.2;
    config.disturbanceSigma = 0.0;
    config.sensorNoiseSigma = 0.0;
    config.durationSeconds = 20.0;
    config.dtSeconds = 0.05;
    config.randomSeed = 7;
    return config;
}

void testPlantStateEvolvesFromAcceleration() {
    Plant plant(/*initial_altitude=*/1000.0,
                /*initial_vertical_velocity=*/0.0,
                /*gravity=*/9.81,
                /*max_thrust_acceleration=*/14.0,
                /*actuator_time_constant=*/0.0,
                /*min_thrust=*/0.0,
                /*max_thrust=*/1.0,
                /*hover_thrust=*/9.81 / 14.0,
                /*vertical_damping=*/0.0,
                /*disturbance_amplitude=*/0.0);

    const PlantState state = plant.step(/*control_command=*/0.2,
                                        /*disturbance_acceleration=*/0.0,
                                        /*dt_seconds=*/0.5);

    const double expectedActualThrust = (9.81 / 14.0) + 0.2;
    const double expectedAcceleration = (expectedActualThrust * 14.0) - 9.81;
    const double expectedVelocity = expectedAcceleration * 0.5;
    const double expectedAltitude = 1000.0 + (expectedVelocity * 0.5);

    assert(nearlyEqual(state.actualThrust, expectedActualThrust));
    assert(nearlyEqual(state.verticalVelocity, expectedVelocity));
    assert(nearlyEqual(state.altitude, expectedAltitude));
}

void testSaturationLimitsActualThrust() {
    Plant plant(/*initial_altitude=*/1000.0,
                /*initial_vertical_velocity=*/0.0,
                /*gravity=*/9.81,
                /*max_thrust_acceleration=*/15.0,
                /*actuator_time_constant=*/0.0,
                /*min_thrust=*/0.2,
                /*max_thrust=*/0.8,
                /*hover_thrust=*/0.5,
                /*vertical_damping=*/0.0,
                /*disturbance_amplitude=*/0.0);

    const PlantState high = plant.step(/*control_command=*/1.0, 0.0, 0.1);
    assert(nearlyEqual(high.actualThrust, 0.8));

    const PlantState low = plant.step(/*control_command=*/-1.0, 0.0, 0.1);
    assert(nearlyEqual(low.actualThrust, 0.2));
}

void testActuatorLagMovesThrustTowardCommand() {
    Plant plant(/*initial_altitude=*/1000.0,
                /*initial_vertical_velocity=*/0.0,
                /*gravity=*/9.81,
                /*max_thrust_acceleration=*/15.0,
                /*actuator_time_constant=*/0.5,
                /*min_thrust=*/0.0,
                /*max_thrust=*/1.0,
                /*hover_thrust=*/0.5,
                /*vertical_damping=*/0.0,
                /*disturbance_amplitude=*/0.0);

    const PlantState step1 = plant.step(/*control_command=*/0.4, 0.0, 0.1);
    const PlantState step2 = plant.step(/*control_command=*/0.4, 0.0, 0.1);

    assert(step1.actualThrust > 0.5);
    assert(step1.actualThrust < 0.9);
    assert(step2.actualThrust > step1.actualThrust);
    assert(step2.actualThrust < 0.9);
}

void testNominalClosedLoopConvergesReasonably() {
    SimulationConfig config = baselineConfig();
    SimulationRunner runner;
    const auto result = runner.run(config);

    assert(!result.samples.empty());
    assert(std::fabs(result.metrics.finalError) < 2.0);
    assert(std::fabs(result.metrics.finalError) < std::fabs(result.samples.front().error));
}

void testSensorDropoutScenarioProducesHeldMeasurements() {
    SimulationConfig config = baselineConfig();
    config.scenario = ScenarioKind::SensorDropout;

    SimulationRunner runner;
    const auto result = runner.run(config);

    bool observedHold = false;
    for (std::size_t i = 1; i < result.samples.size(); ++i) {
        const auto& prev = result.samples[i - 1];
        const auto& cur = result.samples[i];
        if (cur.timeSeconds >= 5.0 && cur.timeSeconds <= 8.0 &&
            nearlyEqual(cur.measuredAltitude, prev.measuredAltitude, 1e-9) &&
            !nearlyEqual(cur.trueAltitude, prev.trueAltitude, 1e-3)) {
            observedHold = true;
            break;
        }
    }

    assert(observedHold);
}

void testStuckSensorScenarioLocksMeasurement() {
    SimulationConfig config = baselineConfig();
    config.scenario = ScenarioKind::StuckSensor;

    SimulationRunner runner;
    const auto result = runner.run(config);

    bool seenAfterFault = false;
    double stuckValue = 0.0;

    for (const auto& sample : result.samples) {
        if (sample.timeSeconds >= 5.0 && !seenAfterFault) {
            stuckValue = sample.measuredAltitude;
            seenAfterFault = true;
        }

        if (seenAfterFault) {
            assert(nearlyEqual(sample.measuredAltitude, stuckValue, 1e-9));
        }
    }

    assert(seenAfterFault);
}

void testHighNoiseBurstIncreasesMeasurementError() {
    SimulationConfig baseline = baselineConfig();
    baseline.scenario = ScenarioKind::Nominal;

    SimulationConfig burst = baseline;
    burst.scenario = ScenarioKind::HighNoiseBurst;

    SimulationRunner runner;
    const auto baselineResult = runner.run(baseline);
    const auto burstResult = runner.run(burst);

    double baselineWindowError = 0.0;
    double burstWindowError = 0.0;
    int baselineCount = 0;
    int burstCount = 0;

    for (const auto& s : baselineResult.samples) {
        if (s.timeSeconds >= 5.0 && s.timeSeconds <= 8.0) {
            baselineWindowError += std::fabs(s.measuredAltitude - s.trueAltitude);
            ++baselineCount;
        }
    }

    for (const auto& s : burstResult.samples) {
        if (s.timeSeconds >= 5.0 && s.timeSeconds <= 8.0) {
            burstWindowError += std::fabs(s.measuredAltitude - s.trueAltitude);
            ++burstCount;
        }
    }

    assert(baselineCount > 0 && burstCount > 0);
    assert((burstWindowError / burstCount) > (baselineWindowError / baselineCount) + 1.0);
}

void testActuatorDegradationHurtsTracking() {
    SimulationConfig nominal = baselineConfig();
    nominal.scenario = ScenarioKind::Nominal;

    SimulationConfig degraded = baselineConfig();
    degraded.scenario = ScenarioKind::ActuatorDegradation;

    SimulationRunner runner;
    const auto nominalResult = runner.run(nominal);
    const auto degradedResult = runner.run(degraded);

    assert(degradedResult.metrics.meanAbsoluteError > nominalResult.metrics.meanAbsoluteError);
}

void testCsvExportWritesHeaderAndRows() {
    SimulationConfig config = baselineConfig();
    config.durationSeconds = 1.0;
    config.dtSeconds = 0.1;

    SimulationRunner runner;
    const auto result = runner.run(config);

    const std::string path = "telemetry_test.csv";
    const bool ok = writeTelemetryCsv(path, result);
    assert(ok);

    std::ifstream in(path);
    assert(in.good());

    std::string header;
    std::getline(in, header);
    assert(header.find("scenario,time_seconds") == 0);

    std::string firstDataLine;
    std::getline(in, firstDataLine);
    assert(!firstDataLine.empty());

    std::remove(path.c_str());
}

void testMetricsCaptureOvershootAndSettlingTime() {
    const std::vector<SimulationSample> samples{
        {0.0, 1000.0, 950.0, 950.0, 0.0, 50.0, 1.0, 0.7, 0.0},
        {1.0, 1000.0, 970.0, 970.0, 3.0, 30.0, 0.8, 0.75, 0.0},
        {2.0, 1000.0, 1005.0, 1005.0, 1.0, -5.0, -0.2, 0.6, 0.0},
        {3.0, 1000.0, 1000.5, 1000.5, 0.2, -0.5, -0.02, 0.58, 0.0},
    };

    const auto metrics = computeMetrics(samples, 1000.0, 0.02);

    assert(nearlyEqual(metrics.overshoot, 5.0));
    assert(nearlyEqual(metrics.finalAltitude, 1000.5));
    assert(nearlyEqual(metrics.finalError, -0.5));
    assert(nearlyEqual(metrics.settlingTimeSeconds, 2.0));
}
} // namespace

int main() {
    testPlantStateEvolvesFromAcceleration();
    testSaturationLimitsActualThrust();
    testActuatorLagMovesThrustTowardCommand();
    testNominalClosedLoopConvergesReasonably();
    testSensorDropoutScenarioProducesHeldMeasurements();
    testStuckSensorScenarioLocksMeasurement();
    testHighNoiseBurstIncreasesMeasurementError();
    testActuatorDegradationHurtsTracking();
    testCsvExportWritesHeaderAndRows();
    testMetricsCaptureOvershootAndSettlingTime();

    std::cout << "All simulation tests passed.\n";
    return 0;
}
