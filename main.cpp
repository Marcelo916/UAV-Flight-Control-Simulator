#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include <atomic>

#include "Sensor.h"
#include "Controller.h"

// Thread 1: simulates environmental noise/disturbance
void sensorNoiseLoop(Sensor& sensor, std::atomic<bool>& runFlag) {
    std::mt19937 rng(std::random_device{}());
    std::normal_distribution<double> noise(0.0, 0.4); // mean 0, stddev 0.4

    while (runFlag.load()) {
        sensor.addNoise(noise(rng));                      // small random drift
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

// Thread 2: control loop (periodic)
void controllerLoop(Sensor& sensor, Controller& ctrl, std::atomic<bool>& runFlag) {
    using namespace std::chrono;
    const auto period = 100ms; // 10Hz control
    auto next = steady_clock::now();

    size_t tick = 0;
    while (runFlag.load()) {
        next += period;

        auto [alt, thr] = ctrl.step(sensor);

        // Log every 5 cycles ~0.5s
        if ((tick++ % 5) == 0) {
            std::cout << "[CTRL] target=" << ctrl.target()
                      << " alt=" << alt
                      << " thr=" << thr
                      << " -> newAlt=" << sensor.readAltitude()
                      << std::endl;
        }

        std::this_thread::sleep_until(next);
    }
}

int main() {
    std::cout << "Mini UAV Flight Control Simulator (C++17)\n";

    Sensor sensor(980.0);         // start a bit below target
    Controller controller(1000.0, /*Kp=*/0.03, /*maxRate=*/2.0);

    std::atomic<bool> runFlag{true};

    std::thread tNoise(sensorNoiseLoop, std::ref(sensor), std::ref(runFlag));
    std::thread tCtrl(controllerLoop, std::ref(sensor), std::ref(controller), std::ref(runFlag));

    // Run for ~15 seconds as a demo
    std::this_thread::sleep_for(std::chrono::seconds(15));
    runFlag.store(false);

    tNoise.join();
    tCtrl.join();

    std::cout << "Final altitude: " << sensor.readAltitude() << "\n";
    std::cout << "Done.\n";
    return 0;
}
