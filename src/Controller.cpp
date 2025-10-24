#include "Controller.h"
#include <cmath>

Controller::Controller(double target_altitude, double kp, double max_rate)
  : targetAltitude_(target_altitude), Kp_(kp), maxRate_(max_rate) {}

void Controller::setTarget(double target) { targetAltitude_ = target; }
double Controller::target() const { return targetAltitude_; }

std::pair<double,double> Controller::step(Sensor& sensor) {
    // Read current altitude
    double alt = sensor.readAltitude();

    // Error (how far from target)
    double error = targetAltitude_ - alt;

    // Proportional control -> throttle in [-1, 1]
    double throttle = clamp(error * Kp_, -1.0, 1.0);

    // Map throttle to a vertical rate (units of altitude per step)
    double verticalRate = throttle * maxRate_;

    // Apply physics back to the sensor (very simple model)
    sensor.applyPhysics(verticalRate);

    return { alt, throttle };
}
