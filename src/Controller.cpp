#include "Controller.h"

Controller::Controller(double target_altitude, double kp)
  : targetAltitude_(target_altitude), Kp_(kp) {}

void Controller::setTarget(double target) { targetAltitude_ = target; }
double Controller::target() const { return targetAltitude_; }
double Controller::gain() const { return Kp_; }

double Controller::computeThrottle(double measured_altitude) const {
    const double error = targetAltitude_ - measured_altitude;
    return clamp(error * Kp_, -1.0, 1.0);
}

std::pair<double, double> Controller::evaluate(double measured_altitude) const {
    const double error = targetAltitude_ - measured_altitude;
    return {error, computeThrottle(measured_altitude)};
}
