#pragma once
#include <utility>

#include "Util.h"

class Controller {
public:
    Controller(double target_altitude, double kp = 0.02);

    double computeThrottle(double measured_altitude) const;
    std::pair<double, double> evaluate(double measured_altitude) const;

    void setTarget(double target);
    double target() const;
    double gain() const;

private:
    double targetAltitude_;
    double Kp_;
};
