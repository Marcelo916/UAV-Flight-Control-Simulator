#pragma once
#include <utility>

#include "Util.h"
#include "Sensor.h"

class Controller {
public:
    Controller(double target_altitude, double kp = 0.02, double max_rate = 1.5);

    // Reads sensor, computes throttle, applies effect to altitude (physics)
    // Returns tuple-like info (current alt, throttle used).
    std::pair<double,double> step(Sensor& sensor);

    void setTarget(double target);
    double target() const;

private:
    double targetAltitude_;
    double Kp_;        // proportional gain
    double maxRate_;   // max vertical rate per step (units per cycle)
};
