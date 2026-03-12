#include <cassert>
#include <cmath>
#include <iostream>

#include "Controller.h"
#include "Sensor.h"

namespace {
constexpr double kEps = 1e-9;

bool nearlyEqual(double a, double b, double eps = kEps) {
    return std::fabs(a - b) <= eps;
}

void testZeroErrorGivesZeroThrottle() {
    Sensor sensor(1000.0);
    Controller controller(1000.0, /*kp=*/0.03, /*max_rate=*/2.0);

    auto [alt, throttle] = controller.step(sensor);

    assert(nearlyEqual(alt, 1000.0));
    assert(nearlyEqual(throttle, 0.0));
    assert(nearlyEqual(sensor.readAltitude(), 1000.0));
}

void testThrottleSaturatesPositiveAndNegative() {
    {
        Sensor sensor(900.0);
        Controller controller(1000.0, /*kp=*/0.05, /*max_rate=*/2.0);

        auto [alt, throttle] = controller.step(sensor);

        assert(nearlyEqual(alt, 900.0));
        assert(nearlyEqual(throttle, 1.0));
        assert(nearlyEqual(sensor.readAltitude(), 902.0));
    }

    {
        Sensor sensor(1100.0);
        Controller controller(1000.0, /*kp=*/0.05, /*max_rate=*/2.0);

        auto [alt, throttle] = controller.step(sensor);

        assert(nearlyEqual(alt, 1100.0));
        assert(nearlyEqual(throttle, -1.0));
        assert(nearlyEqual(sensor.readAltitude(), 1098.0));
    }
}

void testAltitudeUpdateMatchesControllerLogic() {
    Sensor sensor(990.0);
    Controller controller(1000.0, /*kp=*/0.02, /*max_rate=*/1.5);

    auto [alt, throttle] = controller.step(sensor);

    const double expectedThrottle = 0.2;  // (1000 - 990) * 0.02
    const double expectedAltitude = 990.0 + (expectedThrottle * 1.5);

    assert(nearlyEqual(alt, 990.0));
    assert(nearlyEqual(throttle, expectedThrottle));
    assert(nearlyEqual(sensor.readAltitude(), expectedAltitude));
}
} // namespace

int main() {
    testZeroErrorGivesZeroThrottle();
    testThrottleSaturatesPositiveAndNegative();
    testAltitudeUpdateMatchesControllerLogic();

    std::cout << "All controller tests passed.\n";
    return 0;
}
