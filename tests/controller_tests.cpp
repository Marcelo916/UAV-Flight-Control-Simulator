#include <cassert>
#include <cmath>
#include <iostream>

#include "Controller.h"

namespace {
constexpr double kEps = 1e-9;

bool nearlyEqual(double a, double b, double eps = kEps) {
    return std::fabs(a - b) <= eps;
}

void testZeroErrorGivesZeroThrottle() {
    Controller controller(1000.0, /*kp=*/0.03);

    const auto [error, throttle] = controller.evaluate(1000.0);

    assert(nearlyEqual(error, 0.0));
    assert(nearlyEqual(throttle, 0.0));
}

void testThrottleSaturatesPositiveAndNegative() {
    {
        Controller controller(1000.0, /*kp=*/0.05);
        const auto [error, throttle] = controller.evaluate(900.0);

        assert(nearlyEqual(error, 100.0));
        assert(nearlyEqual(throttle, 1.0));
    }

    {
        Controller controller(1000.0, /*kp=*/0.05);
        const auto [error, throttle] = controller.evaluate(1100.0);

        assert(nearlyEqual(error, -100.0));
        assert(nearlyEqual(throttle, -1.0));
    }
}

void testThrottleMatchesProportionalLawInsideLimits() {
    Controller controller(1000.0, /*kp=*/0.02);

    const auto [error, throttle] = controller.evaluate(990.0);

    assert(nearlyEqual(error, 10.0));
    assert(nearlyEqual(throttle, 0.2));
}
} // namespace

int main() {
    testZeroErrorGivesZeroThrottle();
    testThrottleSaturatesPositiveAndNegative();
    testThrottleMatchesProportionalLawInsideLimits();

    std::cout << "All controller tests passed.\n";
    return 0;
}
