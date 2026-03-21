#include "Plant.h"

Plant::Plant(double initial_altitude,
             double initial_vertical_velocity,
             double gravity,
             double max_thrust_acceleration,
             double actuator_time_constant,
             double min_thrust,
             double max_thrust,
             double hover_thrust,
             double vertical_damping,
             double disturbance_amplitude)
  : state_{initial_altitude, initial_vertical_velocity, 0.0},
    gravity_(gravity),
    maxThrustAcceleration_(max_thrust_acceleration),
    actuatorTimeConstant_(actuator_time_constant),
    minThrust_(min_thrust),
    maxThrust_(max_thrust),
    hoverThrust_(hover_thrust < 0.0 ? gravity / max_thrust_acceleration : hover_thrust),
    verticalDamping_(vertical_damping),
    disturbanceAmplitude_(disturbance_amplitude) {
    hoverThrust_ = clamp(hoverThrust_, minThrust_, maxThrust_);
    state_.actualThrust = hoverThrust_;
}

double Plant::altitude() const { return state_.altitude; }
double Plant::verticalVelocity() const { return state_.verticalVelocity; }
double Plant::actualThrust() const { return state_.actualThrust; }
double Plant::gravity() const { return gravity_; }
double Plant::maxThrustAcceleration() const { return maxThrustAcceleration_; }
double Plant::actuatorTimeConstant() const { return actuatorTimeConstant_; }
double Plant::hoverThrust() const { return hoverThrust_; }
double Plant::verticalDamping() const { return verticalDamping_; }
double Plant::disturbanceAmplitude() const { return disturbanceAmplitude_; }
const PlantState& Plant::state() const { return state_; }

double Plant::commandToDesiredThrust(double control_command) const {
    return clamp(hoverThrust_ + control_command, minThrust_, maxThrust_);
}

PlantState Plant::step(double control_command, double disturbance_acceleration, double dt_seconds) {
    const double desiredThrust = commandToDesiredThrust(clamp(control_command, -1.0, 1.0));
    const double responseAlpha = actuatorTimeConstant_ > 0.0
        ? clamp(dt_seconds / actuatorTimeConstant_, 0.0, 1.0)
        : 1.0;

    state_.actualThrust += (desiredThrust - state_.actualThrust) * responseAlpha;
    state_.actualThrust = clamp(state_.actualThrust, minThrust_, maxThrust_);

    const double thrustAcceleration = state_.actualThrust * maxThrustAcceleration_;
    const double dragAcceleration = verticalDamping_ * state_.verticalVelocity;
    const double netAcceleration = thrustAcceleration - gravity_ - dragAcceleration + disturbance_acceleration;

    state_.verticalVelocity += netAcceleration * dt_seconds;
    state_.altitude += state_.verticalVelocity * dt_seconds;

    return state_;
}
