#pragma once

#include "Util.h"

struct PlantState {
    double altitude = 1000.0;
    double verticalVelocity = 0.0;
    double actualThrust = 0.0;
};

class Plant {
public:
    Plant(double initial_altitude = 1000.0,
          double initial_vertical_velocity = 0.0,
          double gravity = 9.81,
          double max_thrust_acceleration = 15.0,
          double actuator_time_constant = 0.35,
          double min_thrust = 0.0,
          double max_thrust = 1.0,
          double hover_thrust = -1.0,
          double vertical_damping = 1.2,
          double disturbance_amplitude = 0.4);

    double altitude() const;
    double verticalVelocity() const;
    double actualThrust() const;
    double gravity() const;
    double maxThrustAcceleration() const;
    double actuatorTimeConstant() const;
    double hoverThrust() const;
    double verticalDamping() const;
    double disturbanceAmplitude() const;
    double actuatorEffectiveness() const;

    void setActuatorEffectiveness(double effectiveness);

    const PlantState& state() const;
    PlantState step(double control_command, double disturbance_acceleration, double dt_seconds);

private:
    double commandToDesiredThrust(double control_command) const;

    PlantState state_;
    double gravity_;
    double maxThrustAcceleration_;
    double actuatorTimeConstant_;
    double minThrust_;
    double maxThrust_;
    double hoverThrust_;
    double verticalDamping_;
    double disturbanceAmplitude_;
    double actuatorEffectiveness_;
};
