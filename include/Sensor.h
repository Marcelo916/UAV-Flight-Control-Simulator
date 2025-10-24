#pragma once
#include <mutex>

class Sensor {
public:
    explicit Sensor(double initial_altitude = 1000.0);

    // Thread-safe reads/writes
    double readAltitude() const;
    void setAltitude(double alt);
    void addNoise(double delta);
    void applyPhysics(double verticalRate); // altitude += verticalRate

private:
    mutable std::mutex mtx_;
    double altitude_;
};
