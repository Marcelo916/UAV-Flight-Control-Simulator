#pragma once
#include <mutex>

class Sensor {
public:
    explicit Sensor(double initial_altitude = 1000.0);

    double readAltitude() const;
    void setAltitude(double alt);

private:
    mutable std::mutex mtx_;
    double altitude_;
};
