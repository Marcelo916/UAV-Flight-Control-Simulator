#include "Sensor.h"

Sensor::Sensor(double initial_altitude) : altitude_(initial_altitude) {}

double Sensor::readAltitude() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return altitude_;
}

void Sensor::setAltitude(double alt) {
    std::lock_guard<std::mutex> lock(mtx_);
    altitude_ = alt;
}

void Sensor::addNoise(double delta) {
    std::lock_guard<std::mutex> lock(mtx_);
    altitude_ += delta;
}

void Sensor::applyPhysics(double verticalRate) {
    std::lock_guard<std::mutex> lock(mtx_);
    altitude_ += verticalRate;
}
