#include "TelemetryManager.hpp"
#include <chrono>

namespace margelo::nitro::mavlink {

void TelemetryManager::updateGPS(const GPSData& gps) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.gps = gps;
    state_.lastUpdate = std::chrono::system_clock::now();
}

void TelemetryManager::updateAttitude(const AttitudeData& attitude) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.attitude = attitude;
    state_.lastUpdate = std::chrono::system_clock::now();
}

void TelemetryManager::updateBattery(const BatteryData& battery) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.battery = battery;
    state_.lastUpdate = std::chrono::system_clock::now();
}

void TelemetryManager::updateHeartbeat(const HeartbeatData& heartbeat) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.heartbeat = heartbeat;
    state_.lastUpdate = std::chrono::system_clock::now();
}

TelemetryState TelemetryManager::getSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

} // namespace margelo::nitro::mavlink
