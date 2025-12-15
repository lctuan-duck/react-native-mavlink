#pragma once
#include "../core/MAVLinkState.hpp"
#include <mutex>

namespace margelo::nitro::mavlink {

class TelemetryManager {
public:
    TelemetryManager() = default;
    ~TelemetryManager() = default;
    
    void updateGPS(const GPSData& gps);
    void updateAttitude(const AttitudeData& attitude);
    void updateBattery(const BatteryData& battery);
    void updateHeartbeat(const HeartbeatData& heartbeat);
    
    TelemetryState getSnapshot() const;
    
private:
    mutable std::mutex mutex_;
    TelemetryState state_;
};

} // namespace margelo::nitro::mavlink
