/**
 * VehicleState.hpp
 * Manages and stores the current state of a MAVLink vehicle
 * Based on QGroundControl Vehicle implementation
 */

#pragma once

#include <string>
#include <mutex>
#include <atomic>
#include "../mavlink/v2.0/common/mavlink.h"

namespace margelo::nitro::mavlink {

class VehicleState {
public:
    VehicleState();
    ~VehicleState() = default;

    // ============================================================================
    // Message Handlers
    // ============================================================================
    
    void handleHeartbeat(const mavlink_heartbeat_t& heartbeat);
    void handleGlobalPositionInt(const mavlink_global_position_int_t& position);
    void handleAttitude(const mavlink_attitude_t& attitude);
    void handleBatteryStatus(const mavlink_battery_status_t& battery);
    void handleGPSRawInt(const mavlink_gps_raw_int_t& gps);
    void handleVFRHUD(const mavlink_vfr_hud_t& vfr);
    void handleSysStatus(const mavlink_sys_status_t& sysStatus);
    
    // ============================================================================
    // Getters - Thread-safe
    // ============================================================================
    
    // Position & Navigation
    double getLatitude() const;
    double getLongitude() const;
    double getAltitude() const;
    double getAltitudeRelative() const;
    double getAltitudeAMSL() const;
    double getHeading() const;
    
    // Speed & Movement
    double getGroundSpeed() const;
    double getAirSpeed() const;
    double getClimbRate() const;
    
    // Attitude
    double getRoll() const;
    double getPitch() const;
    double getYaw() const;
    double getRollSpeed() const;
    double getPitchSpeed() const;
    double getYawSpeed() const;
    
    // Battery (support multiple batteries)
    double getBatteryVoltage(int batteryId) const;
    double getBatteryRemaining(int batteryId) const;
    double getBatteryCurrent(int batteryId) const;
    
    // GPS
    int getGPSFixType() const;
    int getGPSSatelliteCount() const;
    double getGPSHDOP() const;
    
    // System State
    bool isArmed() const;
    bool isFlying() const;
    std::string getFlightMode() const;
    int getSystemId() const;
    int getComponentId() const;
    uint8_t getBaseMode() const;
    uint32_t getCustomMode() const;
    uint8_t getSystemStatus() const;
    uint8_t getAutopilotType() const;
    uint8_t getVehicleType() const;
    
    // System Health
    uint32_t getSensorsPresentBits() const;
    uint32_t getSensorsEnabledBits() const;
    uint32_t getSensorsHealthBits() const;

    // Connection Health (based on QGC VehicleLinkManager)
    bool isHeartbeatTimeout() const;  // Check if heartbeat timeout (>3.5s)
    uint64_t getTimeSinceLastHeartbeat() const;  // Time since last heartbeat (ms)

    // ============================================================================
    // Setters
    // ============================================================================

    void setSystemId(int systemId);
    void setComponentId(int componentId);
    void setFlightMode(const std::string& mode);
    
private:
    // Thread safety
    mutable std::mutex _mutex;
    
    // System identification
    std::atomic<int> _systemId{0};
    std::atomic<int> _componentId{0};
    
    // Position (in degrees and meters)
    std::atomic<double> _latitude{0.0};
    std::atomic<double> _longitude{0.0};
    std::atomic<double> _altitude{0.0};
    std::atomic<double> _altitudeRelative{0.0};
    std::atomic<double> _altitudeAMSL{0.0};
    std::atomic<double> _heading{0.0};
    
    // Velocity (m/s)
    std::atomic<double> _groundSpeed{0.0};
    std::atomic<double> _airSpeed{0.0};
    std::atomic<double> _climbRate{0.0};
    
    // Attitude (radians and rad/s)
    std::atomic<double> _roll{0.0};
    std::atomic<double> _pitch{0.0};
    std::atomic<double> _yaw{0.0};
    std::atomic<double> _rollSpeed{0.0};
    std::atomic<double> _pitchSpeed{0.0};
    std::atomic<double> _yawSpeed{0.0};
    
    // Battery (support up to 3 batteries)
    struct BatteryInfo {
        std::atomic<double> voltage{0.0};
        std::atomic<double> current{0.0};
        std::atomic<double> remaining{-1.0};
        std::atomic<double> temperature{0.0};
    };
    BatteryInfo _batteries[3];
    
    // GPS
    std::atomic<int> _gpsFixType{0};
    std::atomic<int> _gpsSatelliteCount{0};
    std::atomic<double> _gpsHDOP{0.0};
    
    // System state
    std::atomic<uint8_t> _baseMode{0};
    std::atomic<uint32_t> _customMode{0};
    std::atomic<uint8_t> _systemStatus{0};
    std::atomic<uint8_t> _autopilotType{0};
    std::atomic<uint8_t> _vehicleType{0};
    std::atomic<bool> _armed{false};
    
    // System health
    std::atomic<uint32_t> _sensorsPresentBits{0};
    std::atomic<uint32_t> _sensorsEnabledBits{0};
    std::atomic<uint32_t> _sensorsHealthBits{0};
    
    // Flight mode (protected by mutex)
    std::string _flightMode{"UNKNOWN"};
    
    // Timestamps (for detecting stale data)
    std::atomic<uint64_t> _lastHeartbeatMs{0};
    std::atomic<uint64_t> _lastPositionMs{0};
    std::atomic<uint64_t> _lastAttitudeMs{0};
    
    // Helper methods
    uint64_t getCurrentTimeMs() const;
    bool isArmedFromBaseMode(uint8_t baseMode) const;
    std::string flightModeFromCustomMode(uint32_t customMode, uint8_t autopilot) const;

    // Constants (based on QGC VehicleLinkManager)
    static constexpr uint64_t HEARTBEAT_TIMEOUT_MS = 3500;  // 3.5 seconds
};

} // namespace margelo::nitro::mavlink
