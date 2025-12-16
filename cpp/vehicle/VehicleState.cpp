/**
 * VehicleState.cpp
 * Implementation of VehicleState
 */

#include "VehicleState.hpp"
#include "../MAVLink/v2.0/common/mavlink.h"
#include <chrono>
#include <cmath>

namespace margelo::nitro::mavlink {

VehicleState::VehicleState() {
    // Initialize with default values
}

// ============================================================================
// Message Handlers
// ============================================================================

void VehicleState::handleHeartbeat(const mavlink_heartbeat_t& heartbeat) {
    _baseMode = heartbeat.base_mode;
    _customMode = heartbeat.custom_mode;
    _systemStatus = heartbeat.system_status;
    _autopilotType = heartbeat.autopilot;
    _vehicleType = heartbeat.type;
    
    // Update armed state
    _armed = isArmedFromBaseMode(heartbeat.base_mode);
    
    // Update flight mode
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _flightMode = flightModeFromCustomMode(heartbeat.custom_mode, heartbeat.autopilot);
    }
    
    _lastHeartbeatMs = getCurrentTimeMs();
}

void VehicleState::handleGlobalPositionInt(const mavlink_global_position_int_t& position) {
    _latitude = position.lat / 1e7;
    _longitude = position.lon / 1e7;
    _altitude = position.alt / 1000.0;
    _altitudeRelative = position.relative_alt / 1000.0;
    _altitudeAMSL = position.alt / 1000.0;
    _heading = position.hdg / 100.0;
    
    // Velocity
    _groundSpeed = std::sqrt(position.vx * position.vx + position.vy * position.vy) / 100.0;
    _climbRate = -position.vz / 100.0; // Positive up
    
    _lastPositionMs = getCurrentTimeMs();
}

void VehicleState::handleAttitude(const mavlink_attitude_t& attitude) {
    _roll = attitude.roll;
    _pitch = attitude.pitch;
    _yaw = attitude.yaw;
    _rollSpeed = attitude.rollspeed;
    _pitchSpeed = attitude.pitchspeed;
    _yawSpeed = attitude.yawspeed;
    
    _lastAttitudeMs = getCurrentTimeMs();
}

void VehicleState::handleBatteryStatus(const mavlink_battery_status_t& battery) {
    int batteryId = battery.id;
    if (batteryId >= 0 && batteryId < 3) {
        // Voltage in millivolts, convert to volts
        if (battery.voltages[0] != UINT16_MAX) {
            double totalVoltage = 0;
            int cellCount = 0;
            for (int i = 0; i < 10; i++) {
                if (battery.voltages[i] != UINT16_MAX) {
                    totalVoltage += battery.voltages[i];
                    cellCount++;
                }
            }
            if (cellCount > 0) {
                _batteries[batteryId].voltage = totalVoltage / 1000.0;
            }
        }
        
        // Current in centiamperes, convert to amperes
        if (battery.current_battery != -1) {
            _batteries[batteryId].current = battery.current_battery / 100.0;
        }
        
        // Remaining percentage
        if (battery.battery_remaining != -1) {
            _batteries[batteryId].remaining = battery.battery_remaining;
        }
        
        // Temperature
        if (battery.temperature != INT16_MAX) {
            _batteries[batteryId].temperature = battery.temperature / 100.0;
        }
    }
}

void VehicleState::handleGPSRawInt(const mavlink_gps_raw_int_t& gps) {
    _gpsFixType = gps.fix_type;
    _gpsSatelliteCount = gps.satellites_visible;
    _gpsHDOP = gps.eph / 100.0;
}

void VehicleState::handleVFRHUD(const mavlink_vfr_hud_t& vfr) {
    _airSpeed = vfr.airspeed;
    _groundSpeed = vfr.groundspeed;
    _heading = vfr.heading;
    _climbRate = vfr.climb;
}

void VehicleState::handleSysStatus(const mavlink_sys_status_t& sysStatus) {
    _sensorsPresentBits = sysStatus.onboard_control_sensors_present;
    _sensorsEnabledBits = sysStatus.onboard_control_sensors_enabled;
    _sensorsHealthBits = sysStatus.onboard_control_sensors_health;
}

// ============================================================================
// Getters
// ============================================================================

double VehicleState::getLatitude() const { return _latitude.load(); }
double VehicleState::getLongitude() const { return _longitude.load(); }
double VehicleState::getAltitude() const { return _altitudeRelative.load(); }
double VehicleState::getAltitudeRelative() const { return _altitudeRelative.load(); }
double VehicleState::getAltitudeAMSL() const { return _altitudeAMSL.load(); }
double VehicleState::getHeading() const { return _heading.load(); }

double VehicleState::getGroundSpeed() const { return _groundSpeed.load(); }
double VehicleState::getAirSpeed() const { return _airSpeed.load(); }
double VehicleState::getClimbRate() const { return _climbRate.load(); }

double VehicleState::getRoll() const { return _roll.load(); }
double VehicleState::getPitch() const { return _pitch.load(); }
double VehicleState::getYaw() const { return _yaw.load(); }
double VehicleState::getRollSpeed() const { return _rollSpeed.load(); }
double VehicleState::getPitchSpeed() const { return _pitchSpeed.load(); }
double VehicleState::getYawSpeed() const { return _yawSpeed.load(); }

double VehicleState::getBatteryVoltage(int batteryId) const {
    if (batteryId >= 0 && batteryId < 3) {
        return _batteries[batteryId].voltage.load();
    }
    return 0.0;
}

double VehicleState::getBatteryRemaining(int batteryId) const {
    if (batteryId >= 0 && batteryId < 3) {
        return _batteries[batteryId].remaining.load();
    }
    return -1.0;
}

double VehicleState::getBatteryCurrent(int batteryId) const {
    if (batteryId >= 0 && batteryId < 3) {
        return _batteries[batteryId].current.load();
    }
    return 0.0;
}

int VehicleState::getGPSFixType() const { return _gpsFixType.load(); }
int VehicleState::getGPSSatelliteCount() const { return _gpsSatelliteCount.load(); }
double VehicleState::getGPSHDOP() const { return _gpsHDOP.load(); }

bool VehicleState::isArmed() const { return _armed.load(); }

bool VehicleState::isFlying() const {
    // Simple heuristic: armed and altitude > 0.5m or climbing
    return _armed.load() && (_altitudeRelative.load() > 0.5 || _climbRate.load() > 0.1);
}

std::string VehicleState::getFlightMode() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _flightMode;
}

int VehicleState::getSystemId() const { return _systemId.load(); }
int VehicleState::getComponentId() const { return _componentId.load(); }
uint8_t VehicleState::getBaseMode() const { return _baseMode.load(); }
uint32_t VehicleState::getCustomMode() const { return _customMode.load(); }
uint8_t VehicleState::getSystemStatus() const { return _systemStatus.load(); }
uint8_t VehicleState::getAutopilotType() const { return _autopilotType.load(); }
uint8_t VehicleState::getVehicleType() const { return _vehicleType.load(); }

uint32_t VehicleState::getSensorsPresentBits() const { return _sensorsPresentBits.load(); }
uint32_t VehicleState::getSensorsEnabledBits() const { return _sensorsEnabledBits.load(); }
uint32_t VehicleState::getSensorsHealthBits() const { return _sensorsHealthBits.load(); }

// ============================================================================
// Setters
// ============================================================================

void VehicleState::setSystemId(int systemId) {
    _systemId = systemId;
}

void VehicleState::setComponentId(int componentId) {
    _componentId = componentId;
}

void VehicleState::setFlightMode(const std::string& mode) {
    std::lock_guard<std::mutex> lock(_mutex);
    _flightMode = mode;
}

// ============================================================================
// Helper Methods
// ============================================================================

uint64_t VehicleState::getCurrentTimeMs() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

bool VehicleState::isArmedFromBaseMode(uint8_t baseMode) const {
    return (baseMode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
}

std::string VehicleState::flightModeFromCustomMode(uint32_t customMode, uint8_t autopilot) const {
    // This is simplified - in reality, you'd need to map custom modes based on autopilot type
    // PX4 and ArduPilot have different custom mode mappings
    
    if (autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        // ArduPilot modes (example for Copter)
        switch (customMode) {
            case 0: return "STABILIZE";
            case 1: return "ACRO";
            case 2: return "ALT_HOLD";
            case 3: return "AUTO";
            case 4: return "GUIDED";
            case 5: return "LOITER";
            case 6: return "RTL";
            case 7: return "CIRCLE";
            case 9: return "LAND";
            case 16: return "POSHOLD";
            default: return "MODE_" + std::to_string(customMode);
        }
    } else if (autopilot == MAV_AUTOPILOT_PX4) {
        // PX4 modes
        uint8_t mainMode = (customMode >> 16) & 0xFF;
        switch (mainMode) {
            case 1: return "MANUAL";
            case 2: return "ALTITUDE";
            case 3: return "POSITION";
            case 4: return "AUTO";
            case 5: return "ACRO";
            case 6: return "OFFBOARD";
            case 7: return "STABILIZED";
            case 8: return "RATTITUDE";
            default: return "MODE_" + std::to_string(customMode);
        }
    }
    
    return "UNKNOWN";
}

} // namespace margelo::nitro::mavlink
