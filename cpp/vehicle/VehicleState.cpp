/**
 * VehicleState.cpp
 * Implementation of VehicleState
 */

#include "VehicleState.hpp"
#include "../mavlink/v2.0/common/mavlink.h"
#include <chrono>
#include <cmath>

#ifdef __ANDROID__
#include <android/log.h>
#endif

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
    
    #ifdef __ANDROID__
    // Debug log to confirm HEARTBEAT is being processed
    static int heartbeatCount = 0;
    if (++heartbeatCount % 20 == 0) { // Log every 20th heartbeat (every 20s at 1Hz)
        __android_log_print(ANDROID_LOG_DEBUG, "MAVLink", 
            "HEARTBEAT #%d: autopilot=%d type=%d armed=%d mode=%s", 
            heartbeatCount, heartbeat.autopilot, heartbeat.type, 
            _armed.load(), _flightMode.c_str());
    }
    #endif
    
    _lastHeartbeatMs = getCurrentTimeMs();
}

void VehicleState::handleGlobalPositionInt(const mavlink_global_position_int_t& position) {
    // ArduPilot sends bogus GLOBAL_POSITION_INT messages with lat/lon 0/0 even when it has no GPS signal
    // Apparently, this is in order to transport relative altitude information (QGC Vehicle.cc:738)
    if (position.lat == 0 && position.lon == 0) {
        // Still update altitude even with bogus position
        _altitude = position.alt / 1000.0;
        _altitudeRelative = position.relative_alt / 1000.0;
        _altitudeAMSL = position.alt / 1000.0;
        return;
    }
    
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
    
    #ifdef __ANDROID__
    // Debug log to confirm ATTITUDE is being processed
    static int attitudeCount = 0;
    if (++attitudeCount % 100 == 0) { // Log every 100th message to avoid spam
        __android_log_print(ANDROID_LOG_DEBUG, "MAVLink", 
            "ATTITUDE #%d: roll=%.4f pitch=%.4f yaw=%.4f", 
            attitudeCount, attitude.roll, attitude.pitch, attitude.yaw);
    }
    #endif
    
    _lastAttitudeMs = getCurrentTimeMs();
}

void VehicleState::handleBatteryStatus(const mavlink_battery_status_t& battery) {
    int batteryId = battery.id;
    if (batteryId >= 0 && batteryId < 3) {
        // Calculate total voltage from all cell voltages
        // Based on QGC BatteryFactGroupListModel::_handleBatteryStatus
        double totalVoltage = 0.0;
        bool hasValidVoltage = false;
        
        // Process main voltages array (cells 1-10)
        // voltages are in millivolts, UINT16_MAX means invalid/not used
        for (int i = 0; i < 10; i++) {
            if (battery.voltages[i] == UINT16_MAX) {
                break; // No more valid cells
            }
            totalVoltage += battery.voltages[i];
            hasValidVoltage = true;
        }
        
        // Process extension voltages (cells 11-14)
        // voltages_ext are in millivolts, 0 means invalid/not used
        for (int i = 0; i < 4; i++) {
            if (battery.voltages_ext[i] == 0) {
                break; // No more valid cells
            }
            totalVoltage += battery.voltages_ext[i];
        }
        
        // Convert from millivolts to volts
        if (hasValidVoltage) {
            _batteries[batteryId].voltage = totalVoltage / 1000.0;
        }
        
        // Current in centiamperes, convert to amperes
        // -1 means unknown
        if (battery.current_battery != -1) {
            _batteries[batteryId].current = battery.current_battery / 100.0;
        }
        
        // Remaining percentage
        // -1 means unknown
        if (battery.battery_remaining != -1) {
            _batteries[batteryId].remaining = battery.battery_remaining;
        }
        
        // Temperature in centi-degrees Celsius
        // INT16_MAX means unknown
        if (battery.temperature != INT16_MAX) {
            _batteries[batteryId].temperature = battery.temperature / 100.0;
        }
    }
    
    // Update timestamp for stream re-initialization detection 
    _lastBatteryStatusMs = getCurrentTimeMs();
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
    
    // SYS_STATUS also contains battery info (fallback if no BATTERY_STATUS message)
    // voltage_battery in millivolts, convert to volts
    if (sysStatus.voltage_battery != UINT16_MAX) {
        _batteries[0].voltage = sysStatus.voltage_battery / 1000.0;
    }
    
    // battery_remaining in percentage (-1 if unknown)
    if (sysStatus.battery_remaining != -1) {
        _batteries[0].remaining = sysStatus.battery_remaining;
    }
    
    // current_battery in centiamperes, convert to amperes
    if (sysStatus.current_battery != -1) {
        _batteries[0].current = sysStatus.current_battery / 100.0;
    }
    
    #ifdef __ANDROID__
    // Debug log to confirm SYS_STATUS is being processed
    static int sysStatusCount = 0;
    if (++sysStatusCount % 50 == 0) { // Log every 50th message
        __android_log_print(ANDROID_LOG_DEBUG, "MAVLink", 
            "SYS_STATUS #%d: voltage=%d mV (%.2fV), remaining=%d%%", 
            sysStatusCount, sysStatus.voltage_battery, 
            _batteries[0].voltage.load(), sysStatus.battery_remaining);
    }
    #endif
}

void VehicleState::handleHomePosition(const mavlink_home_position_t& home) {
    // HOME_POSITION message handling (based on QGC Vehicle.cc)
    _homeLatitude = home.latitude / 1e7;
    _homeLongitude = home.longitude / 1e7;
    _homeAltitude = home.altitude / 1000.0;  // mm to meters
    _hasHomePosition = true;
}

void VehicleState::handleExtendedSysState(const mavlink_extended_sys_state_t& extState) {
    // EXTENDED_SYS_STATE handling (based on QGC Vehicle.cc lines 891-921)
    _landedState = extState.landed_state;
    
    switch (extState.landed_state) {
        case MAV_LANDED_STATE_ON_GROUND:
            _flying = false;
            _landing = false;
            break;
        case MAV_LANDED_STATE_TAKEOFF:
        case MAV_LANDED_STATE_IN_AIR:
            _flying = true;
            _landing = false;
            break;
        case MAV_LANDED_STATE_LANDING:
            _flying = true;
            _landing = true;
            break;
        default:
            // MAV_LANDED_STATE_UNDEFINED - keep current values
            break;
    }
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
    // If we have valid EXTENDED_SYS_STATE data, use it (QGC approach)
    uint8_t landedState = _landedState.load();
    if (landedState != MAV_LANDED_STATE_UNDEFINED) {
        return _flying.load();
    }
    
    // Fallback to heuristic for autopilots that don't send EXTENDED_SYS_STATE
    // Based on QGC Vehicle.cc behavior
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

// Connection Health
bool VehicleState::isHeartbeatTimeout() const {
    uint64_t lastHeartbeat = _lastHeartbeatMs.load();
    if (lastHeartbeat == 0) {
        // Never received heartbeat yet
        return false;
    }
    uint64_t timeSinceLastHeartbeat = getCurrentTimeMs() - lastHeartbeat;
    return timeSinceLastHeartbeat > HEARTBEAT_TIMEOUT_MS;
}

uint64_t VehicleState::getTimeSinceLastHeartbeat() const {
    uint64_t lastHeartbeat = _lastHeartbeatMs.load();
    if (lastHeartbeat == 0) {
        return 0;
    }
    return getCurrentTimeMs() - lastHeartbeat;
}

uint64_t VehicleState::getTimeSinceLastBatteryStatus() const {
    uint64_t lastBattery = _lastBatteryStatusMs.load();
    if (lastBattery == 0) {
        return 0;
    }
    return getCurrentTimeMs() - lastBattery;
}

// Home Position getters
double VehicleState::getHomeLatitude() const { return _homeLatitude.load(); }
double VehicleState::getHomeLongitude() const { return _homeLongitude.load(); }
double VehicleState::getHomeAltitude() const { return _homeAltitude.load(); }
bool VehicleState::hasHomePosition() const { return _hasHomePosition.load(); }

// Landed state getters
bool VehicleState::isLanding() const { return _landing.load(); }
uint8_t VehicleState::getLandedState() const { return _landedState.load(); }

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
