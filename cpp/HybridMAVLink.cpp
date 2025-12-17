/**
 * HybridMAVLink.cpp
 * Main implementation of MAVLink bridge for React Native
 */

#include "HybridMAVLink.hpp"
#include <chrono>
#include <thread>

#ifdef __ANDROID__
#include <android/log.h>
#endif

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#endif

namespace margelo::nitro::mavlink {

// ============================================================================
// CONNECTION MANAGEMENT
// ============================================================================

std::shared_ptr<Promise<bool>> HybridMAVLink::connectWithConfig(const ConnectionConfig& config) {
    auto promise = Promise<bool>::create();
    
    // Create thread to connect async
    std::thread([this, config, promise]() {
        bool success = false;
        
        try {
            // Cast double to ConnectionType enum
            ConnectionType connType = static_cast<ConnectionType>(static_cast<int>(config.type));
            
            // Use public connect() method with all parameters
            success = _connectionManager->connect(
                connType,
                config.address,
                static_cast<int>(config.port),
                static_cast<int>(config.baudRate)
            );
            
            if (success) {
                _isConnected = true;
                // Start sending GCS heartbeat to let vehicle know our address
                startGCSHeartbeat();
                // Note: System/component IDs and managers will be initialized when first message received
            }
            
            promise->resolve(success);
        }
        catch (const std::exception& e) {
            promise->reject(std::make_exception_ptr(e));
        }
    }).detach();
    
    return promise;
}

std::shared_ptr<Promise<void>> HybridMAVLink::disconnect() {
    auto promise = Promise<void>::create();
    
    _isConnected = false;
    _connectionManager->disconnect();
    _commandExecutor.reset();
    
    promise->resolve();
    return promise;
}

bool HybridMAVLink::isConnected() {
    return _isConnected && _vehicleState->getSystemId() > 0;
}

// ============================================================================
// VEHICLE STATE & TELEMETRY (Getters)
// ============================================================================

double HybridMAVLink::getLatitude() {
    return _vehicleState->getLatitude();
}

double HybridMAVLink::getLongitude() {
    return _vehicleState->getLongitude();
}

double HybridMAVLink::getAltitude() {
    return _vehicleState->getAltitude();
}

double HybridMAVLink::getHeading() {
    return _vehicleState->getHeading();
}

double HybridMAVLink::getGroundSpeed() {
    return _vehicleState->getGroundSpeed();
}

double HybridMAVLink::getAirSpeed() {
    return _vehicleState->getAirSpeed();
}

double HybridMAVLink::getClimbRate() {
    return _vehicleState->getClimbRate();
}

double HybridMAVLink::getRoll() {
    return _vehicleState->getRoll();
}

double HybridMAVLink::getPitch() {
    return _vehicleState->getPitch();
}

double HybridMAVLink::getYaw() {
    return _vehicleState->getYaw();
}

double HybridMAVLink::getBatteryVoltage(double batteryId) {
    return _vehicleState->getBatteryVoltage(static_cast<int>(batteryId));
}

double HybridMAVLink::getBatteryRemaining(double batteryId) {
    return _vehicleState->getBatteryRemaining(static_cast<int>(batteryId));
}

double HybridMAVLink::getGPSFixType() {
    return _vehicleState->getGPSFixType();
}

double HybridMAVLink::getGPSSatelliteCount() {
    return _vehicleState->getGPSSatelliteCount();
}

bool HybridMAVLink::isArmed() {
    return _vehicleState->isArmed();
}

bool HybridMAVLink::isFlying() {
    return _vehicleState->isFlying();
}

std::string HybridMAVLink::getFlightMode() {
    return _vehicleState->getFlightMode();
}

double HybridMAVLink::getSystemId() {
    return _vehicleState->getSystemId();
}

double HybridMAVLink::getComponentId() {
    return _vehicleState->getComponentId();
}

// ============================================================================
// VEHICLE CONTROL - BASIC
// ============================================================================

std::shared_ptr<Promise<bool>> HybridMAVLink::setArmed(bool arm, bool force) {
    // MAV_CMD_COMPONENT_ARM_DISARM
    // param1: 1 to arm, 0 to disarm
    // param2: force arm/disarm (0=normal, 21196=force)
    float forceParam = force ? 21196.0f : 0.0f;
    return executeCommand(
        MAV_CMD_COMPONENT_ARM_DISARM,
        arm ? 1.0f : 0.0f,
        forceParam
    );
}

std::shared_ptr<Promise<bool>> HybridMAVLink::setFlightMode(const std::string& mode) {
    // NOTE: This is firmware-specific!
    // React Native side must provide correct mode string for the vehicle's firmware
    // ArduPilot: "STABILIZE", "GUIDED", "RTL", etc.
    // PX4: "MANUAL", "POSITION", "AUTO", etc.
    
    // Store mode string in vehicle state for reference
    _vehicleState->setFlightMode(mode);
    
    // MAVLink doesn't have a "set mode by string" command
    // This is a simplified implementation - real implementation needs firmware-specific mapping
    // For now, just reject with message that this needs to be implemented properly
    auto promise = Promise<bool>::create();
    promise->reject(std::make_exception_ptr(std::runtime_error(
        "setFlightMode with string not implemented. Use MAV_CMD_DO_SET_MODE with custom_mode directly."
    )));
    return promise;
}

// ============================================================================
// GUIDED MODE COMMANDS
// ============================================================================

std::shared_ptr<Promise<bool>> HybridMAVLink::guidedTakeoff(double altitude) {
    // MAV_CMD_NAV_TAKEOFF
    // param7: desired altitude
    return executeCommand(
        MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0,
        0, 0, 
        static_cast<float>(altitude)
    );
}

std::shared_ptr<Promise<bool>> HybridMAVLink::guidedLand() {
    // MAV_CMD_NAV_LAND
    return executeCommand(MAV_CMD_NAV_LAND);
}

std::shared_ptr<Promise<bool>> HybridMAVLink::guidedRTL(bool /*smartRTL*/) {
    // MAV_CMD_NAV_RETURN_TO_LAUNCH
    // smartRTL parameter is ignored - ArduCopter specific
    // Use MAV_CMD_DO_SET_MODE with custom_mode if you need SMART_RTL
    return executeCommand(MAV_CMD_NAV_RETURN_TO_LAUNCH);
}

std::shared_ptr<Promise<bool>> HybridMAVLink::guidedGotoCoordinate(const Coordinate& coordinate) {
    auto promise = Promise<bool>::create();
    
    if (!_commandExecutor) {
        promise->reject(std::make_exception_ptr(std::runtime_error("Not connected")));
        return promise;
    }
    
    // Send SET_POSITION_TARGET_GLOBAL_INT
    mavlink_message_t msg;
    mavlink_set_position_target_global_int_t target = {};
    
    target.target_system = _vehicleState->getSystemId();
    target.target_component = _vehicleState->getComponentId();
    target.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    
    // Convert to int32 (degrees * 1e7)
    target.lat_int = static_cast<int32_t>(coordinate.latitude * 1e7);
    target.lon_int = static_cast<int32_t>(coordinate.longitude * 1e7);
    target.alt = static_cast<float>(coordinate.altitude);
    
    // Position only (no velocity/acceleration)
    target.type_mask = 0b0000111111111000; // Only position
    
    mavlink_msg_set_position_target_global_int_encode(
        255, // System ID (GCS)
        MAV_COMP_ID_MISSIONPLANNER,
        &msg,
        &target
    );
    
    _connectionManager->sendMessage(msg);
    promise->resolve(true);
    
    return promise;
}

std::shared_ptr<Promise<bool>> HybridMAVLink::guidedChangeAltitude(double altitudeChange) {
    auto promise = Promise<bool>::create();
    
    // Calculate new altitude
    double currentAlt = _vehicleState->getAltitude();
    double newAlt = currentAlt + altitudeChange;
    
    // Send MAV_CMD_DO_REPOSITION
    return executeCommand(
        MAV_CMD_DO_REPOSITION,
        -1, // Ground speed unchanged
        MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, // Change mode to guided
        0, 0, // Reserved
        0, 0, // Keep current lat/lon
        static_cast<float>(newAlt)
    );
}

std::shared_ptr<Promise<bool>> HybridMAVLink::guidedChangeHeading(double heading) {
    // MAV_CMD_CONDITION_YAW
    // param1: target angle [0-360]
    // param2: angular speed (deg/s)
    // param3: direction (-1:ccw, 1:cw)
    // param4: relative offset (1) or absolute angle (0)
    return executeCommand(
        MAV_CMD_CONDITION_YAW,
        static_cast<float>(heading),
        10.0f, // 10 deg/s
        1.0f, // CW
        0.0f // Absolute
    );
}

std::shared_ptr<Promise<bool>> HybridMAVLink::guidedOrbitParams(const OrbitParams& params) {
    // MAV_CMD_DO_ORBIT
    // param1: radius (m)
    // param2: velocity (m/s, negative for counter-clockwise)
    // param3: yaw behavior (0=front to center, 1=hold initial heading)
    // param5: latitude
    // param6: longitude
    // param7: altitude (MSL)
    return executeCommand(
        MAV_CMD_DO_ORBIT,
        static_cast<float>(params.radius),
        static_cast<float>(params.velocity),
        0, // yaw behavior: front to center
        0, // Reserved
        static_cast<float>(params.latitude),
        static_cast<float>(params.longitude),
        0  // altitude (use current altitude)
    );
}

std::shared_ptr<Promise<bool>> HybridMAVLink::guidedROICoordinate(const Coordinate& coordinate) {
    // MAV_CMD_DO_SET_ROI_LOCATION
    return executeCommand(
        MAV_CMD_DO_SET_ROI_LOCATION,
        0, 0, 0, 0, // Reserved
        static_cast<float>(coordinate.latitude),
        static_cast<float>(coordinate.longitude),
        static_cast<float>(coordinate.altitude)
    );
}

std::shared_ptr<Promise<bool>> HybridMAVLink::guidedClearROI() {
    // MAV_CMD_DO_SET_ROI_NONE
    return executeCommand(MAV_CMD_DO_SET_ROI_NONE);
}

std::shared_ptr<Promise<bool>> HybridMAVLink::pauseVehicle() {
    // MAV_CMD_DO_PAUSE_CONTINUE
    // Note: Different firmwares may handle this differently
    return executeCommand(MAV_CMD_DO_PAUSE_CONTINUE, 0); // 0 = pause
}

std::shared_ptr<Promise<bool>> HybridMAVLink::emergencyStop() {
    // MAV_CMD_DO_MOTOR_TEST with stop
    // Or disarm with force
    return setArmed(false, true);
}

// ============================================================================
// MISSION MANAGEMENT
// ============================================================================

std::shared_ptr<Promise<bool>> HybridMAVLink::startMission() {
    // MAV_CMD_MISSION_START
    return executeCommand(MAV_CMD_MISSION_START);
}

std::shared_ptr<Promise<bool>> HybridMAVLink::setCurrentMissionItem(double sequence) {
    auto promise = Promise<bool>::create();
    
    if (!_connectionManager->isConnected()) {
        promise->reject(std::make_exception_ptr(std::runtime_error("Not connected")));
        return promise;
    }
    
    // Send MISSION_SET_CURRENT
    mavlink_message_t msg;
    mavlink_mission_set_current_t current = {};
    
    current.target_system = _vehicleState->getSystemId();
    current.target_component = _vehicleState->getComponentId();
    current.seq = static_cast<uint16_t>(sequence);
    
    mavlink_msg_mission_set_current_encode(
        255,
        MAV_COMP_ID_MISSIONPLANNER,
        &msg,
        &current
    );
    
    _connectionManager->sendMessage(msg);
    _currentMissionItem = static_cast<int>(sequence);
    promise->resolve(true);
    
    return promise;
}

double HybridMAVLink::getCurrentMissionItem() {
    return _currentMissionItem;
}

std::shared_ptr<Promise<bool>> HybridMAVLink::clearMission() {
    auto promise = Promise<bool>::create();
    
    if (!_connectionManager->isConnected()) {
        promise->reject(std::make_exception_ptr(std::runtime_error("Not connected")));
        return promise;
    }
    
    // Send MISSION_CLEAR_ALL
    mavlink_message_t msg;
    mavlink_mission_clear_all_t clear = {};
    
    clear.target_system = _vehicleState->getSystemId();
    clear.target_component = _vehicleState->getComponentId();
    
    mavlink_msg_mission_clear_all_encode(
        255,
        MAV_COMP_ID_MISSIONPLANNER,
        &msg,
        &clear
    );
    
    _connectionManager->sendMessage(msg);
    _currentMissionItem = -1;
    promise->resolve(true);
    
    return promise;
}

// ============================================================================
// PARAMETER MANAGEMENT
// ============================================================================

std::shared_ptr<Promise<double>> HybridMAVLink::getParameter(const std::string& name) {
    auto promise = Promise<double>::create();
    
    if (!_parameterManager) {
        promise->reject(std::make_exception_ptr(std::runtime_error("Not connected")));
        return promise;
    }
    
    _parameterManager->getParameter(name, [promise](bool success, float value) {
        if (success) {
            promise->resolve(static_cast<double>(value));
        } else {
            promise->reject(std::make_exception_ptr(std::runtime_error("Parameter request failed or timed out")));
        }
    });
    
    return promise;
}

std::shared_ptr<Promise<bool>> HybridMAVLink::setParameter(const std::string& name, double value) {
    auto promise = Promise<bool>::create();
    
    if (!_parameterManager) {
        promise->reject(std::make_exception_ptr(std::runtime_error("Not connected")));
        return promise;
    }
    
    _parameterManager->setParameter(
        name, 
        static_cast<float>(value),
        MAV_PARAM_TYPE_REAL32,
        [promise](bool success) {
            if (success) {
                promise->resolve(true);
            } else {
                promise->reject(std::make_exception_ptr(std::runtime_error("Parameter set failed or timed out")));
            }
        }
    );
    
    return promise;
}

std::shared_ptr<Promise<bool>> HybridMAVLink::setParameterValue(const ParameterSet& param) {
    return setParameter(param.name, param.value);
}

std::shared_ptr<Promise<bool>> HybridMAVLink::refreshParameters() {
    auto promise = Promise<bool>::create();
    
    if (!_parameterManager) {
        promise->reject(std::make_exception_ptr(std::runtime_error("Not connected")));
        return promise;
    }
    
    _parameterManager->requestAllParameters();
    promise->resolve(true);
    
    return promise;
}

// ============================================================================
// CAMERA & GIMBAL
// ============================================================================

std::shared_ptr<Promise<bool>> HybridMAVLink::triggerCamera() {
    // MAV_CMD_DO_DIGICAM_CONTROL
    return executeCommand(
        MAV_CMD_DO_DIGICAM_CONTROL,
        0, 0, 0, 0, 1, 0, 0 // param5=1 triggers camera
    );
}

std::shared_ptr<Promise<bool>> HybridMAVLink::startVideoRecording() {
    // MAV_CMD_VIDEO_START_CAPTURE
    return executeCommand(MAV_CMD_VIDEO_START_CAPTURE);
}

std::shared_ptr<Promise<bool>> HybridMAVLink::stopVideoRecording() {
    // MAV_CMD_VIDEO_STOP_CAPTURE
    return executeCommand(MAV_CMD_VIDEO_STOP_CAPTURE);
}

std::shared_ptr<Promise<bool>> HybridMAVLink::setGimbalAttitudeParams(const GimbalAttitude& attitude) {
    // MAV_CMD_DO_MOUNT_CONTROL
    // params 1-3: pitch, roll, yaw (degrees)
    // param7: mode (2=MAV_MOUNT_MODE_MAVLINK_TARGETING)
    return executeCommand(
        MAV_CMD_DO_MOUNT_CONTROL,
        static_cast<float>(attitude.pitch),
        0, // roll (not used, GimbalAttitude only has pitch/yaw)
        static_cast<float>(attitude.yaw),
        0, 0, 0,
        MAV_MOUNT_MODE_MAVLINK_TARGETING
    );
}

// ============================================================================
// MANUAL CONTROL
// ============================================================================

void HybridMAVLink::sendManualControlInput(const ManualControlInput& input) {
    if (!_connectionManager->isConnected()) return;
    
    mavlink_message_t msg;
    mavlink_manual_control_t control = {};
    
    control.target = _vehicleState->getSystemId();
    
    // MAVLink MANUAL_CONTROL uses x=pitch, y=roll, z=throttle, r=yaw
    // Our interface uses roll/pitch/yaw/throttle (-1000 to 1000, 0 to 1000 for throttle)
    control.x = static_cast<int16_t>(input.pitch);  // pitch
    control.y = static_cast<int16_t>(input.roll);   // roll
    control.z = static_cast<int16_t>(input.throttle); // throttle (0-1000)
    control.r = static_cast<int16_t>(input.yaw);    // yaw
    control.buttons = 0; // Not exposed in ManualControlInput
    
    mavlink_msg_manual_control_encode(
        255,
        MAV_COMP_ID_MISSIONPLANNER,
        &msg,
        &control
    );
    
    _connectionManager->sendMessage(msg);
}

// ============================================================================
// ADVANCED COMMANDS
// ============================================================================

std::shared_ptr<Promise<bool>> HybridMAVLink::rebootAutopilot() {
    // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    // param1: 1=reboot autopilot
    return executeCommand(
        MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        1.0f // Reboot autopilot
    );
}

std::shared_ptr<Promise<bool>> HybridMAVLink::requestDataStreamParams(const DataStreamRequest& request) {
    auto promise = Promise<bool>::create();
    
    if (!_connectionManager->isConnected()) {
        promise->reject(std::make_exception_ptr(std::runtime_error("Not connected")));
        return promise;
    }
    
    // Send REQUEST_DATA_STREAM message (based on QGC Vehicle.cc line 1655-1685)
    mavlink_message_t msg;
    mavlink_request_data_stream_t dataStream = {};
    
    dataStream.target_system = _vehicleState->getSystemId();
    dataStream.target_component = _vehicleState->getComponentId();
    dataStream.req_stream_id = static_cast<uint8_t>(request.streamId);
    dataStream.req_message_rate = static_cast<uint16_t>(request.rate);
    dataStream.start_stop = 1; // 1=start, 0=stop
    
    mavlink_msg_request_data_stream_encode(
        _connectionManager->getSystemId(),
        _connectionManager->getComponentId(),
        &msg,
        &dataStream
    );
    
    _connectionManager->sendMessage(msg);
    promise->resolve(true);
    return promise;
}

std::shared_ptr<Promise<bool>> HybridMAVLink::sendCommandParams(const CommandParams& params) {
    return executeCommand(
        static_cast<uint16_t>(params.command),
        static_cast<float>(params.param1),
        static_cast<float>(params.param2),
        static_cast<float>(params.param3),
        static_cast<float>(params.param4),
        static_cast<float>(params.param5),
        static_cast<float>(params.param6),
        static_cast<float>(params.param7)
    );
}

// ============================================================================
// PRIVATE HELPERS
// ============================================================================

std::shared_ptr<Promise<bool>> HybridMAVLink::executeCommand(
    uint16_t command,
    float param1, float param2, float param3, float param4,
    float param5, float param6, float param7
) {
    auto promise = Promise<bool>::create();
    
    if (!_commandExecutor) {
        promise->reject(std::make_exception_ptr(std::runtime_error("Not connected")));
        return promise;
    }
    
    _commandExecutor->sendCommand(
        command, param1, param2, param3, param4, param5, param6, param7,
        [promise](CommandResult result, uint8_t /*progress*/) {
            if (result == CommandResult::SUCCESS || result == CommandResult::IN_PROGRESS) {
                promise->resolve(true);
            } else {
                // Map CommandResult to error message
                std::string msg = "Command failed";
                if (result == CommandResult::FAILED) msg = "Command failed";
                else if (result == CommandResult::TIMEOUT) msg = "Command timeout";
                else if (result == CommandResult::DENIED) msg = "Command denied";
                promise->reject(std::make_exception_ptr(std::runtime_error(msg)));
            }
        }
    );
    
    return promise;
}

void HybridMAVLink::handleMavlinkMessage(const mavlink_message_t& message) {
    // Route telemetry messages to VehicleState
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            _vehicleState->handleHeartbeat(heartbeat);
            
            // First HEARTBEAT from vehicle - save system/component ID
            // Based on QGC Vehicle.cc:454-458
            if (_vehicleState->getSystemId() == 0) {
                #ifdef __ANDROID__
                __android_log_print(ANDROID_LOG_INFO, "MAVLink", "First HEARTBEAT: sysid=%d, compid=%d", 
                    message.sysid, message.compid);
                #endif
                
                _vehicleState->setSystemId(message.sysid);
                _vehicleState->setComponentId(message.compid);
                
                // Initialize CommandExecutor and ParameterManager now that we know target system
                // Based on QGC Vehicle.cc:289-290
                if (_isConnected) {
                    _commandExecutor = std::make_shared<CommandExecutor>(
                        _connectionManager,
                        message.sysid,
                        message.compid
                    );
                    _parameterManager = std::make_shared<ParameterManager>(
                        _connectionManager,
                        message.sysid,
                        message.compid
                    );
                    
                    #ifdef __ANDROID__
                    __android_log_print(ANDROID_LOG_INFO, "MAVLink", "Managers initialized for system %d", message.sysid);
                    #endif
                }
            }
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t position;
            mavlink_msg_global_position_int_decode(&message, &position);
            _vehicleState->handleGlobalPositionInt(position);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&message, &attitude);
            _vehicleState->handleAttitude(attitude);
            break;
        }
        case MAVLINK_MSG_ID_BATTERY_STATUS: {
            mavlink_battery_status_t battery;
            mavlink_msg_battery_status_decode(&message, &battery);
            _vehicleState->handleBatteryStatus(battery);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&message, &gps);
            _vehicleState->handleGPSRawInt(gps);
            break;
        }
        case MAVLINK_MSG_ID_VFR_HUD: {
            mavlink_vfr_hud_t vfr;
            mavlink_msg_vfr_hud_decode(&message, &vfr);
            _vehicleState->handleVFRHUD(vfr);
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sysStatus;
            mavlink_msg_sys_status_decode(&message, &sysStatus);
            _vehicleState->handleSysStatus(sysStatus);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            if (_commandExecutor) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&message, &ack);
                _commandExecutor->handleCommandAck(ack);
            }
            break;
        }
        case MAVLINK_MSG_ID_PARAM_VALUE: {
            if (_parameterManager) {
                mavlink_param_value_t paramValue;
                mavlink_msg_param_value_decode(&message, &paramValue);
                _parameterManager->handleParamValue(paramValue);
            }
            break;
        }
        case MAVLINK_MSG_ID_MISSION_CURRENT: {
            mavlink_mission_current_t current;
            mavlink_msg_mission_current_decode(&message, &current);
            _currentMissionItem = current.seq;
            break;
        }
    }
}

void HybridMAVLink::timeoutCheckLoop() {
    while (_shouldRun) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        if (_commandExecutor) {
            _commandExecutor->checkTimeouts();
        }
        
        if (_parameterManager) {
            _parameterManager->checkTimeouts();
        }
    }
}

void HybridMAVLink::startGCSHeartbeat() {
    // Start thread to send GCS heartbeat every 1 second
    // This lets vehicle know our UDP address for replies
    std::thread([this]() {
        #ifdef __ANDROID__
        __android_log_print(ANDROID_LOG_INFO, "MAVLink", "Starting GCS heartbeat thread");
        #endif
        
        while (_isConnected && _shouldRun) {
            sendGCSHeartbeat();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }).detach();
}

void HybridMAVLink::sendGCSHeartbeat() {
    mavlink_message_t msg;
    
    // GCS heartbeat: type=MAV_TYPE_GCS, autopilot=MAV_AUTOPILOT_INVALID
    mavlink_msg_heartbeat_pack(
        255,  // System ID for GCS
        190,  // Component ID for GCS
        &msg,
        MAV_TYPE_GCS,           // type
        MAV_AUTOPILOT_INVALID,  // autopilot
        0,                      // base_mode
        0,                      // custom_mode
        MAV_STATE_ACTIVE        // system_status
    );
    
    // Send to connection
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    _connectionManager->sendMessage(buffer, len);
    
    #ifdef __ANDROID__
    static int heartbeat_count = 0;
    if (++heartbeat_count % 10 == 0) { // Log every 10 heartbeats
        __android_log_print(ANDROID_LOG_DEBUG, "MAVLink", "Sent GCS heartbeat #%d", heartbeat_count);
    }
    #endif
}

} // namespace margelo::nitro::mavlink
