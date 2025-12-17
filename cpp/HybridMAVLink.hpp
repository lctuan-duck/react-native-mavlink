/**
 * HybridMAVLink.hpp
 * Main MAVLink implementation for React Native
 */

#pragma once

#include <NitroModules/Promise.hpp>
#include "HybridMAVLinkSpec.hpp"
#include "vehicle/VehicleState.hpp"
#include "connection/ConnectionManager.hpp"
#include "commands/CommandExecutor.hpp"
#include "parameters/ParameterManager.hpp"
#include <memory>
#include <thread>
#include <atomic>

namespace margelo::nitro::mavlink {

class HybridMAVLink : public HybridMAVLinkSpec {
public:
    HybridMAVLink() : HybridObject(TAG) {
        _vehicleState = std::make_shared<VehicleState>();
        _connectionManager = std::make_shared<ConnectionManager>();
        
        // Setup message callback
        _connectionManager->setMessageCallback([this](const mavlink_message_t& msg) {
            handleMavlinkMessage(msg);
        });
        
        // Setup connection callback  
        _connectionManager->setConnectionCallback([this](bool connected) {
            _isConnected = connected;
            // Note: CommandExecutor and ParameterManager are now created when first message received
            // This ensures we have valid system/component IDs from the vehicle
        });
        
        // Start timeout check thread
        _timeoutCheckThread = std::thread(&HybridMAVLink::timeoutCheckLoop, this);
    }
    
    ~HybridMAVLink() {
        _shouldRun = false;
        if (_timeoutCheckThread.joinable()) {
            _timeoutCheckThread.join();
        }
        disconnect();
    }

    // ============================================================================
    // CONNECTION MANAGEMENT
    // ============================================================================
    
    std::shared_ptr<Promise<bool>> connectWithConfig(const ConnectionConfig& config) override;
    std::shared_ptr<Promise<void>> disconnect() override;
    bool isConnected() override;
    
    // ============================================================================
    // VEHICLE STATE & TELEMETRY
    // ============================================================================
    
    double getLatitude() override;
    double getLongitude() override;
    double getAltitude() override;
    double getHeading() override;
    double getGroundSpeed() override;
    double getAirSpeed() override;
    double getClimbRate() override;
    double getRoll() override;
    double getPitch() override;
    double getYaw() override;
    double getBatteryVoltage(double batteryId) override;
    double getBatteryRemaining(double batteryId) override;
    double getGPSFixType() override;
    double getGPSSatelliteCount() override;
    bool isArmed() override;
    bool isFlying() override;
    std::string getFlightMode() override;
    double getSystemId() override;
    double getComponentId() override;
    
    // ============================================================================
    // VEHICLE CONTROL - BASIC
    // ============================================================================
    
    std::shared_ptr<Promise<bool>> setArmed(bool arm, bool force) override;
    std::shared_ptr<Promise<bool>> setFlightMode(const std::string& mode) override;
    
    // ============================================================================
    // GUIDED MODE COMMANDS
    // ============================================================================
    
    std::shared_ptr<Promise<bool>> guidedTakeoff(double altitude) override;
    std::shared_ptr<Promise<bool>> guidedLand() override;
    std::shared_ptr<Promise<bool>> guidedRTL(bool smartRTL) override;
    std::shared_ptr<Promise<bool>> guidedGotoCoordinate(const Coordinate& coordinate) override;
    std::shared_ptr<Promise<bool>> guidedChangeAltitude(double altitudeChange) override;
    std::shared_ptr<Promise<bool>> guidedChangeHeading(double heading) override;
    std::shared_ptr<Promise<bool>> guidedOrbitParams(const OrbitParams& params) override;
    std::shared_ptr<Promise<bool>> guidedROICoordinate(const Coordinate& coordinate) override;
    std::shared_ptr<Promise<bool>> guidedClearROI() override;
    std::shared_ptr<Promise<bool>> pauseVehicle() override;
    std::shared_ptr<Promise<bool>> emergencyStop() override;
    
    // ============================================================================
    // MISSION MANAGEMENT
    // ============================================================================
    
    std::shared_ptr<Promise<bool>> startMission() override;
    std::shared_ptr<Promise<bool>> setCurrentMissionItem(double sequence) override;
    double getCurrentMissionItem() override;
    std::shared_ptr<Promise<bool>> clearMission() override;
    
    // ============================================================================
    // PARAMETER MANAGEMENT
    // ============================================================================
    
    std::shared_ptr<Promise<double>> getParameter(const std::string& name) override;
    std::shared_ptr<Promise<bool>> setParameter(const std::string& name, double value) override;
    std::shared_ptr<Promise<bool>> setParameterValue(const ParameterSet& param) override;
    std::shared_ptr<Promise<bool>> refreshParameters() override;
    
    // ============================================================================
    // CAMERA & GIMBAL
    // ============================================================================
    
    std::shared_ptr<Promise<bool>> triggerCamera() override;
    std::shared_ptr<Promise<bool>> startVideoRecording() override;
    std::shared_ptr<Promise<bool>> stopVideoRecording() override;
    std::shared_ptr<Promise<bool>> setGimbalAttitudeParams(const GimbalAttitude& attitude) override;
    
    // ============================================================================
    // MANUAL CONTROL
    // ============================================================================
    
    void sendManualControlInput(const ManualControlInput& input) override;
    
    // ============================================================================
    // ADVANCED COMMANDS
    // ============================================================================
    
    std::shared_ptr<Promise<bool>> rebootAutopilot() override;
    std::shared_ptr<Promise<bool>> requestDataStreamParams(const DataStreamRequest& request) override;
    std::shared_ptr<Promise<bool>> sendCommandParams(const CommandParams& params) override;

private:
    // Components
    std::shared_ptr<VehicleState> _vehicleState;
    std::shared_ptr<ConnectionManager> _connectionManager;
    std::shared_ptr<CommandExecutor> _commandExecutor;
    std::shared_ptr<ParameterManager> _parameterManager;
    
    // State
    std::atomic<bool> _isConnected{false};
    std::atomic<bool> _shouldRun{true};
    std::atomic<int> _currentMissionItem{-1};
    
    // Timeout checking
    std::thread _timeoutCheckThread;
    void timeoutCheckLoop();
    
    // Message handling
    void handleMavlinkMessage(const mavlink_message_t& message);
    
    // Helper to execute commands with promise
    std::shared_ptr<Promise<bool>> executeCommand(
        uint16_t command,
        float param1 = 0, float param2 = 0, float param3 = 0, float param4 = 0,
        float param5 = 0, float param6 = 0, float param7 = 0
    );
};

} // namespace margelo::nitro::mavlink