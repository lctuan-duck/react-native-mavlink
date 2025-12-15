#pragma once

#include "HybridMAVLinkSpec.hpp"
#include "core/MAVLinkCore.hpp"
#include <vector>
#include <unordered_map>
#include <memory>

namespace margelo::nitro::mavlink
{
  using namespace margelo::nitro;

  class HybirdMAVLink : public HybridMAVLinkSpec
  {
  public:
    HybirdMAVLink();
    ~HybirdMAVLink() override;

    // Methods
    std::shared_ptr<Promise<void>> startUdp(const UdpOptions &options) override;
    std::shared_ptr<Promise<void>> stopUdp() override;
    std::shared_ptr<Promise<void>> startTcp(const TcpOptions &options) override;
    std::shared_ptr<Promise<void>> stopTcp() override;
    std::shared_ptr<Promise<std::shared_ptr<ArrayBuffer>>> encode(double messageId, const std::shared_ptr<ArrayBuffer> &payload) override;
    std::shared_ptr<Promise<DecodedMessage>> decode(const std::shared_ptr<ArrayBuffer> &raw) override;
    std::shared_ptr<Promise<void>> sendCommandLong(const CommandLongArgs &args) override;
    std::shared_ptr<Promise<void>> sendCommandInt(const CommandIntArgs &args) override;
    std::shared_ptr<Promise<void>> requestParams() override;
    std::shared_ptr<Promise<void>> setParam(const std::string &name, const std::variant<std::string, double> &value) override;
    // Mission
    std::shared_ptr<Promise<void>> requestMissionList() override;
    std::shared_ptr<Promise<void>> sendMissionCount(double count) override;
    std::shared_ptr<Promise<void>> sendMissionItemInt(const MissionItemInt &item) override;
    std::shared_ptr<Promise<void>> clearAllMissions() override;
    std::shared_ptr<Promise<void>> setCurrentMission(double seq) override;
    std::shared_ptr<Promise<void>> setMissionUpload(const std::vector<MissionItemInt> &items) override;
    std::shared_ptr<Promise<void>> enableAutoMissionUpload(bool enable) override;
    double onRaw(const std::function<void(const std::shared_ptr<ArrayBuffer> &)> &listener) override;
    void offRaw(double token) override;
    double onHeartbeat(const std::function<void(const HeartbeatEvent &)> &listener) override;
    void offHeartbeat(double token) override;
    double onStatus(const std::function<void(const StatusEvent &)> &listener) override;
    void offStatus(double token) override;
    double onTelemetry(const std::function<void(const TelemetryEvent &)> &listener) override;
    void offTelemetry(double token) override;
    double onGps(const std::function<void(const GpsEvent &)> &listener) override;
    void offGps(double token) override;
    double onAttitude(const std::function<void(const AttitudeEvent &)> &listener) override;
    void offAttitude(double token) override;
    double onBattery(const std::function<void(const BatteryEvent &)> &listener) override;
    void offBattery(double token) override;
    double onAck(const std::function<void(const AckEvent &)> &listener) override;
    void offAck(double token) override;
    double onParameter(const std::function<void(const ParameterEvent &)> &listener) override;
    void offParameter(double token) override;
    std::shared_ptr<Promise<std::variant<nitro::NullType, TelemetryEvent>>> getTelemetrySnapshot() override;
    double onMode(const std::function<void(const ModeEvent &)> &listener) override;
    void offMode(double token) override;
    double onArm(const std::function<void(const ArmEvent &)> &listener) override;
    void offArm(double token) override;
    double onConnect(const std::function<void()> &listener) override;
    void offConnect(double token) override;
    double onDisconnect(const std::function<void(const std::optional<std::string> &)> &listener) override;
    void offDisconnect(double token) override;
    
    // Mission subscriptions
    double onMissionCount(const std::function<void(const MissionCountEvent &)> &listener) override;
    void offMissionCount(double token) override;
    double onMissionRequest(const std::function<void(const MissionRequestEvent &)> &listener) override;
    void offMissionRequest(double token) override;
    double onMissionItem(const std::function<void(const MissionItemEvent &)> &listener) override;
    void offMissionItem(double token) override;
    double onMissionAck(const std::function<void(const MissionAckEvent &)> &listener) override;
    void offMissionAck(double token) override;
    
    // Logging helpers
    std::shared_ptr<Promise<void>> requestLogList() override;
    std::shared_ptr<Promise<void>> requestLogData(const LogDataRequestArgs& args) override;
    std::shared_ptr<Promise<void>> requestDataTransmissionHandshake(const DataTransmissionHandshakeArgs& args) override;

    // Subscriptions for Logging
    double onLogging(const std::function<void(const LoggingEvent &)> &listener) override;
    void offLogging(double token) override;

    // Subscriptions for Camera/Gimbal
    double onCameraStatus(const std::function<void(const CameraStatusEvent &)> &listener) override;
    void offCameraStatus(double token) override;
    double onCameraCaptureStatus(const std::function<void(const CameraCaptureStatusEvent &)> &listener) override;
    void offCameraCaptureStatus(double token) override;
    double onGimbalDeviceAttitudeStatus(const std::function<void(const GimbalDeviceAttitudeStatusEvent &)> &listener) override;
    void offGimbalDeviceAttitudeStatus(double token) override;

  private:
    // Listener stores
    double nextToken = 1.0;
    std::unordered_map<double, std::function<void(const std::shared_ptr<ArrayBuffer> &)>> rawListeners;
    std::unordered_map<double, std::function<void(const HeartbeatEvent &)>> heartbeatListeners;
    std::unordered_map<double, std::function<void(const StatusEvent &)>> statusListeners;
    std::unordered_map<double, std::function<void(const TelemetryEvent &)>> telemetryListeners;
    std::unordered_map<double, std::function<void(const GpsEvent &)>> gpsListeners;
    std::unordered_map<double, std::function<void(const AttitudeEvent &)>> attitudeListeners;
    std::unordered_map<double, std::function<void(const BatteryEvent &)>> batteryListeners;
    std::unordered_map<double, std::function<void(const AckEvent &)>> ackListeners;
    std::unordered_map<double, std::function<void(const ParameterEvent &)>> parameterListeners;
    std::unordered_map<double, std::function<void(const ModeEvent &)>> modeListeners;
    std::unordered_map<double, std::function<void(const ArmEvent &)>> armListeners;
    std::unordered_map<double, std::function<void()>> connectListeners;
    std::unordered_map<double, std::function<void(const std::optional<std::string> &)>> disconnectListeners;
    std::unordered_map<double, std::function<void(const MissionCountEvent &)>> missionCountListeners;
    std::unordered_map<double, std::function<void(const MissionRequestEvent &)>> missionRequestListeners;
    std::unordered_map<double, std::function<void(const MissionItemEvent &)>> missionItemListeners;
    std::unordered_map<double, std::function<void(const MissionAckEvent &)>> missionAckListeners;
    // Logging listeners
    std::unordered_map<double, std::function<void(const LoggingEvent &)>> loggingListeners;
    // Camera/Gimbal listeners
    std::unordered_map<double, std::function<void(const CameraStatusEvent &)>> cameraStatusListeners;
    std::unordered_map<double, std::function<void(const CameraCaptureStatusEvent &)>> cameraCaptureStatusListeners;
    std::unordered_map<double, std::function<void(const GimbalDeviceAttitudeStatusEvent &)>> gimbalDeviceAttitudeStatusListeners;
    
    // Core instance
    std::unique_ptr<MAVLinkCore> core_;
    
    // Helper methods
    void initCore();
  };
}

}
