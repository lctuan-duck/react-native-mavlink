#include "../../HybirdMAVLink.hpp"
#include <NitroModules/ArrayBuffer.hpp>

namespace margelo::nitro::mavlink
{
  // Raw data events
  double HybirdMAVLink::onRaw(const std::function<void(const std::shared_ptr<ArrayBuffer>&)>& listener)
  {
    double token = nextToken++;
    rawListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offRaw(double token)
  {
    rawListeners.erase(token);
  }

  // Heartbeat events
  double HybirdMAVLink::onHeartbeat(const std::function<void(const HeartbeatEvent&)>& listener)
  {
    double token = nextToken++;
    heartbeatListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offHeartbeat(double token)
  {
    heartbeatListeners.erase(token);
  }

  // Status events
  double HybirdMAVLink::onStatus(const std::function<void(const StatusEvent&)>& listener)
  {
    double token = nextToken++;
    statusListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offStatus(double token)
  {
    statusListeners.erase(token);
  }

  // Telemetry events
  double HybirdMAVLink::onTelemetry(const std::function<void(const TelemetryEvent&)>& listener)
  {
    double token = nextToken++;
    telemetryListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offTelemetry(double token)
  {
    telemetryListeners.erase(token);
  }

  // GPS events
  double HybirdMAVLink::onGps(const std::function<void(const GpsEvent&)>& listener)
  {
    double token = nextToken++;
    gpsListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offGps(double token)
  {
    gpsListeners.erase(token);
  }

  // Attitude events
  double HybirdMAVLink::onAttitude(const std::function<void(const AttitudeEvent&)>& listener)
  {
    double token = nextToken++;
    attitudeListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offAttitude(double token)
  {
    attitudeListeners.erase(token);
  }

  // Battery events
  double HybirdMAVLink::onBattery(const std::function<void(const BatteryEvent&)>& listener)
  {
    double token = nextToken++;
    batteryListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offBattery(double token)
  {
    batteryListeners.erase(token);
  }

  // ACK events
  double HybirdMAVLink::onAck(const std::function<void(const AckEvent&)>& listener)
  {
    double token = nextToken++;
    ackListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offAck(double token)
  {
    ackListeners.erase(token);
  }

  // Parameter events
  double HybirdMAVLink::onParameter(const std::function<void(const ParameterEvent&)>& listener)
  {
    double token = nextToken++;
    parameterListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offParameter(double token)
  {
    parameterListeners.erase(token);
  }

  // Mode events
  double HybirdMAVLink::onMode(const std::function<void(const ModeEvent&)>& listener)
  {
    double token = nextToken++;
    modeListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offMode(double token)
  {
    modeListeners.erase(token);
  }

  // Arm events
  double HybirdMAVLink::onArm(const std::function<void(const ArmEvent&)>& listener)
  {
    double token = nextToken++;
    armListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offArm(double token)
  {
    armListeners.erase(token);
  }

  // Connect/Disconnect events
  double HybirdMAVLink::onConnect(const std::function<void()>& listener)
  {
    double token = nextToken++;
    connectListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offConnect(double token)
  {
    connectListeners.erase(token);
  }

  double HybirdMAVLink::onDisconnect(const std::function<void(const std::optional<std::string>&)>& listener)
  {
    double token = nextToken++;
    disconnectListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offDisconnect(double token)
  {
    disconnectListeners.erase(token);
  }

  // Camera events
  double HybirdMAVLink::onCameraStatus(const std::function<void(const CameraStatusEvent&)>& listener)
  {
    double token = nextToken++;
    cameraStatusListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offCameraStatus(double token)
  {
    cameraStatusListeners.erase(token);
  }

  double HybirdMAVLink::onCameraCaptureStatus(const std::function<void(const CameraCaptureStatusEvent&)>& listener)
  {
    double token = nextToken++;
    cameraCaptureStatusListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offCameraCaptureStatus(double token)
  {
    cameraCaptureStatusListeners.erase(token);
  }

  // Gimbal events
  double HybirdMAVLink::onGimbalDeviceAttitudeStatus(const std::function<void(const GimbalDeviceAttitudeStatusEvent&)>& listener)
  {
    double token = nextToken++;
    gimbalDeviceAttitudeStatusListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offGimbalDeviceAttitudeStatus(double token)
  {
    gimbalDeviceAttitudeStatusListeners.erase(token);
  }

} // namespace margelo::nitro::mavlink
