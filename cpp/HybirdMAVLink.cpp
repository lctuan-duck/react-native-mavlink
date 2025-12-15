#include "HybirdMAVLink.hpp"
#include "core/MAVLinkCore.hpp"
#include <NitroModules/ArrayBuffer.hpp>

namespace margelo::nitro::mavlink
{
  // Constructor & Destructor
  HybirdMAVLink::HybirdMAVLink() : HybridMAVLinkSpec()
  {
    initCore();
  }

  HybirdMAVLink::~HybirdMAVLink()
  {
    if (core_)
    {
      core_->stopUDP();
      core_->stopTCP();
    }
  }

  // Core initialization with event wiring
  void HybirdMAVLink::initCore()
  {
    core_ = std::make_unique<MAVLinkCore>();
    
    // Setup event callbacks - wire core events to listener maps
    core_->onEvent("heartbeat", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.heartbeat.has_value()) {
          HeartbeatEvent ev{};
          ev.type = telemetry.heartbeat->type;
          ev.autopilot = telemetry.heartbeat->autopilot;
          ev.baseMode = telemetry.heartbeat->baseMode;
          ev.customMode = telemetry.heartbeat->customMode;
          ev.systemStatus = telemetry.heartbeat->systemStatus;
          ev.systemId = telemetry.heartbeat->systemId;
          ev.componentId = telemetry.heartbeat->componentId;
          ev.timestampMs = telemetry.heartbeat->timestampMs;
          
          for (auto& kv : heartbeatListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    core_->onEvent("gps", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.gps.has_value()) {
          // GPS event
          GpsEvent ev{};
          ev.latitude = telemetry.gps->latitude;
          ev.longitude = telemetry.gps->longitude;
          ev.altitude = telemetry.gps->altitude;
          ev.fixType = telemetry.gps->fixType;
          ev.satellitesVisible = telemetry.gps->satellitesVisible;
          ev.timestampMs = telemetry.gps->timestampMs;
          
          for (auto& kv : gpsListeners) {
            kv.second(ev);
          }
          
          // Also emit aggregated telemetry event
          TelemetryEvent telEv{};
          
          if (telemetry.attitude.has_value()) {
            Attitude att{};
            att.roll = telemetry.attitude->roll;
            att.pitch = telemetry.attitude->pitch;
            att.yaw = telemetry.attitude->yaw;
            telEv.attitude = att;
          }
          
          GPSInfo gpsInfo{};
          gpsInfo.latE7 = static_cast<int32_t>(telemetry.gps->latitude * 1e7);
          gpsInfo.lonE7 = static_cast<int32_t>(telemetry.gps->longitude * 1e7);
          gpsInfo.altMm = static_cast<int32_t>(telemetry.gps->altitude * 1000);
          gpsInfo.satellites = telemetry.gps->satellitesVisible;
          gpsInfo.fixType = telemetry.gps->fixType;
          telEv.gps = gpsInfo;
          telEv.timestampMs = telemetry.gps->timestampMs;
          
          for (auto& kv : telemetryListeners) {
            kv.second(telEv);
          }
        }
      } catch(...) {}
    });
    
    core_->onEvent("attitude", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.attitude.has_value()) {
          // Attitude event
          AttitudeEvent ev{};
          ev.roll = telemetry.attitude->roll;
          ev.pitch = telemetry.attitude->pitch;
          ev.yaw = telemetry.attitude->yaw;
          ev.rollspeed = telemetry.attitude->rollspeed;
          ev.pitchspeed = telemetry.attitude->pitchspeed;
          ev.yawspeed = telemetry.attitude->yawspeed;
          ev.timestampMs = telemetry.attitude->timestampMs;
          
          for (auto& kv : attitudeListeners) {
            kv.second(ev);
          }
          
          // Also emit aggregated telemetry event
          TelemetryEvent telEv{};
          
          Attitude att{};
          att.roll = telemetry.attitude->roll;
          att.pitch = telemetry.attitude->pitch;
          att.yaw = telemetry.attitude->yaw;
          telEv.attitude = att;
          
          if (telemetry.gps.has_value()) {
            GPSInfo gpsInfo{};
            gpsInfo.latE7 = static_cast<int32_t>(telemetry.gps->latitude * 1e7);
            gpsInfo.lonE7 = static_cast<int32_t>(telemetry.gps->longitude * 1e7);
            gpsInfo.altMm = static_cast<int32_t>(telemetry.gps->altitude * 1000);
            gpsInfo.satellites = telemetry.gps->satellitesVisible;
            gpsInfo.fixType = telemetry.gps->fixType;
            telEv.gps = gpsInfo;
          }
          
          telEv.timestampMs = telemetry.attitude->timestampMs;
          
          for (auto& kv : telemetryListeners) {
            kv.second(telEv);
          }
        }
      } catch(...) {}
    });
    
    core_->onEvent("battery", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.battery.has_value()) {
          // Battery event
          BatteryEvent ev{};
          ev.voltage = telemetry.battery->voltage;
          ev.current = telemetry.battery->current;
          ev.remaining = telemetry.battery->remaining;
          ev.timestampMs = telemetry.battery->timestampMs;
          
          for (auto& kv : batteryListeners) {
            kv.second(ev);
          }
          
          // Also emit status event
          StatusEvent statusEv{};
          BatteryInfo batInfo{};
          batInfo.voltage = telemetry.battery->voltage;
          batInfo.current = telemetry.battery->current;
          batInfo.remaining = telemetry.battery->remaining;
          statusEv.battery = batInfo;
          statusEv.timestampMs = telemetry.battery->timestampMs;
          
          for (auto& kv : statusListeners) {
            kv.second(statusEv);
          }
        }
      } catch(...) {}
    });
    
    // Connect/disconnect events
    core_->onEvent("connect", [this](const std::string&, const std::any&) {
      for (auto& kv : connectListeners) {
        kv.second();
      }
    });
    
    core_->onEvent("disconnect", [this](const std::string&, const std::any& data) {
      std::optional<std::string> reason;
      try {
        reason = std::any_cast<std::string>(data);
      } catch(...) {}
      
      for (auto& kv : disconnectListeners) {
        kv.second(reason);
      }
    });
    
    // Command ACK event
    core_->onEvent("ack", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.commandAck.has_value()) {
          AckEvent ev{};
          ev.command = telemetry.commandAck->command;
          ev.result = telemetry.commandAck->result;
          ev.timestampMs = telemetry.commandAck->timestampMs;
          
          for (auto& kv : ackListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    // Parameter event
    core_->onEvent("parameter", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.parameter.has_value()) {
          ParameterEvent ev{};
          ev.name = telemetry.parameter->name;
          ev.value = telemetry.parameter->value;
          ev.type = telemetry.parameter->type;
          ev.index = telemetry.parameter->index;
          
          for (auto& kv : parameterListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    // Mode change event
    core_->onEvent("mode", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.modeChange.has_value()) {
          ModeEvent ev{};
          ev.baseMode = telemetry.modeChange->baseMode;
          ev.customMode = telemetry.modeChange->customMode;
          ev.timestampMs = telemetry.modeChange->timestampMs;
          
          for (auto& kv : modeListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    // Arm/disarm event
    core_->onEvent("arm", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.armChange.has_value()) {
          ArmEvent ev{};
          ev.armed = telemetry.armChange->armed;
          ev.timestampMs = telemetry.armChange->timestampMs;
          
          for (auto& kv : armListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    // Mission events
    core_->onEvent("missionCount", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.missionCount.has_value()) {
          MissionCountEvent ev{};
          ev.targetSystem = telemetry.missionCount->targetSystem;
          ev.targetComponent = telemetry.missionCount->targetComponent;
          ev.count = telemetry.missionCount->count;
          ev.timestampMs = telemetry.missionCount->timestampMs;
          
          for (auto& kv : missionCountListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    core_->onEvent("missionRequest", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.missionRequest.has_value()) {
          MissionRequestEvent ev{};
          ev.targetSystem = telemetry.missionRequest->targetSystem;
          ev.targetComponent = telemetry.missionRequest->targetComponent;
          ev.seq = telemetry.missionRequest->seq;
          ev.timestampMs = telemetry.missionRequest->timestampMs;
          
          for (auto& kv : missionRequestListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    core_->onEvent("missionItem", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.missionItemInt.has_value()) {
          MissionItemEvent ev{};
          MissionItemInt item{};
          item.seq = telemetry.missionItemInt->seq;
          item.frame = telemetry.missionItemInt->frame;
          item.command = telemetry.missionItemInt->command;
          item.current = telemetry.missionItemInt->current;
          item.autocontinue = telemetry.missionItemInt->autocontinue;
          item.param1 = telemetry.missionItemInt->param1;
          item.param2 = telemetry.missionItemInt->param2;
          item.param3 = telemetry.missionItemInt->param3;
          item.param4 = telemetry.missionItemInt->param4;
          item.x = telemetry.missionItemInt->x;
          item.y = telemetry.missionItemInt->y;
          item.z = telemetry.missionItemInt->z;
          ev.item = item;
          ev.timestampMs = telemetry.missionItemInt->timestampMs;
          
          for (auto& kv : missionItemListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    core_->onEvent("missionAck", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        if (telemetry.missionAck.has_value()) {
          MissionAckEvent ev{};
          ev.type = telemetry.missionAck->type;
          ev.timestampMs = telemetry.missionAck->timestampMs;
          
          for (auto& kv : missionAckListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    // Logging event (combines log entry and log data)
    core_->onEvent("logging", [this](const std::string&, const std::any& data) {
      try {
        const auto& telemetry = std::any_cast<const TelemetryState&>(data);
        LoggingEvent ev{};
        
        if (telemetry.logEntry.has_value()) {
          LogEntry entry{};
          entry.id = telemetry.logEntry->id;
          entry.numLogs = telemetry.logEntry->numLogs;
          entry.lastLogNum = telemetry.logEntry->lastLogNum;
          entry.timeUtc = telemetry.logEntry->timeUtc;
          entry.size = telemetry.logEntry->size;
          ev.entry = entry;
          ev.timestampMs = telemetry.logEntry->timestampMs;
        }
        
        if (telemetry.logData.has_value()) {
          LogDataChunk chunk{};
          chunk.id = telemetry.logData->id;
          chunk.ofs = telemetry.logData->ofs;
          auto dataBuffer = ArrayBuffer::allocate(telemetry.logData->count);
          std::memcpy(dataBuffer->data(), telemetry.logData->data, telemetry.logData->count);
          chunk.data = dataBuffer;
          ev.data = chunk;
          ev.timestampMs = telemetry.logData->timestampMs;
        }
        
        if (ev.entry.has_value() || ev.data.has_value()) {
          for (auto& kv : loggingListeners) {
            kv.second(ev);
          }
        }
      } catch(...) {}
    });
    
    // Camera/Gimbal events (stub handlers - can be extended with actual message parsing)
    // These are less commonly used, so keeping as stubs for now
    core_->onEvent("cameraStatus", [this](const std::string&, const std::any&) {
      // Camera status message parsing not yet implemented
      // User can extend by adding CAMERA_INFORMATION, CAMERA_SETTINGS parsing
    });
    
    core_->onEvent("cameraCaptureStatus", [this](const std::string&, const std::any&) {
      // Camera capture status parsing not yet implemented
      // User can extend by adding CAMERA_CAPTURE_STATUS parsing
    });
    
    core_->onEvent("gimbalDeviceAttitudeStatus", [this](const std::string&, const std::any&) {
      // Gimbal attitude parsing not yet implemented
      // User can extend by adding GIMBAL_DEVICE_ATTITUDE_STATUS parsing
    });
  }

  // Telemetry snapshot
  std::shared_ptr<Promise<std::variant<nitro::NullType, TelemetryEvent>>> HybirdMAVLink::getTelemetrySnapshot()
  {
    auto promise = std::make_shared<Promise<std::variant<nitro::NullType, TelemetryEvent>>>();
    
    if (!core_) {
      promise->resolve(nitro::NullType{});
      return promise;
    }
    
    try {
      auto state = core_->getTelemetrySnapshot();
      TelemetryEvent ev{};
      
      // Build nested GPS object
      if (state.gps.has_value()) {
        GPSInfo gps{};
        gps.latE7 = static_cast<int32_t>(state.gps->latitude * 1e7);
        gps.lonE7 = static_cast<int32_t>(state.gps->longitude * 1e7);
        gps.altMm = static_cast<int32_t>(state.gps->altitude * 1000);
        gps.satellites = state.gps->satellitesVisible;
        gps.fixType = state.gps->fixType;
        ev.gps = gps;
      }
      
      // Build nested Attitude object
      if (state.attitude.has_value()) {
        Attitude attitude{};
        attitude.roll = state.attitude->roll;
        attitude.pitch = state.attitude->pitch;
        attitude.yaw = state.attitude->yaw;
        ev.attitude = attitude;
      }
      
      // VFR_HUD and LocalNED not implemented in state yet
      // ev.vfrHud = ...
      // ev.localNed = ...
      
      // Set timestamp
      ev.timestampMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        state.lastUpdate.time_since_epoch()
      ).count());
      
      promise->resolve(ev);
    } catch (const std::exception& e) {
      promise->resolve(nitro::NullType{});
    }
    
    return promise;
  }

} // namespace margelo::nitro::mavlink

// Implementation organized by functionality in bridge/ folder:
// - bridge/transport/Transport.cpp: startUdp, stopUdp, startTcp, stopTcp
// - bridge/commands/Commands.cpp: encode, decode, sendCommandLong, sendCommandInt
// - bridge/parameters/Parameters.cpp: requestParams, setParam
// - bridge/events/Events.cpp: All onXXX/offXXX event listener methods
// - bridge/mission/Mission.cpp: Mission management methods and events
// - bridge/logging/Logging.cpp: Logging and data transmission methods
