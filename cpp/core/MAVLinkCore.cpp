#include "MAVLinkCore.hpp"
#include "../transport/UDPTransport.hpp"
#include "../transport/TCPTransport.hpp"
#include "../parser/MessageParser.hpp"
#include "../telemetry/TelemetryManager.hpp"
#include "../events/EventEmitter.hpp"
#include <memory>

namespace margelo::nitro::mavlink {

class MAVLinkCore::Impl {
public:
    std::unique_ptr<UDPTransport> udpTransport;
    std::unique_ptr<TCPTransport> tcpTransport;
    std::unique_ptr<MessageParser> parser;
    std::unique_ptr<TelemetryManager> telemetry;
    std::unique_ptr<EventEmitter> eventEmitter;
    
    RawDataCallback rawDataCallback;
    EventCallback eventCallback;
    ErrorCallback errorCallback;
    
    // Support for typed event callbacks with std::any
    std::map<std::string, std::vector<std::function<void(const std::string&, const std::any&)>>> anyEventCallbacks;
    
    // State tracking for mode and arm detection
    std::optional<uint8_t> lastBaseMode;
    std::optional<uint32_t> lastCustomMode;
    std::optional<bool> lastArmedState;
    
    void handleIncomingData(const std::vector<uint8_t>& data) {
        // Notify raw data
        if (rawDataCallback) {
            rawDataCallback(data);
        }
        
        // Parse message
        auto parsed = parser->parse(data.data(), data.size());
        if (!parsed) return;
        
        // Process parsed message
        processParsedMessage(*parsed);
    }
    
    void processParsedMessage(const ParsedMessage& msg) {
        // Update telemetry and emit events for each message type
        if (msg.heartbeat.has_value()) {
            HeartbeatData hb;
            hb.systemId = msg.systemId;
            hb.componentId = msg.componentId;
            hb.autopilot = msg.heartbeat->autopilot;
            hb.type = msg.heartbeat->type;
            hb.baseMode = msg.heartbeat->base_mode;
            hb.customMode = msg.heartbeat->custom_mode;
            hb.systemStatus = msg.heartbeat->system_status;
            hb.timestampMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count());
            telemetry->updateHeartbeat(hb);
            
            // Get fresh snapshot and emit
            TelemetryState telState = telemetry->getSnapshot();
            emitAnyEvent("heartbeat", telState);
            
            if (eventCallback) {
                eventCallback("heartbeat", "{}");
            }
            
            // Detect mode changes
            bool modeChanged = false;
            if (lastBaseMode.has_value() && lastCustomMode.has_value()) {
                if (lastBaseMode.value() != hb.baseMode || lastCustomMode.value() != hb.customMode) {
                    modeChanged = true;
                }
            }
            lastBaseMode = hb.baseMode;
            lastCustomMode = hb.customMode;
            
            if (modeChanged) {
                TelemetryState modeState = telemetry->getSnapshot();
                modeState.modeChange = ModeChangeData{
                    hb.baseMode,
                    hb.customMode,
                    hb.timestampMs
                };
                emitAnyEvent("mode", modeState);
            }
            
            // Detect arm/disarm changes (bit 7 of base_mode is armed flag)
            bool currentArmed = (hb.baseMode & 0x80) != 0;
            bool armChanged = false;
            if (lastArmedState.has_value()) {
                if (lastArmedState.value() != currentArmed) {
                    armChanged = true;
                }
            }
            lastArmedState = currentArmed;
            
            if (armChanged) {
                TelemetryState armState = telemetry->getSnapshot();
                armState.armChange = ArmChangeData{
                    currentArmed,
                    hb.timestampMs
                };
                emitAnyEvent("arm", armState);
            }
        }
        
        // GPS handling: prefer GLOBAL_POSITION_INT (has velocity) over GPS_RAW_INT
        bool gpsUpdated = false;
        
        if (msg.gpsPosition.has_value()) {
            GPSData gps;
            gps.latitude = msg.gpsPosition->lat / 1e7;
            gps.longitude = msg.gpsPosition->lon / 1e7;
            gps.altitude = msg.gpsPosition->alt / 1000.0;
            gps.vx = msg.gpsPosition->vx / 100.0;
            gps.vy = msg.gpsPosition->vy / 100.0;
            gps.vz = msg.gpsPosition->vz / 100.0;
            gps.timestampMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count());
            telemetry->updateGPS(gps);
            gpsUpdated = true;
            
            if (eventCallback) {
                eventCallback("gps", "{}");
            }
        } else if (msg.gpsRaw.has_value()) {
            // Only use GPS_RAW_INT if GLOBAL_POSITION_INT not available
            GPSData gps;
            gps.latitude = msg.gpsRaw->lat / 1e7;
            gps.longitude = msg.gpsRaw->lon / 1e7;
            gps.altitude = msg.gpsRaw->alt / 1000.0;
            gps.vx = 0;  // GPS_RAW_INT doesn't have velocity
            gps.vy = 0;
            gps.vz = 0;
            gps.satellitesVisible = msg.gpsRaw->satellites_visible;
            gps.fixType = msg.gpsRaw->fix_type;
            gps.timestampMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count());
            telemetry->updateGPS(gps);
            gpsUpdated = true;
        }
        
        // Emit GPS event once after update
        if (gpsUpdated) {
            TelemetryState telState = telemetry->getSnapshot();
            emitAnyEvent("gps", telState);
        }
        
        if (msg.attitude.has_value()) {
            AttitudeData att;
            att.roll = msg.attitude->roll;
            att.pitch = msg.attitude->pitch;
            att.yaw = msg.attitude->yaw;
            att.rollspeed = msg.attitude->rollspeed;
            att.pitchspeed = msg.attitude->pitchspeed;
            att.yawspeed = msg.attitude->yawspeed;
            att.timestampMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count());
            telemetry->updateAttitude(att);
            
            // Get fresh snapshot and emit
            TelemetryState telState = telemetry->getSnapshot();
            emitAnyEvent("attitude", telState);
            
            if (eventCallback) {
                eventCallback("attitude", "{}");
            }
        }
        
        if (msg.sysStatus.has_value()) {
            BatteryData bat;
            bat.voltage = msg.sysStatus->voltage_battery / 1000.0;
            bat.current = msg.sysStatus->current_battery / 100.0;
            bat.remaining = msg.sysStatus->battery_remaining;
            bat.timestampMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count());
            telemetry->updateBattery(bat);
            
            // Get fresh snapshot and emit
            TelemetryState telState = telemetry->getSnapshot();
            emitAnyEvent("battery", telState);
            
            if (eventCallback) {
                eventCallback("battery", "{}");
            }
        }
        
        if (msg.commandAck.has_value()) {
            TelemetryState telState = telemetry->getSnapshot();
            telState.commandAck = CommandAckData{
                msg.commandAck->command,
                msg.commandAck->result,
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count())
            };
            emitAnyEvent("ack", telState);
        }
        
        if (msg.paramValue.has_value()) {
            TelemetryState telState = telemetry->getSnapshot();
            char paramName[17] = {0};
            std::memcpy(paramName, msg.paramValue->param_id, 16);
            
            telState.parameter = ParameterData{
                std::string(paramName),
                msg.paramValue->param_value,
                msg.paramValue->param_type,
                msg.paramValue->param_index
            };
            emitAnyEvent("parameter", telState);
        }
        
        if (msg.missionCount.has_value()) {
            TelemetryState telState = telemetry->getSnapshot();
            telState.missionCount = MissionCountData{
                msg.missionCount->target_system,
                msg.missionCount->target_component,
                msg.missionCount->count,
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count())
            };
            emitAnyEvent("missionCount", telState);
        }
        
        if (msg.missionRequest.has_value()) {
            TelemetryState telState = telemetry->getSnapshot();
            telState.missionRequest = MissionRequestData{
                msg.missionRequest->target_system,
                msg.missionRequest->target_component,
                msg.missionRequest->seq,
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count())
            };
            emitAnyEvent("missionRequest", telState);
        }
        
        if (msg.missionItemInt.has_value()) {
            TelemetryState telState = telemetry->getSnapshot();
            telState.missionItemInt = MissionItemIntData{
                msg.missionItemInt->seq,
                msg.missionItemInt->frame,
                msg.missionItemInt->command,
                msg.missionItemInt->current,
                msg.missionItemInt->autocontinue,
                msg.missionItemInt->param1,
                msg.missionItemInt->param2,
                msg.missionItemInt->param3,
                msg.missionItemInt->param4,
                msg.missionItemInt->x,
                msg.missionItemInt->y,
                msg.missionItemInt->z,
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count())
            };
            emitAnyEvent("missionItem", telState);
        }
        
        if (msg.missionAck.has_value()) {
            TelemetryState telState = telemetry->getSnapshot();
            telState.missionAck = MissionAckData{
                msg.missionAck->type,
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count())
            };
            emitAnyEvent("missionAck", telState);
        }
        
        if (msg.logEntry.has_value()) {
            TelemetryState telState = telemetry->getSnapshot();
            telState.logEntry = LogEntryData{
                msg.logEntry->id,
                msg.logEntry->num_logs,
                msg.logEntry->last_log_num,
                msg.logEntry->time_utc,
                msg.logEntry->size,
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count())
            };
            
            // Emit to logging event
            emitAnyEvent("logging", telState);
        }
        
        if (msg.logData.has_value()) {
            TelemetryState telState = telemetry->getSnapshot();
            LogDataData logDataVal{};
            logDataVal.id = msg.logData->id;
            logDataVal.ofs = msg.logData->ofs;
            logDataVal.count = msg.logData->count;
            std::memcpy(logDataVal.data, msg.logData->data, msg.logData->count);
            logDataVal.timestampMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count());
            telState.logData = logDataVal;
            
            // Emit to logging event
            emitAnyEvent("logging", telState);
        }
    }
    
    void emitAnyEvent(const std::string& eventName, const TelemetryState& data) {
        auto it = anyEventCallbacks.find(eventName);
        if (it != anyEventCallbacks.end()) {
            for (auto& callback : it->second) {
                callback(eventName, data);
            }
        }
    }
};

MAVLinkCore::MAVLinkCore() : impl_(std::make_unique<Impl>()) {
    impl_->parser = std::make_unique<MessageParser>();
    impl_->telemetry = std::make_unique<TelemetryManager>();
    impl_->eventEmitter = std::make_unique<EventEmitter>();
}

MAVLinkCore::~MAVLinkCore() {
    stopUDP();
    stopTCP();
}

bool MAVLinkCore::startUDP(const UdpOptions& options) {
    if (impl_->udpTransport && impl_->udpTransport->isRunning()) {
        return false;
    }
    
    impl_->udpTransport = std::make_unique<UDPTransport>(options.port, options.host);
    impl_->udpTransport->setRemote(options.remoteHost, options.remotePort);
    
    impl_->udpTransport->setDataCallback([this](const std::vector<uint8_t>& data) {
        impl_->handleIncomingData(data);
    });
    
    impl_->udpTransport->setErrorCallback([this](const std::string& error) {
        if (impl_->errorCallback) {
            impl_->errorCallback(error);
        }
    });
    
    bool started = impl_->udpTransport->start();
    if (started) {
        // Emit connect event with empty TelemetryState
        impl_->emitAnyEvent("connect", TelemetryState{});
    }
    return started;
}

void MAVLinkCore::stopUDP() {
    if (impl_->udpTransport) {
        impl_->udpTransport->stop();
        impl_->udpTransport.reset();
        // Emit disconnect event with empty TelemetryState
        impl_->emitAnyEvent("disconnect", TelemetryState{});
    }
}

bool MAVLinkCore::startTCP(const TcpOptions& options) {
    if (impl_->tcpTransport && impl_->tcpTransport->isRunning()) {
        return false;
    }
    
    impl_->tcpTransport = std::make_unique<TCPTransport>(options.host, options.port);
    
    impl_->tcpTransport->setDataCallback([this](const std::vector<uint8_t>& data) {
        impl_->handleIncomingData(data);
    });
    
    impl_->tcpTransport->setErrorCallback([this](const std::string& error) {
        if (impl_->errorCallback) {
            impl_->errorCallback(error);
        }
    });
    
    bool started = impl_->tcpTransport->start();
    if (started) {
        // Emit connect event with empty TelemetryState
        impl_->emitAnyEvent("connect", TelemetryState{});
    }
    return started;
}

void MAVLinkCore::stopTCP() {
    if (impl_->tcpTransport) {
        impl_->tcpTransport->stop();
        impl_->tcpTransport.reset();
        // Emit disconnect event with empty TelemetryState
        impl_->emitAnyEvent("disconnect", TelemetryState{});
    }
}

bool MAVLinkCore::sendData(const uint8_t* data, size_t length) {
    if (impl_->tcpTransport && impl_->tcpTransport->isRunning()) {
        return impl_->tcpTransport->send(data, length);
    }
    if (impl_->udpTransport && impl_->udpTransport->isRunning()) {
        return impl_->udpTransport->send(data, length);
    }
    return false;
}

TelemetryState MAVLinkCore::getTelemetrySnapshot() const {
    return impl_->telemetry->getSnapshot();
}

void MAVLinkCore::setRawDataCallback(RawDataCallback callback) {
    impl_->rawDataCallback = std::move(callback);
}

void MAVLinkCore::setEventCallback(EventCallback callback) {
    impl_->eventCallback = std::move(callback);
}

void MAVLinkCore::onEvent(const std::string& eventName, AnyEventCallback callback) {
    impl_->anyEventCallbacks[eventName].push_back(std::move(callback));
}

void MAVLinkCore::setErrorCallback(ErrorCallback callback) {
    impl_->errorCallback = std::move(callback);
}

} // namespace margelo::nitro::mavlink
