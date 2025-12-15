#pragma once
#include <optional>
#include <chrono>
#include <cstdint>
#include <string>

namespace margelo::nitro::mavlink {

struct GPSData {
    double latitude{0};
    double longitude{0};
    double altitude{0};
    double vx{0};
    double vy{0};
    double vz{0};
    uint8_t satellitesVisible{0};
    uint8_t fixType{0};
    uint64_t timestampMs{0};
};

struct AttitudeData {
    double roll{0};
    double pitch{0};
    double yaw{0};
    double rollspeed{0};
    double pitchspeed{0};
    double yawspeed{0};
    uint64_t timestampMs{0};
};

struct BatteryData {
    double voltage{0};
    double current{0};
    double remaining{0};
    uint64_t timestampMs{0};
};

struct HeartbeatData {
    uint8_t systemId{0};
    uint8_t componentId{0};
    uint8_t autopilot{0};
    uint8_t type{0};
    uint8_t baseMode{0};
    uint32_t customMode{0};
    uint8_t systemStatus{0};
    uint64_t timestampMs{0};
};

struct CommandAckData {
    uint16_t command{0};
    uint8_t result{0};
    uint64_t timestampMs{0};
};

struct ParameterData {
    std::string name;
    float value{0};
    uint8_t type{0};
    uint16_t index{0};
};

struct ModeChangeData {
    uint8_t baseMode{0};
    uint32_t customMode{0};
    uint64_t timestampMs{0};
};

struct ArmChangeData {
    bool armed{false};
    uint64_t timestampMs{0};
};

struct MissionCountData {
    uint8_t targetSystem{0};
    uint8_t targetComponent{0};
    uint16_t count{0};
    uint64_t timestampMs{0};
};

struct MissionRequestData {
    uint8_t targetSystem{0};
    uint8_t targetComponent{0};
    uint16_t seq{0};
    uint64_t timestampMs{0};
};

struct MissionItemIntData {
    uint16_t seq{0};
    uint8_t frame{0};
    uint16_t command{0};
    uint8_t current{0};
    uint8_t autocontinue{0};
    float param1{0};
    float param2{0};
    float param3{0};
    float param4{0};
    int32_t x{0};
    int32_t y{0};
    float z{0};
    uint64_t timestampMs{0};
};

struct MissionAckData {
    uint8_t type{0};
    uint64_t timestampMs{0};
};

struct LogEntryData {
    uint16_t id{0};
    uint16_t numLogs{0};
    uint16_t lastLogNum{0};
    uint32_t timeUtc{0};
    uint32_t size{0};
    uint64_t timestampMs{0};
};

struct LogDataData {
    uint16_t id{0};
    uint32_t ofs{0};
    uint8_t count{0};
    uint8_t data[90];  // MAX_LOG_DATA_LEN
    uint64_t timestampMs{0};
};

struct TelemetryState {
    std::optional<GPSData> gps;
    std::optional<AttitudeData> attitude;
    std::optional<BatteryData> battery;
    std::optional<HeartbeatData> heartbeat;
    std::optional<CommandAckData> commandAck;
    std::optional<ParameterData> parameter;
    std::optional<ModeChangeData> modeChange;
    std::optional<ArmChangeData> armChange;
    std::optional<MissionCountData> missionCount;
    std::optional<MissionRequestData> missionRequest;
    std::optional<MissionItemIntData> missionItemInt;
    std::optional<MissionAckData> missionAck;
    std::optional<LogEntryData> logEntry;
    std::optional<LogDataData> logData;
    std::chrono::system_clock::time_point lastUpdate;
};

} // namespace margelo::nitro::mavlink
