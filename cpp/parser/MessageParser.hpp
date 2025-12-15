#pragma once
#include "../mavlink/v2.0/common/mavlink.h"
#include <optional>
#include <vector>
#include <cstdint>

namespace margelo::nitro::mavlink {

struct ParsedMessage {
    uint32_t messageId;
    uint8_t systemId;
    uint8_t componentId;
    std::vector<uint8_t> payload;
    
    // Decoded messages
    std::optional<mavlink_heartbeat_t> heartbeat;
    std::optional<mavlink_sys_status_t> sysStatus;
    std::optional<mavlink_attitude_t> attitude;
    std::optional<mavlink_global_position_int_t> gpsPosition;
    std::optional<mavlink_gps_raw_int_t> gpsRaw;
    std::optional<mavlink_command_ack_t> commandAck;
    std::optional<mavlink_param_value_t> paramValue;
    
    // Mission messages
    std::optional<mavlink_mission_count_t> missionCount;
    std::optional<mavlink_mission_request_t> missionRequest;
    std::optional<mavlink_mission_item_int_t> missionItemInt;
    std::optional<mavlink_mission_ack_t> missionAck;
    
    // Logging messages
    std::optional<mavlink_log_entry_t> logEntry;
    std::optional<mavlink_log_data_t> logData;
};

class MessageParser {
public:
    MessageParser();
    ~MessageParser() = default;
    
    std::optional<ParsedMessage> parse(const uint8_t* data, size_t length);
    void reset();
    
private:
    mavlink_message_t message_;
    mavlink_status_t status_;
    uint8_t channel_;
    
    void decodeMessage(const mavlink_message_t& msg, ParsedMessage& parsed);
};

} // namespace margelo::nitro::mavlink
