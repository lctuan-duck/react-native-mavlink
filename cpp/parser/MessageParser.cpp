#include "MessageParser.hpp"
#include <cstring>

namespace margelo::nitro::mavlink {

MessageParser::MessageParser() : channel_(MAVLINK_COMM_0) {
    std::memset(&message_, 0, sizeof(message_));
    std::memset(&status_, 0, sizeof(status_));
}

void MessageParser::reset() {
    std::memset(&message_, 0, sizeof(message_));
    std::memset(&status_, 0, sizeof(status_));
}

std::optional<ParsedMessage> MessageParser::parse(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (mavlink_parse_char(channel_, data[i], &message_, &status_)) {
            ParsedMessage parsed;
            parsed.messageId = message_.msgid;
            parsed.systemId = message_.sysid;
            parsed.componentId = message_.compid;
            
            const uint8_t* payload = reinterpret_cast<const uint8_t*>(message_.payload64);
            parsed.payload.assign(payload, payload + message_.len);
            
            decodeMessage(message_, parsed);
            
            return parsed;
        }
    }
    return std::nullopt;
}

void MessageParser::decodeMessage(const mavlink_message_t& msg, ParsedMessage& parsed) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            parsed.heartbeat = hb;
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys;
            mavlink_msg_sys_status_decode(&msg, &sys);
            parsed.sysStatus = sys;
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg, &att);
            parsed.attitude = att;
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t gps;
            mavlink_msg_global_position_int_decode(&msg, &gps);
            parsed.gpsPosition = gps;
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&msg, &gps);
            parsed.gpsRaw = gps;
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            parsed.commandAck = ack;
            break;
        }
        case MAVLINK_MSG_ID_PARAM_VALUE: {
            mavlink_param_value_t param;
            mavlink_msg_param_value_decode(&msg, &param);
            parsed.paramValue = param;
            break;
        }
        case MAVLINK_MSG_ID_MISSION_COUNT: {
            mavlink_mission_count_t mission;
            mavlink_msg_mission_count_decode(&msg, &mission);
            parsed.missionCount = mission;
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST: {
            mavlink_mission_request_t mission;
            mavlink_msg_mission_request_decode(&msg, &mission);
            parsed.missionRequest = mission;
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
            mavlink_mission_item_int_t mission;
            mavlink_msg_mission_item_int_decode(&msg, &mission);
            parsed.missionItemInt = mission;
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ACK: {
            mavlink_mission_ack_t mission;
            mavlink_msg_mission_ack_decode(&msg, &mission);
            parsed.missionAck = mission;
            break;
        }
        case MAVLINK_MSG_ID_LOG_ENTRY: {
            mavlink_log_entry_t log;
            mavlink_msg_log_entry_decode(&msg, &log);
            parsed.logEntry = log;
            break;
        }
        case MAVLINK_MSG_ID_LOG_DATA: {
            mavlink_log_data_t log;
            mavlink_msg_log_data_decode(&msg, &log);
            parsed.logData = log;
            break;
        }
        default:
            break;
    }
}

} // namespace margelo::nitro::mavlink
