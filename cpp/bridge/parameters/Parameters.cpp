#include "../../HybirdMAVLink.hpp"
#include "../../core/MAVLinkCore.hpp"
#include "../../mavlink/v2.0/common/mavlink.h"
#include <NitroModules/Promise.hpp>
#include <stdexcept>

namespace margelo::nitro::mavlink
{
  std::shared_ptr<Promise<void>> HybirdMAVLink::requestParams()
  {
    return Promise<void>::async([this]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      // Pack PARAM_REQUEST_LIST message
      mavlink_msg_param_request_list_pack(
        255, 190, &msg,  // system_id=255 (GCS), component_id=190
        1,               // target_system
        1                // target_component
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to send parameter request");
      }
    });
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::setParam(const std::string& name, const std::variant<std::string, double>& value)
  {
    return Promise<void>::async([this, name, value]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      // Convert parameter name to char array (max 16 chars)
      char param_id[16] = {0};
      strncpy(param_id, name.c_str(), sizeof(param_id) - 1);
      
      // Get float value
      float param_value;
      if (std::holds_alternative<double>(value)) {
        param_value = static_cast<float>(std::get<double>(value));
      } else {
        // If string, try to parse as number
        try {
          param_value = std::stof(std::get<std::string>(value));
        } catch (...) {
          throw std::runtime_error("Invalid parameter value");
        }
      }
      
      // Pack PARAM_SET message
      mavlink_msg_param_set_pack(
        255, 190, &msg,
        1,                     // target_system
        1,                     // target_component
        param_id,              // param_id
        param_value,           // param_value
        MAV_PARAM_TYPE_REAL32  // param_type
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to send parameter set");
      }
    });
  }

} // namespace margelo::nitro::mavlink
