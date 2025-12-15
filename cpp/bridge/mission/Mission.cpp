#include "../../HybirdMAVLink.hpp"
#include <stdexcept>
#include "../../core/MAVLinkCore.hpp"
#include "../../mavlink/v2.0/common/mavlink.h"
#include <NitroModules/Promise.hpp>

namespace margelo::nitro::mavlink
{
  // Request mission list from vehicle
  std::shared_ptr<Promise<void>> HybirdMAVLink::requestMissionList()
  {
    return Promise<void>::async([this]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      // Pack MISSION_REQUEST_LIST message
      mavlink_msg_mission_request_list_pack(
        255, 190, &msg,
        1,  // target_system
        1,  // target_component
        MAV_MISSION_TYPE_MISSION  // mission_type
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to send mission request");
      }
    });
  }

  // Send mission count to vehicle
  std::shared_ptr<Promise<void>> HybirdMAVLink::sendMissionCount(double count)
  {
    return Promise<void>::async([this, count]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      mavlink_msg_mission_count_pack(
        255, 190, &msg,
        1,  // target_system
        1,  // target_component
        static_cast<uint16_t>(count),
        MAV_MISSION_TYPE_MISSION,
        0   // opaque_id
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to send mission count");
      }
    });
  }

  // Send individual mission item
  std::shared_ptr<Promise<void>> HybirdMAVLink::sendMissionItemInt(const MissionItemInt& item)
  {
    return Promise<void>::async([this, item]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      // Extract optional params (default to 0.0 if not provided)
      float param1 = item.param1.has_value() ? static_cast<float>(item.param1.value()) : 0.0f;
      float param2 = item.param2.has_value() ? static_cast<float>(item.param2.value()) : 0.0f;
      float param3 = item.param3.has_value() ? static_cast<float>(item.param3.value()) : 0.0f;
      float param4 = item.param4.has_value() ? static_cast<float>(item.param4.value()) : 0.0f;
      
      mavlink_msg_mission_item_int_pack(
        255, 190, &msg,
        1,  // target_system
        1,  // target_component
        static_cast<uint16_t>(item.seq),
        static_cast<uint8_t>(item.frame),
        static_cast<uint16_t>(item.command),
        static_cast<uint8_t>(item.current),
        static_cast<uint8_t>(item.autocontinue),
        param1, param2, param3, param4,
        static_cast<int32_t>(item.x),
        static_cast<int32_t>(item.y),
        static_cast<float>(item.z),
        MAV_MISSION_TYPE_MISSION
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to send mission item");
      }
    });
  }

  // Clear all missions
  std::shared_ptr<Promise<void>> HybirdMAVLink::clearAllMissions()
  {
    return Promise<void>::async([this]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      mavlink_msg_mission_clear_all_pack(
        255, 190, &msg,
        1,  // target_system
        1,  // target_component
        MAV_MISSION_TYPE_MISSION
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to clear missions");
      }
    });
  }

  // Set current mission item
  std::shared_ptr<Promise<void>> HybirdMAVLink::setCurrentMission(double seq)
  {
    return Promise<void>::async([this, seq]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      mavlink_msg_mission_set_current_pack(
        255, 190, &msg,
        1,  // target_system
        1,  // target_component
        static_cast<uint16_t>(seq)
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to set current mission");
      }
    });
  }

  // Upload multiple mission items (auto state machine)
  std::shared_ptr<Promise<void>> HybirdMAVLink::setMissionUpload(const std::vector<MissionItemInt>& items)
  {
    return Promise<void>::async([this, items]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      if (items.empty()) {
        throw std::runtime_error("Mission items list is empty");
      }
      
      // First send mission count
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      mavlink_msg_mission_count_pack(
        255, 190, &msg,
        1, 1,
        static_cast<uint16_t>(items.size()),
        MAV_MISSION_TYPE_MISSION,
        0  // opaque_id
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to initiate mission upload");
      }
      
      // TODO: Implement state machine to send items one by one upon receiving MISSION_REQUEST
    });
  }

  // Enable/disable auto mission upload
  std::shared_ptr<Promise<void>> HybirdMAVLink::enableAutoMissionUpload(bool enable)
  {
    return Promise<void>::async([enable]() {
      // TODO: Store flag and use in mission upload logic
      (void)enable; // Suppress unused parameter warning
    });
  }

  // Mission event listeners
  double HybirdMAVLink::onMissionCount(const std::function<void(const MissionCountEvent&)>& listener)
  {
    double token = nextToken++;
    missionCountListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offMissionCount(double token)
  {
    missionCountListeners.erase(token);
  }

  double HybirdMAVLink::onMissionRequest(const std::function<void(const MissionRequestEvent&)>& listener)
  {
    double token = nextToken++;
    missionRequestListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offMissionRequest(double token)
  {
    missionRequestListeners.erase(token);
  }

  double HybirdMAVLink::onMissionItem(const std::function<void(const MissionItemEvent&)>& listener)
  {
    double token = nextToken++;
    missionItemListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offMissionItem(double token)
  {
    missionItemListeners.erase(token);
  }

  double HybirdMAVLink::onMissionAck(const std::function<void(const MissionAckEvent&)>& listener)
  {
    double token = nextToken++;
    missionAckListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offMissionAck(double token)
  {
    missionAckListeners.erase(token);
  }

} // namespace margelo::nitro::mavlink
