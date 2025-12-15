#include "../../HybirdMAVLink.hpp"
#include <stdexcept>
#include "../../core/MAVLinkCore.hpp"
#include "../../mavlink/v2.0/common/mavlink.h"
#include <NitroModules/Promise.hpp>

namespace margelo::nitro::mavlink
{
  std::shared_ptr<Promise<void>> HybirdMAVLink::requestLogList()
  {
    return Promise<void>::async([this]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      // Pack LOG_REQUEST_LIST message
      mavlink_msg_log_request_list_pack(
        255, 190, &msg,
        1,       // target_system
        1,       // target_component
        0,       // start (0 = first log)
        0xFFFF   // end (0xFFFF = last log)
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to send log list request");
      }
    });
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::requestLogData(const LogDataRequestArgs& args)
  {
    return Promise<void>::async([this, args]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      // Pack LOG_REQUEST_DATA message
      mavlink_msg_log_request_data_pack(
        255, 190, &msg,
        1,                                    // target_system
        1,                                    // target_component
        static_cast<uint16_t>(args.id),       // id
        static_cast<uint32_t>(args.ofs),      // ofs (offset)
        static_cast<uint32_t>(args.count)     // count (bytes to read)
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to send log data request");
      }
    });
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::requestDataTransmissionHandshake(const DataTransmissionHandshakeArgs& args)
  {
    return Promise<void>::async([this, args]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      // Pack DATA_TRANSMISSION_HANDSHAKE message
      mavlink_msg_data_transmission_handshake_pack(
        255, 190, &msg,
        MAVLINK_DATA_STREAM_IMG_JPEG,            // type
        static_cast<uint32_t>(args.size),        // size
        static_cast<uint16_t>(0),                // width
        static_cast<uint16_t>(0),                // height
        static_cast<uint16_t>(args.packets),     // packets
        static_cast<uint8_t>(args.payload),      // payload (bytes per packet, max 255)
        static_cast<uint8_t>(args.jpgQuality)    // jpg_quality
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (!core_->sendData(buffer, len)) {
        throw std::runtime_error("Failed to send data transmission handshake");
      }
    });
  }

  // Logging event listeners
  double HybirdMAVLink::onLogging(const std::function<void(const LoggingEvent&)>& listener)
  {
    double token = nextToken++;
    loggingListeners[token] = listener;
    return token;
  }

  void HybirdMAVLink::offLogging(double token)
  {
    loggingListeners.erase(token);
  }

} // namespace margelo::nitro::mavlink
