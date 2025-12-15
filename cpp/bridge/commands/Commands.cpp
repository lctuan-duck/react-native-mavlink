#include "../../HybirdMAVLink.hpp"
#include "../../core/MAVLinkCore.hpp"
#include "../../mavlink/v2.0/common/common.h"
#include <NitroModules/Promise.hpp>
#include <NitroModules/ArrayBuffer.hpp>
#include <stdexcept>

namespace margelo::nitro::mavlink
{
  std::shared_ptr<Promise<std::shared_ptr<ArrayBuffer>>> HybirdMAVLink::encode(double messageId, const std::shared_ptr<ArrayBuffer>& payload)
  {
    auto promise = Promise<std::shared_ptr<ArrayBuffer>>::async();
    
    try {
      mavlink_message_t msg;
      
      // Initialize message with given ID
      msg.msgid = static_cast<uint32_t>(messageId);
      msg.sysid = 255;  // GCS
      msg.compid = 190; // Companion computer
      
      // Copy payload data
      uint8_t* payloadData = static_cast<uint8_t*>(payload->data());
      size_t payloadSize = payload->size();
      
      if (payloadSize > MAVLINK_MAX_PAYLOAD_LEN) {
        promise->reject(std::make_exception_ptr(std::runtime_error("Payload too large")));
        return promise;
      }
      
      msg.len = static_cast<uint8_t>(payloadSize);
      memcpy(msg.payload64, payloadData, payloadSize);
      
      // Serialize message to buffer
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      // Create result buffer with actual length
      auto result = ArrayBuffer::allocate(len);
      memcpy(result->data(), buffer, len);
      
      promise->resolve(result);
    } catch (const std::exception& e) {
      promise->reject(std::make_exception_ptr(e));
    }
    
    return promise;
  }

  std::shared_ptr<Promise<DecodedMessage>> HybirdMAVLink::decode(const std::shared_ptr<ArrayBuffer>& raw)
  {
    auto promise = Promise<DecodedMessage>::async();
    
    try {
      mavlink_message_t msg;
      mavlink_status_t status;
      
      uint8_t* data = static_cast<uint8_t*>(raw->data());
      size_t length = raw->size();
      
      // Parse the message
      for (size_t i = 0; i < length; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
          // Successfully parsed a message
          DecodedMessage result;
          result.messageId = msg.msgid;
          result.systemId = msg.sysid;
          result.componentId = msg.compid;
          
          // Extract payload
          auto payloadBuffer = ArrayBuffer::allocate(msg.len);
          memcpy(payloadBuffer->data(), msg.payload64, msg.len);
          result.payload = payloadBuffer;
          
          promise->resolve(result);
          return promise;
        }
      }
      
      promise->reject(std::make_exception_ptr(std::runtime_error("No valid MAVLink message found")));
    } catch (const std::exception& e) {
      promise->reject(std::make_exception_ptr(e));
    }
    
    return promise;
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::sendCommandLong(const CommandLongArgs& args)
  {
    auto promise = Promise<void>::async();
    
    if (!core_) {
      promise->reject(std::make_exception_ptr(std::runtime_error("Core not initialized")));
      return promise;
    }
    
    try {
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      // Extract parameters from individual fields (matching CommandIntArgs pattern)
      float param1 = args.param1.has_value() ? static_cast<float>(args.param1.value()) : 0.0f;
      float param2 = args.param2.has_value() ? static_cast<float>(args.param2.value()) : 0.0f;
      float param3 = args.param3.has_value() ? static_cast<float>(args.param3.value()) : 0.0f;
      float param4 = args.param4.has_value() ? static_cast<float>(args.param4.value()) : 0.0f;
      float param5 = args.param5.has_value() ? static_cast<float>(args.param5.value()) : 0.0f;
      float param6 = args.param6.has_value() ? static_cast<float>(args.param6.value()) : 0.0f;
      float param7 = args.param7.has_value() ? static_cast<float>(args.param7.value()) : 0.0f;
      
      uint8_t targetSystem = args.targetSystem.has_value() ? static_cast<uint8_t>(args.targetSystem.value()) : 1;
      uint8_t targetComponent = args.targetComponent.has_value() ? static_cast<uint8_t>(args.targetComponent.value()) : 1;
      
      // Pack COMMAND_LONG message
      mavlink_msg_command_long_pack(
        255, 190, &msg,  // system_id=255 (GCS), component_id=190
        targetSystem,
        targetComponent,
        static_cast<uint16_t>(args.command),
        0,  // confirmation
        param1, param2, param3, param4, param5, param6, param7
      );
      
      // Serialize to buffer
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      // Send via core
      if (core_->sendData(buffer, len)) {
        promise->resolve();
      } else {
        promise->reject(std::make_exception_ptr(std::runtime_error("Failed to send command")));
      }
    } catch (const std::exception& e) {
      promise->reject(std::make_exception_ptr(e));
    }
    
    return promise;
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::sendCommandInt(const CommandIntArgs& args)
  {
    auto promise = Promise<void>::async();
    
    if (!core_) {
      promise->reject(std::make_exception_ptr(std::runtime_error("Core not initialized")));
      return promise;
    }
    
    try {
      mavlink_message_t msg;
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      
      // Extract parameters (now explicit fields instead of array)
      float param1 = args.param1.has_value() ? static_cast<float>(args.param1.value()) : 0.0f;
      float param2 = args.param2.has_value() ? static_cast<float>(args.param2.value()) : 0.0f;
      float param3 = args.param3.has_value() ? static_cast<float>(args.param3.value()) : 0.0f;
      float param4 = args.param4.has_value() ? static_cast<float>(args.param4.value()) : 0.0f;
      
      // x, y, z are now explicit required fields
      int32_t x = static_cast<int32_t>(args.x);
      int32_t y = static_cast<int32_t>(args.y);
      float z = static_cast<float>(args.z);
      
      uint8_t targetSystem = args.targetSystem.has_value() ? static_cast<uint8_t>(args.targetSystem.value()) : 1;
      uint8_t targetComponent = args.targetComponent.has_value() ? static_cast<uint8_t>(args.targetComponent.value()) : 1;
      uint8_t frame = args.frame.has_value() ? static_cast<uint8_t>(args.frame.value()) : 3; // Default: MAV_FRAME_GLOBAL_RELATIVE_ALT
      
      // Pack COMMAND_INT message
      mavlink_msg_command_int_pack(
        255, 190, &msg,
        targetSystem,
        targetComponent,
        frame,  // Use explicit frame parameter
        static_cast<uint16_t>(args.command),
        0,  // current
        0,  // autocontinue
        param1, param2, param3, param4,
        x, y, z
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
      
      if (core_->sendData(buffer, len)) {
        promise->resolve();
      } else {
        promise->reject(std::make_exception_ptr(std::runtime_error("Failed to send command")));
      }
    } catch (const std::exception& e) {
      promise->reject(std::make_exception_ptr(e));
    }
    
    return promise;
  }

} // namespace margelo::nitro::mavlink
