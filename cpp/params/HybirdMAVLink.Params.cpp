// Parameter APIs split from HybirdMAVLink.cpp
#include "../HybirdMAVLink.hpp"
extern "C" {
#include "mavlink/v2.0/common/mavlink_msg_param_set.h"
}

namespace margelo::nitro::mavlink {

std::shared_ptr<Promise<void>> HybirdMAVLink::requestParams() {
  auto promise = std::make_shared<Promise<void>>();
  // Send PARAM_REQUEST_LIST
  mavlink_message_t msg; mavlink_param_request_list_t req{};
  req.target_system = (uint8_t)(lastSystemId.has_value() ? lastSystemId.value() : 1);
  req.target_component = (uint8_t)(lastComponentId.has_value() ? lastComponentId.value() : 1);
  mavlink_msg_param_request_list_encode(1,1,&msg,&req);
  std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
  uint16_t lenOut = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  buffer.resize(lenOut);
  bool sent=false; if (tcpClient) sent=tcpClient->send(buffer.data(), buffer.size()); if (!sent && udpServer) sent=udpServer->send(buffer.data(), buffer.size());
  if (sent) promise->resolve(); else promise->reject("Failed to send PARAM_REQUEST_LIST");
  return promise;
}

std::shared_ptr<Promise<void>> HybirdMAVLink::setParam(const std::string& name, double value, double type) {
  auto promise = std::make_shared<Promise<void>>();
  mavlink_message_t msg; mavlink_param_set_t ps{};
  ps.target_system = (uint8_t)(lastSystemId.has_value() ? lastSystemId.value() : 1);
  ps.target_component = (uint8_t)(lastComponentId.has_value() ? lastComponentId.value() : 1);
  strncpy((char*)ps.param_id, name.c_str(), sizeof(ps.param_id));
  ps.param_value = (float)value;
  ps.param_type = (uint8_t)type;
  mavlink_msg_param_set_encode(1,1,&msg,&ps);
  std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
  uint16_t lenOut = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  buffer.resize(lenOut);
  bool sent = false;
  if (tcpClient) sent = tcpClient->send(buffer.data(), buffer.size());
  if (!sent && udpServer) sent = udpServer->send(buffer.data(), buffer.size());
  if (sent) promise->resolve(); else promise->reject("Failed to send PARAM_SET");
  return promise;
}

} // namespace margelo::nitro::mavlink