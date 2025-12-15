// Mission-related implementations split from HybirdMAVLink.cpp
#include "../HybirdMAVLink.hpp"
extern "C" {
#include "mavlink/v2.0/common/mavlink_msg_mission_request_list.h"
#include "mavlink/v2.0/common/mavlink_msg_mission_count.h"
#include "mavlink/v2.0/common/mavlink_msg_mission_item_int.h"
#include "mavlink/v2.0/common/mavlink_msg_mission_clear_all.h"
#include "mavlink/v2.0/common/mavlink_msg_mission_set_current.h"
}

namespace margelo::nitro::mavlink {

std::shared_ptr<Promise<void>> HybirdMAVLink::requestMissionList() {
  auto promise = std::make_shared<Promise<void>>();
  mavlink_message_t msg;
  mavlink_mission_request_list_t req{};
  req.target_system = (uint8_t)(lastSystemId.has_value() ? lastSystemId.value() : 1);
  req.target_component = (uint8_t)(lastComponentId.has_value() ? lastComponentId.value() : 1);
  mavlink_msg_mission_request_list_encode(1, 1, &msg, &req);
  std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
  uint16_t lenOut = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  buffer.resize(lenOut);
  bool sent = false;
  if (tcpClient) sent = tcpClient->send(buffer.data(), buffer.size());
  if (!sent && udpServer) sent = udpServer->send(buffer.data(), buffer.size());
  if (sent) promise->resolve(); else promise->reject("Failed to send MISSION_REQUEST_LIST");
  return promise;
}

std::shared_ptr<Promise<void>> HybirdMAVLink::sendMissionCount(double count) {
  auto promise = std::make_shared<Promise<void>>();
  mavlink_message_t msg;
  mavlink_mission_count_t mc{};
  mc.target_system = (uint8_t)(lastSystemId.has_value() ? lastSystemId.value() : 1);
  mc.target_component = (uint8_t)(lastComponentId.has_value() ? lastComponentId.value() : 1);
  mc.count = (uint16_t)count;
  mavlink_msg_mission_count_encode(1, 1, &msg, &mc);
  std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
  uint16_t lenOut = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  buffer.resize(lenOut);
  bool sent = false;
  if (tcpClient) sent = tcpClient->send(buffer.data(), buffer.size());
  if (!sent && udpServer) sent = udpServer->send(buffer.data(), buffer.size());
  if (sent) promise->resolve(); else promise->reject("Failed to send MISSION_COUNT");
  return promise;
}

std::shared_ptr<Promise<void>> HybirdMAVLink::sendMissionItemInt(const MissionItemInt& item) {
  auto promise = std::make_shared<Promise<void>>();
  mavlink_message_t msg;
  mavlink_mission_item_int_t mi{};
  mi.target_system = (uint8_t)(lastSystemId.has_value() ? lastSystemId.value() : 1);
  mi.target_component = (uint8_t)(lastComponentId.has_value() ? lastComponentId.value() : 1);
  mi.seq = (uint16_t)item.seq;
  mi.frame = (uint8_t)item.frame;
  mi.command = (uint16_t)item.command;
  mi.current = (uint8_t)item.current;
  mi.autocontinue = (uint8_t)item.autocontinue;
  mi.x = (int32_t)item.x;
  mi.y = (int32_t)item.y;
  mi.z = (float)item.z;
  mavlink_msg_mission_item_int_encode(1, 1, &msg, &mi);
  std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
  uint16_t lenOut = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  buffer.resize(lenOut);
  bool sent = false;
  if (tcpClient) sent = tcpClient->send(buffer.data(), buffer.size());
  if (!sent && udpServer) sent = udpServer->send(buffer.data(), buffer.size());
  if (sent) promise->resolve(); else promise->reject("Failed to send MISSION_ITEM_INT");
  return promise;
}

std::shared_ptr<Promise<void>> HybirdMAVLink::clearAllMissions() {
  auto promise = std::make_shared<Promise<void>>();
  mavlink_message_t msg;
  mavlink_mission_clear_all_t clr{};
  clr.target_system = (uint8_t)(lastSystemId.has_value() ? lastSystemId.value() : 1);
  clr.target_component = (uint8_t)(lastComponentId.has_value() ? lastComponentId.value() : 1);
  mavlink_msg_mission_clear_all_encode(1, 1, &msg, &clr);
  std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
  uint16_t lenOut = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  buffer.resize(lenOut);
  bool sent = false;
  if (tcpClient) sent = tcpClient->send(buffer.data(), buffer.size());
  if (!sent && udpServer) sent = udpServer->send(buffer.data(), buffer.size());
  if (sent) promise->resolve(); else promise->reject("Failed to send MISSION_CLEAR_ALL");
  return promise;
}

std::shared_ptr<Promise<void>> HybirdMAVLink::setCurrentMission(double seq) {
  auto promise = std::make_shared<Promise<void>>();
  mavlink_message_t msg;
  mavlink_mission_set_current_t cur{};
  cur.target_system = (uint8_t)(lastSystemId.has_value() ? lastSystemId.value() : 1);
  cur.target_component = (uint8_t)(lastComponentId.has_value() ? lastComponentId.value() : 1);
  cur.seq = (uint16_t)seq;
  mavlink_msg_mission_set_current_encode(1, 1, &msg, &cur);
  std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
  uint16_t lenOut = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  buffer.resize(lenOut);
  bool sent = false;
  if (tcpClient) sent = tcpClient->send(buffer.data(), buffer.size());
  if (!sent && udpServer) sent = udpServer->send(buffer.data(), buffer.size());
  if (sent) promise->resolve(); else promise->reject("Failed to send MISSION_SET_CURRENT");
  return promise;
}

} // namespace margelo::nitro::mavlink