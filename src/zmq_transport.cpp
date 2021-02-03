/**
 *  @file   zmq_transport.cpp
 *  @date   2020-09-21
 *  @author Sungkyu Kang
 *  @brief
 *        Bridge Client class for Cloud Bridge
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 *
 *         SPDX-License-Identifier: MIT
 */

#include "cloud_bridge/zmq_transport.hpp"
#include "cloud_bridge/bridge_rcl_node.hpp"
#include "cloud_bridge/logging.hpp"

#include <memory>
#include <string>
#include <functional>
#include <iostream>

enum
{
  OP_ADD_SUBSCRIBER = 1,
  OP_ADD_PUBLISHER = 2,
  OP_PUBLISH = 3,
};

ZmqTransport::ZmqTransport(BridgeRclNode &rcl_node, 
  void *pubSocket, void *subSocket,
  void *reqSocket, void *repSocket)
    : rcl_node_(rcl_node),
      m_pPubSocket(pubSocket),
      m_pSubSocket(subSocket),
      m_pReqSocket(reqSocket),
      m_pRepSocket(repSocket)
{
}

ZmqTransport::~ZmqTransport()
{
  zmq_msg_close(&m_pMsgSub);
  zmq_msg_close(&m_pMsgReq);
  zmq_msg_close(&m_pMsgRep);
}

void ZmqTransport::start()
{
  if (zmq_msg_init(&m_pMsgSub) < 0)
  {
    return;
  }
  if (zmq_msg_init(&m_pMsgReq) < 0)
  {
    return;
  }
  if (zmq_msg_init(&m_pMsgRep) < 0)
  {
    return;
  }
  m_threadProc = std::thread([=]() { read_zmq_sub(); });
  m_threadServiceProc = std::thread([=]() { read_zmq_service(); });
}

void ZmqTransport::stop()
{
}

void ZmqTransport::read_zmq_sub()
{
  bool m_bRun = true;
  void *pBuffer = nullptr;
  int bufferLength = 0;
  while (m_bRun)
  {
    const bool succeeded = receive_zmq(&pBuffer, bufferLength);
    if (!succeeded || bufferLength < 0)
    {
      ERROR("zmq receive error return size(" << bufferLength << "): " << zmq_strerror(zmq_errno()));
      continue;
    }
    
    auto ptr = static_cast<uint8_t *>(pBuffer);
    buffer.insert(buffer.end(), ptr, ptr + bufferLength);
    handle_publish();
  }
}

void ZmqTransport::read_zmq_service()
{
  bool m_bRun = true;
  void *pBuffer = nullptr;
  int bufferLength = 0;
  while (m_bRun)
  {
    const bool succeeded = receive_zmq_service(&pBuffer, bufferLength);
    if (!succeeded || bufferLength < 0)
    {
      ERROR("zmq receive error return size(" << bufferLength << "): " << zmq_strerror(zmq_errno()));
      continue;
    }
    auto ptr = static_cast<uint8_t *>(pBuffer);
    rep_buffer.insert(rep_buffer.end(), ptr, ptr + bufferLength);
    handle_service();
  }
}

void ZmqTransport::handle_add_subscriber()
{
  if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > buffer.size())
  {
    DEBUG("handle_add_subscriber too short header");
    return;
  }

  size_t offset = sizeof(uint8_t);

  uint32_t topic_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + topic_length > buffer.size())
  {
    DEBUG("handle_add_subscriber short topic " << (offset + topic_length) << " " << buffer.size());
    return;
  }

  std::string topic((char *)&buffer[offset], topic_length);
  offset += topic_length;

  uint32_t type_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + type_length > buffer.size())
  {
    DEBUG("handle_add_subscriber short type " << (offset + type_length) << " " << buffer.size());
    return;
  }

  std::string type((char *)&buffer[offset], type_length);
  offset += type_length;

  DEBUG("OP_ADD_SUBSCRIBER, topic = " << topic << ", type = " << type);

  // rcl_node_.add_subscriber(topic, type, this);

  buffer.erase(buffer.begin(), buffer.begin() + offset);
}

void ZmqTransport::handle_add_publisher()
{
  if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > buffer.size())
  {
    DEBUG("handle_add_publisher too short header");
    return;
  }

  size_t offset = sizeof(uint8_t);

  uint32_t topic_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + topic_length > buffer.size())
  {
    DEBUG("handle_add_publisher short1 " << (offset + topic_length) << " " << buffer.size());
    return;
  }

  std::string topic((char *)&buffer[offset], topic_length);
  offset += topic_length;

  uint32_t type_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + type_length > buffer.size())
  {
    DEBUG("handle_add_publisher short2 " << (offset + type_length) << " " << buffer.size());
    return;
  }

  std::string type((char *)&buffer[offset], type_length);
  offset += type_length;

  DEBUG("OP_ADD_PUBLISHER, topic = " << topic << ", type = " << type);

  // rcl_node_.add_publisher(topic, type, this);

  buffer.erase(buffer.begin(), buffer.begin() + offset);
}

void ZmqTransport::handle_publish()
{
  if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > buffer.size())
  {
    return;
  }

  size_t offset = 0;

  uint32_t topic_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + topic_length > buffer.size())
  {
    return;
  }
  std::string topic((char *)&buffer[offset], topic_length);
  offset += topic_length;

  uint32_t type_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + type_length > buffer.size())
  {
    return;
  }
  std::string type((char *)&buffer[offset], type_length);
  offset += type_length;

  uint32_t message_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + message_length > buffer.size())
  {
    return;
  }
  std::vector<uint8_t> message(&buffer[offset], &buffer[offset] + message_length);
  offset += message_length;

  DEBUG("OP_PUBLISH, topic = " << topic);

  check_topic_data(topic, type);
  rcl_node_.publish(topic, message);

  buffer.erase(buffer.begin(), buffer.begin() + offset);
}

void ZmqTransport::check_topic_data(std::string topic, std::string type)
{
  if(topic == "tf") {
    type = "tf2_msgs/msg/TFMessage";
  }

  for (const auto& td: topicDatas) {
    if(td->topic.compare(topic) == 0 && td->type.compare(type) == 0) {
      return;
    }
  }
  // not contains
  TopicData* topicData = new TopicData();
  topicData->topic = topic;
  topicData->type = type;
  topicDatas.insert(topicData);

  std::map<std::string, std::string>::iterator iter;
  std::string qos = "default";
  // for(iter = qos_map.begin(); iter != qos_map.end(); iter++){
  //   if(topic == iter->first) {
  //     qos = iter->second;
  //   }
  // }
  LOG("add_publisher topic: " << topic << ", type: " << type << ", qos_string: "<< qos);
  rcl_node_.add_publisher(topic, type, this, qos);
}

void ZmqTransport::set_qos_map(std::map<std::string, std::string> map)
{
  this->qos_map = map;
}

void ZmqTransport::send_publish(const std::string &topic, const std::string &type, const std::vector<uint8_t> &msg)
{
  DEBUG("PUBLISH, topic = " << topic);
  std::vector<uint8_t> data;
  data.reserve(sizeof(uint32_t) + topic.size() + sizeof(uint32_t) + type.size() +sizeof(uint32_t) + msg.size());

  data.push_back(uint8_t(topic.size() >> 0));
  data.push_back(uint8_t(topic.size() >> 8));
  data.push_back(uint8_t(topic.size() >> 16));
  data.push_back(uint8_t(topic.size() >> 24));
  data.insert(data.end(), (uint8_t *)topic.data(), (uint8_t *)topic.data() + topic.size());

  data.push_back(uint8_t(type.size() >> 0));
  data.push_back(uint8_t(type.size() >> 8));
  data.push_back(uint8_t(type.size() >> 16));
  data.push_back(uint8_t(type.size() >> 24));
  data.insert(data.end(), (uint8_t *)type.data(), (uint8_t *)type.data() + type.size());

  data.push_back(uint8_t(msg.size() >> 0));
  data.push_back(uint8_t(msg.size() >> 8));
  data.push_back(uint8_t(msg.size() >> 16));
  data.push_back(uint8_t(msg.size() >> 24));
  data.insert(data.end(), msg.data(), msg.data() + msg.size());

  send_zmq(data.data(), data.size());
}

uint32_t ZmqTransport::get32le(size_t offset) const
{
  return buffer[offset + 0] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16) | (buffer[offset + 3] << 24);
}

uint32_t ZmqTransport::get32le_rep(size_t offset) const
{
  return rep_buffer[offset + 0] | (rep_buffer[offset + 1] << 8) | (rep_buffer[offset + 2] << 16) | (rep_buffer[offset + 3] << 24);
}

bool ZmqTransport::receive_zmq(void **zmq_buffer, int &bufferLength)
{
  if (&m_pMsgSub == nullptr) {
    return false;
  }

  bufferLength = zmq_msg_recv(&m_pMsgSub, m_pSubSocket, 0);

  if (bufferLength == 0) {
    return false;
  }
  *zmq_buffer = zmq_msg_data(&m_pMsgSub);

  if (*zmq_buffer == nullptr) {
    return false;
  }
  return true;
}

bool ZmqTransport::send_zmq(const void *buffer, const int bufferLength)
{
  std::unique_lock<std::mutex> lock(m_mutexSend);
  zmq_msg_t msg;
  if (zmq_msg_init_size(&msg, bufferLength) < 0) {
    return false;
  }
  memcpy(zmq_msg_data(&msg), buffer, bufferLength);
  /* Send the message to the socket */
  if (zmq_msg_send(&msg, m_pPubSocket, 0) < 0) { 

    return false;
  }
  zmq_msg_close(&msg);
  return true;
}

void ZmqTransport::send_request_and_get_response(const std::string &topic,
  const std::vector<uint8_t> &req_msg, std::vector<uint8_t> &res_msg)
{
  DEBUG("SEND_REQUEST, topic = " << topic);
  std::vector<uint8_t> data;
  data.reserve(sizeof(uint32_t) + topic.size() + sizeof(uint32_t) + req_msg.size());

  data.push_back(uint8_t(topic.size() >> 0));
  data.push_back(uint8_t(topic.size() >> 8));
  data.push_back(uint8_t(topic.size() >> 16));
  data.push_back(uint8_t(topic.size() >> 24));
  data.insert(data.end(), (uint8_t *)topic.data(), (uint8_t *)topic.data() + topic.size());

  data.push_back(uint8_t(req_msg.size() >> 0));
  data.push_back(uint8_t(req_msg.size() >> 8));
  data.push_back(uint8_t(req_msg.size() >> 16));
  data.push_back(uint8_t(req_msg.size() >> 24));
  data.insert(data.end(), req_msg.data(), req_msg.data() + req_msg.size());

  void* recvBuffer = nullptr;
  int recvBufferLength = 0;
  send_zmq_request(data.data(), data.size(), &recvBuffer, recvBufferLength);
  auto ptr = static_cast<uint8_t *>(recvBuffer);

  res_msg.insert(res_msg.end(), ptr, ptr + recvBufferLength);
}

bool ZmqTransport::receive_zmq_service(void **zmq_buffer, int &bufferLength)
{
  std::unique_lock<std::mutex> lock(m_mutexRep);
  if (&m_pMsgRep == nullptr) {
    return false;
  }

  bufferLength = zmq_msg_recv(&m_pMsgRep, m_pRepSocket, 0);

  if (bufferLength == 0) {
    return false;
  }
  *zmq_buffer = zmq_msg_data(&m_pMsgRep);

  if (*zmq_buffer == nullptr) {
    return false;
  }
  return true;
}

bool ZmqTransport::send_zmq_request(const void* buffer, const int bufferLength, 
  void** recvBuffer, int& recvBufferLength)
{
  std::unique_lock<std::mutex> lock(m_mutexReq);
  zmq_msg_t msg;
  if (zmq_msg_init_size(&msg, bufferLength) < 0)
    return false;
  memcpy(zmq_msg_data(&msg), buffer, bufferLength);
  // /* Send the message to the socket */
  if (zmq_msg_send(&msg, m_pReqSocket, 0) < 0)
    return false;
	zmq_msg_close(&msg);
  recvBufferLength = zmq_msg_recv(&m_pMsgReq, m_pReqSocket, 0);

  if (recvBufferLength == 0)
    return false;

  *recvBuffer = zmq_msg_data(&m_pMsgReq);
  if (*recvBuffer == nullptr)
    return false;

  return true;
}

void ZmqTransport::handle_service()
{
  if (sizeof(uint8_t) + 2 > rep_buffer.size())
  {
    return;
  }

  size_t offset = 0;
  uint32_t topic_length = get32le_rep(offset);
  offset += sizeof(uint32_t);
  if (offset + topic_length > rep_buffer.size())
  {
    return;
  }
  std::string topic((char *)&rep_buffer[offset], topic_length);
  offset += topic_length;
  uint32_t message_length = get32le_rep(offset);
  offset += sizeof(uint32_t);
  if (offset + message_length > rep_buffer.size())
  {
    return;
  }
  std::vector<uint8_t> req_data(&rep_buffer[offset], &rep_buffer[offset] + message_length);
  offset += message_length;

  DEBUG("OP_SERVICE, topic = " << topic);
  std::vector<uint8_t> res_data;
  rcl_node_.service(topic, req_data, res_data);
  send_zmq_reply(res_data.data(), res_data.size());
  rep_buffer.erase(rep_buffer.begin(), rep_buffer.begin() + offset);
}

bool ZmqTransport::send_zmq_reply(const void* buffer, const int bufferLength)
{
  std::unique_lock<std::mutex> lock(m_mutexRep);
  zmq_msg_t msg;
  if (zmq_msg_init_size(&msg, bufferLength) < 0)
    return false;
  memcpy(zmq_msg_data(&msg), buffer, bufferLength);    
  // /* Send the message to the socket */
  if (zmq_msg_send(&msg, m_pRepSocket, 0) < 0)
    return false;
	zmq_msg_close(&msg);

  return true;
}