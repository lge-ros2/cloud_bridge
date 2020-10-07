/**
 *  @file   bridge_client.cpp
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

#include "cloud_bridge/bridge_client.hpp"
#include "cloud_bridge/bridge_node.hpp"
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

BridgeClient::BridgeClient(BridgeNode &node, void *pubSocket, void *subSocket)
    : node(node),
      m_pPubSocket(pubSocket),
      m_pSubSocket(subSocket)
{
}

BridgeClient::~BridgeClient()
{
  zmq_msg_close(&m_pMsgSub);
}

void BridgeClient::start()
{
  if (zmq_msg_init(&m_pMsgSub) < 0)
  {
    return;
  }
  m_threadProc = std::thread([=]() { handle_read(); });
}

void BridgeClient::stop()
{
}

void BridgeClient::handle_read()
{
  bool m_bRun = true;
  void *pBuffer = nullptr;
  int bufferLength = 0;
  while (m_bRun)
  {
    const bool succeeded = receive_zmq(&pBuffer, bufferLength);
    if (!succeeded || bufferLength < 0)
    {
      LOG("zmq receive error return size(" << bufferLength << "): " << zmq_strerror(zmq_errno()));
      continue;
    }
    
    auto ptr = static_cast<uint8_t *>(pBuffer);
    buffer.insert(buffer.end(), ptr, ptr + bufferLength);
    handle_publish();
  }
}

void BridgeClient::handle_add_subscriber()
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
    DEBUG("handle_add_subscriber short1 " << (offset + topic_length) << " " << buffer.size());
    return;
  }

  std::string topic((char *)&buffer[offset], topic_length);
  offset += topic_length;

  uint32_t type_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + type_length > buffer.size())
  {
    DEBUG("handle_add_subscriber short2 " << (offset + type_length) << " " << buffer.size());
    return;
  }

  std::string type((char *)&buffer[offset], type_length);
  offset += type_length;

  DEBUG("OP_ADD_SUBSCRIBER, topic = " << topic << ", type = " << type);

  node.add_subscriber(topic, type, this);

  buffer.erase(buffer.begin(), buffer.begin() + offset);
}

void BridgeClient::handle_add_publisher()
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

  node.add_publisher(topic, type, this);

  buffer.erase(buffer.begin(), buffer.begin() + offset);
}

void BridgeClient::handle_publish()
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
  node.publish(topic, message);

  buffer.erase(buffer.begin(), buffer.begin() + offset);
}

void BridgeClient::check_topic_data(std::string topic, std::string type)
{
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
  node.add_publisher(topic, type, this);
}

void BridgeClient::publish(const std::string &topic, const std::string &type, const std::vector<uint8_t> &msg)
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

uint32_t BridgeClient::get32le(size_t offset) const
{
  return buffer[offset + 0] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16) | (buffer[offset + 3] << 24);
}

bool BridgeClient::receive_zmq(void **zmq_buffer, int &bufferLength)
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

bool BridgeClient::send_zmq(const void *buffer, const int bufferLength)
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