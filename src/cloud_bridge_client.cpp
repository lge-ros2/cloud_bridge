/**
 *  @file   cloud_bridge_client.cpp
 *  @date   2021-01-28
 *  @author Sungkyu Kang
 *  @brief
 *          Transfer ROS2 message to cloud using ZeroMQ
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2020 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 *
 *         SPDX-License-Identifier: MIT
 */

#include "cloud_bridge/cloud_bridge_client.hpp"

using namespace std::chrono_literals;

CloudBridgeClient::CloudBridgeClient(string nodeName)
  : CloudBridgeBase(nodeName)
{
  m_pNodeHandle = std::shared_ptr<::rclcpp::Node>(this);
  m_pZmqCtx = zmq_ctx_new();

  // Get Parameters for connect
  m_strCloudIp = declare_parameter("cloud_ip", "127.0.0.1");
  m_iManagePort = declare_parameter("manage_port", 25565);
  
  ERROR("CloudBridgeClient ports");
  ERROR("\t manage: " << m_iManagePort);
  
  Setup();
  initBridgeRclNode();  // from CloudBridgeBase
  initBridgeParams();   // from CloudBridgeBase
  Connect();
}

CloudBridgeClient::~CloudBridgeClient()
{

}

bool CloudBridgeClient::Setup()
{
  m_pManageSocket = zmq_socket(m_pZmqCtx, ZMQ_REQ);
  m_pPubSocket = zmq_socket(m_pZmqCtx, ZMQ_PUB);
  m_pSubSocket = zmq_socket(m_pZmqCtx, ZMQ_SUB);
  m_pReqSocket = zmq_socket(m_pZmqCtx, ZMQ_REQ);
  m_pRepSocket = zmq_socket(m_pZmqCtx, ZMQ_REP);

  if (m_pSubSocket == nullptr) {
    ERROR("NULL Socket for sub!!");
    return false;
  }

  if (zmq_msg_init(&m_pManageMsg) < 0) {
    ERROR("manage msg init failed:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }

  std::string manageAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iManagePort);
  LOG("Zmq Connect ManageSocket to " << manageAddress.c_str());
  if (zmq_connect(m_pManageSocket, manageAddress.c_str()) < 0) {
    ERROR("Connect Manage Err:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }
  SetPorts();

  return true;
}

void CloudBridgeClient::SetPorts()
{
  std::vector<uint8_t> data = SetManageReq(0, "topic", "type");
  
  void* recvBuffer = nullptr;
  int recvBufferLength = 0;
  SendManageRequest(data.data(), data.size(), false, &recvBuffer, recvBufferLength);
  auto ptr = static_cast<uint8_t *>(recvBuffer);
  std::vector<uint8_t> recvData;
  recvData.insert(recvData.end(), ptr, ptr + recvBufferLength);
  
  size_t offset = sizeof(uint8_t);

  m_iSubPort = (int)get32le(recvData, offset);
  offset += sizeof(uint32_t);
  m_iPubPort = (int)get32le(recvData, offset);
  offset += sizeof(uint32_t);
  m_iRepPort = (int)get32le(recvData, offset);
  offset += sizeof(uint32_t);
  m_iReqPort = (int)get32le(recvData, offset);
}

std::vector<uint8_t> CloudBridgeClient::SetManageReq(uint8_t op, std::string topic, std::string type)
{
  std::vector<uint8_t> data;
  data.reserve(sizeof(uint8_t) + sizeof(uint32_t) + topic.size() + sizeof(uint32_t) + type.size());

  // 0: get_ports
  data.push_back(op);

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
  return data;
}

bool CloudBridgeClient::SendManageRequest(
  const void* buffer, const int bufferLength, bool isNonBlockingMode, 
  void** recvBuffer, int& recvBufferLength)
{
  std::unique_lock<std::mutex> lock(m_mutexManage);
  zmq_msg_t msg;
  if (zmq_msg_init_size(&msg, bufferLength) < 0)
    return false;
  memcpy(zmq_msg_data(&msg), buffer, bufferLength);
  // /* Send the message to the socket */
  if (zmq_msg_send(&msg, m_pManageSocket, (isNonBlockingMode)? ZMQ_DONTWAIT:0) < 0)
    return false;
	zmq_msg_close(&msg);
  recvBufferLength = zmq_msg_recv(&m_pManageMsg, m_pManageSocket, (isNonBlockingMode)? ZMQ_DONTWAIT:0);
  if (recvBufferLength == 0)
    return false;
  *recvBuffer = zmq_msg_data(&m_pManageMsg);
  if (*recvBuffer == nullptr)
    return false;

  return true;
}

bool CloudBridgeClient::Connect() 
{
  std::string subAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iSubPort);
  std::string pubAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iPubPort);
  std::string reqAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iReqPort);
  std::string repAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iRepPort);

  ERROR("\t sub: " << m_iSubPort);
  ERROR("\t pub: " << m_iPubPort);
  ERROR("\t req: " << m_iReqPort);
  ERROR("\t rep: " << m_iRepPort);

  // Connect Pub Socket
  LOG("Zmq Connect PubSocket to " << pubAddress.c_str());
  if (zmq_connect(m_pPubSocket, pubAddress.c_str()) < 0)
  {
    ERROR("Connect Pub Err:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }
  LOG("Zmq Connect SubSocket to " << subAddress.c_str());
  if (zmq_connect(m_pSubSocket, subAddress.c_str()) < 0)
  {
    ERROR("Connect Sub Err:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }
  LOG("Zmq Connect ReqSocket to " << reqAddress.c_str());
  if (zmq_connect(m_pReqSocket, reqAddress.c_str()) < 0)
  {
    ERROR("Bind Req Err:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }
  LOG("Zmq Connect RepSocket to " << repAddress.c_str());
  if (zmq_connect(m_pRepSocket, repAddress.c_str()) < 0)
  {
    ERROR("Bind Res Err:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }

  if (zmq_setsockopt(m_pSubSocket, ZMQ_SUBSCRIBE, "", 0))
  {
    ERROR("SetSock Err:" << std::string(zmq_strerror(zmq_errno())));
  }

  m_bRun = true;
  zmq_transport_->start();

  return true;
}

