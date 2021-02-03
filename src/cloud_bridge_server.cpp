/**
 *  @file   cloud_bridge_server.cpp
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

#include "cloud_bridge/cloud_bridge_server.hpp"

using namespace std::chrono_literals;

CloudBridgeServer::CloudBridgeServer(string nodeName)
  : CloudBridgeBase(nodeName)
{
  m_pNodeHandle = std::shared_ptr<::rclcpp::Node>(this);
  m_pZmqCtx = zmq_ctx_new();

  // Get Parameters for connect
  m_iManagePort = declare_parameter("manage_port", 25565);
  m_iSubPort = declare_parameter("sub_port", 25567);
  m_iPubPort = declare_parameter("pub_port", 25568);
  m_iReqPort = declare_parameter("req_port", 25569);
  m_iRepPort = declare_parameter("rep_port", 25570);

  m_iHostSubPort = declare_parameter("host_sub_port", m_iSubPort);
  m_iHostPubPort = declare_parameter("host_pub_port", m_iPubPort);
  m_iHostReqPort = declare_parameter("host_req_port", m_iReqPort);
  m_iHostRepPort = declare_parameter("host_rep_port", m_iRepPort);

  ERROR("CloudBridgeServer ports");
  ERROR("\t manage: " << m_iManagePort);
  ERROR("\t sub: " << m_iSubPort);
  ERROR("\t pub: " << m_iPubPort);
  ERROR("\t req: " << m_iReqPort);
  ERROR("\t rep: " << m_iRepPort);
  ERROR("\t host_sub: " << m_iHostSubPort);
  ERROR("\t host_pub: " << m_iHostPubPort);
  ERROR("\t host_req: " << m_iHostReqPort);
  ERROR("\t host_rep: " << m_iHostRepPort);

  Setup();
  initBridgeRclNode(); // from CloudBridgeBase
  initBridgeParams();  // from CloudBridgeBase
  Connect();
}

CloudBridgeServer::~CloudBridgeServer()
{

}

bool CloudBridgeServer::Setup()
{
  m_pManageSocket = zmq_socket(m_pZmqCtx, ZMQ_REP);
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

  std::string manageAddress = "tcp://*:" + std::to_string(m_iManagePort);
  LOG("Zmq Bind ManageSocket to " << manageAddress.c_str());
  if (zmq_bind(m_pManageSocket, manageAddress.c_str()) < 0) {
    ERROR("Bind Manage Err:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }

  return true;
}

void CloudBridgeServer::ReadManageProc()
{
  void *pBuffer = nullptr;
  int bufferLength = 0;
  while (m_bRun)
  {
    const bool succeeded = ReceiveManageRequest(&pBuffer, bufferLength, false);
    if (!succeeded || bufferLength < 0)
    {
      LOG("zmq receive error return size: " << bufferLength << ", errno: "<<  zmq_strerror(zmq_errno()));
      continue;
    }
    auto ptr = static_cast<uint8_t *>(pBuffer);
    std::vector<uint8_t> recvData;
    recvData.insert(recvData.end(), ptr, ptr + bufferLength);
    std::vector<uint8_t> replyData;
    // 0: get_ports, 1: add_sub, 2: add_pub
    if(recvData[0] == 0) {
      replyData.reserve(sizeof(uint8_t) + sizeof(uint32_t) * 4);
      replyData.push_back(recvData[0]);

      replyData.push_back(uint8_t(m_iHostPubPort >> 0));
      replyData.push_back(uint8_t(m_iHostPubPort >> 8));
      replyData.push_back(uint8_t(m_iHostPubPort >> 16));
      replyData.push_back(uint8_t(m_iHostPubPort >> 24));

      replyData.push_back(uint8_t(m_iHostSubPort >> 0));
      replyData.push_back(uint8_t(m_iHostSubPort >> 8));
      replyData.push_back(uint8_t(m_iHostSubPort >> 16));
      replyData.push_back(uint8_t(m_iHostSubPort >> 24));

      replyData.push_back(uint8_t(m_iHostReqPort >> 0));
      replyData.push_back(uint8_t(m_iHostReqPort >> 8));
      replyData.push_back(uint8_t(m_iHostReqPort >> 16));
      replyData.push_back(uint8_t(m_iHostReqPort >> 24));

      replyData.push_back(uint8_t(m_iHostRepPort >> 0));
      replyData.push_back(uint8_t(m_iHostRepPort >> 8));
      replyData.push_back(uint8_t(m_iHostRepPort >> 16));
      replyData.push_back(uint8_t(m_iHostRepPort >> 24));            
    }
    SendManageReply(replyData.data(), replyData.size(), false);
  }
}

bool CloudBridgeServer::ReceiveManageRequest(void** buffer, int& bufferLength, bool isNonBlockingMode)
{
  if (&m_pManageMsg == nullptr)
    return false;
  bufferLength = zmq_msg_recv(&m_pManageMsg, m_pManageSocket, (isNonBlockingMode)? ZMQ_DONTWAIT:0);
  if (bufferLength == 0)
    return false;
  *buffer = zmq_msg_data(&m_pManageMsg);
  if (*buffer == nullptr)
    return false;
  return true;
}

bool CloudBridgeServer::SendManageReply(const void* buffer, const int bufferLength, bool isNonBlockingMode)
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

  return true;
}

bool CloudBridgeServer::Connect() 
{

  std::string subAddress = "tcp://*:" + std::to_string(m_iSubPort);
  std::string pubAddress = "tcp://*:" + std::to_string(m_iPubPort);
  std::string reqAddress = "tcp://*:" + std::to_string(m_iReqPort);
  std::string repAddress = "tcp://*:" + std::to_string(m_iRepPort);

  LOG("Zmq Bind PubSocket to " << pubAddress.c_str());
  if (zmq_bind(m_pPubSocket, pubAddress.c_str()) < 0)
  {
    ERROR("Bind Pub Err:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }
  LOG("Zmq Bind SubSocket to " << subAddress.c_str());
  if (zmq_bind(m_pSubSocket, subAddress.c_str()) < 0)
  {
    ERROR("Bind Sub Err:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }
  LOG("Zmq Bind ReqSocket to " << reqAddress.c_str());
  if (zmq_bind(m_pReqSocket, reqAddress.c_str()) < 0)
  {
    ERROR("Bind Req Err:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }
  LOG("Zmq Bind RepSocket to " << repAddress.c_str());
  if (zmq_bind(m_pRepSocket, repAddress.c_str()) < 0)
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
  m_threadManageProc = std::thread([=]() { ReadManageProc(); });
  return true;
}
