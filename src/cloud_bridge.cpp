/**
 *  @file   cloud_bridge.cpp
 *  @date   2020-04-07
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

#include "cloud_bridge/cloud_bridge.hpp"

using namespace std::chrono_literals;

CloudBridge::CloudBridge(string nodeName, bool isBind)
  : Node(nodeName, rclcpp::NodeOptions())
  , m_pZmqCtx(nullptr)
{
  m_bIsServer = isBind;
  m_pNodeHandle = std::shared_ptr<::rclcpp::Node>(this);
  m_pZmqCtx = zmq_ctx_new();

  // Get Parameters for connect
  m_strCloudIp = declare_parameter("cloud_ip", "127.0.0.1");
  if(m_bIsServer) {
    m_iManagePort = declare_parameter("cloud_manage_port", 25565);
    m_iSubPort = declare_parameter("cloud_sub_port", 25567);
    m_iPubPort = declare_parameter("cloud_pub_port", 25568);
    m_iReqPort = declare_parameter("cloud_req_port", 25569);
    m_iRepPort = declare_parameter("cloud_rep_port", 25570);
  } else {
    m_iManagePort = declare_parameter("cloud_manage_port", 25565);
  }

  // Get Parameters for topic
  std::vector<string> deault_sub_list;
  std::vector<string> deault_pub_list;
  m_vectorSubTopic = declare_parameter("sub_list", deault_sub_list);
  m_vectorPubTopic = declare_parameter("pub_list", deault_pub_list);
  // Get Parameters for service
  std::vector<string> deault_srv_server_list;
  std::vector<string> deault_srv_client_list;
  m_vectorSrvServer = declare_parameter("srv_server_list", deault_srv_server_list);
  m_vectorSrvClient = declare_parameter("srv_client_list", deault_srv_client_list);
  
  // Get Parameters for tf
  std::vector<string> deault_tf_list;
  m_vectorTfLookup = declare_parameter("tf_list", deault_tf_list);

  Setup();
  initBridgeNode();
  Connect();
  Start();
}

CloudBridge::~CloudBridge()
{
  m_bRun = false;

  if (m_threadProc.joinable())
  {
    m_threadProc.join();
  }
  zmq_msg_close(&m_pSubMsg);

  rcl_ret_t rc;
  rcl_init_options_t init = rcl_get_zero_initialized_init_options();
  rc = rcl_init_options_fini(&init);
  if (rc != RCL_RET_OK)
  {
      ERROR("rcl_init_options_fini failed: " << rc);
  }
}

bool CloudBridge::Setup()
{
  if(m_bIsServer) {
    m_pManageSocket = zmq_socket(m_pZmqCtx, ZMQ_REP);
  } else {
    m_pManageSocket = zmq_socket(m_pZmqCtx, ZMQ_REQ);
  }
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
  if (zmq_msg_init(&m_pSubMsg) < 0) {
    ERROR("sub msg init failed:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }
  if (zmq_msg_init(&m_pReqMsg) < 0) {
    ERROR("req msg init failed:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }
  if (zmq_msg_init(&m_pRepMsg) < 0) {
    ERROR("res msg init failed:" << std::string(zmq_strerror(zmq_errno())));
    return false;
  }

  std::string manageAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iManagePort);
  if(m_bIsServer) {
    LOG("Zmq Bind ManageSocket to " << manageAddress.c_str());
    if (zmq_bind(m_pManageSocket, manageAddress.c_str()) < 0) {
      ERROR("Bind Manage Err:" << std::string(zmq_strerror(zmq_errno())));
      return false;
    }
  } else {
    LOG("Zmq Connect ManageSocket to " << manageAddress.c_str());
    if (zmq_connect(m_pManageSocket, manageAddress.c_str()) < 0) {
      ERROR("Connect Manage Err:" << std::string(zmq_strerror(zmq_errno())));
      return false;
    }
    SetPorts();
  }
  return true;
}

void CloudBridge::SetPorts()
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

std::vector<uint8_t> CloudBridge::SetManageReq(uint8_t op, std::string topic, std::string type)
{
  std::vector<uint8_t> data;
  data.reserve(sizeof(uint8_t) + sizeof(uint32_t) + topic.size() + sizeof(uint32_t) + type.size());

  // 0: get_ports, 1: add_sub, 2: add_pub
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

bool CloudBridge::SendManageRequest(
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

void CloudBridge::ReadManageProc()
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

      replyData.push_back(uint8_t(m_iPubPort >> 0));
      replyData.push_back(uint8_t(m_iPubPort >> 8));
      replyData.push_back(uint8_t(m_iPubPort >> 16));
      replyData.push_back(uint8_t(m_iPubPort >> 24));

      replyData.push_back(uint8_t(m_iSubPort >> 0));
      replyData.push_back(uint8_t(m_iSubPort >> 8));
      replyData.push_back(uint8_t(m_iSubPort >> 16));
      replyData.push_back(uint8_t(m_iSubPort >> 24));

      replyData.push_back(uint8_t(m_iReqPort >> 0));
      replyData.push_back(uint8_t(m_iReqPort >> 8));
      replyData.push_back(uint8_t(m_iReqPort >> 16));
      replyData.push_back(uint8_t(m_iReqPort >> 24));

      replyData.push_back(uint8_t(m_iRepPort >> 0));
      replyData.push_back(uint8_t(m_iRepPort >> 8));
      replyData.push_back(uint8_t(m_iRepPort >> 16));
      replyData.push_back(uint8_t(m_iRepPort >> 24));
    }
    SendManageReply(replyData.data(), replyData.size(), false);
  }
}

bool CloudBridge::ReceiveManageRequest(void** buffer, int& bufferLength, bool isNonBlockingMode)
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

uint32_t CloudBridge::get32le(std::vector<uint8_t> buffer, size_t size) const
{
  return buffer[size + 0] | (buffer[size + 1] << 8) | (buffer[size + 2] << 16) | (buffer[size + 3] << 24);
}

bool CloudBridge::SendManageReply(const void* buffer, const int bufferLength, bool isNonBlockingMode)
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

void CloudBridge::initBridgeNode() {
  bridge_node_context_ = rcl_get_zero_initialized_context();
  bridge_node_init_ = rcl_get_zero_initialized_init_options();
  bridge_node_alloc_ = rcl_get_default_allocator();
  rcl_ret_t rc;

  rc = rcl_init_options_init(&bridge_node_init_, bridge_node_alloc_);
  if (rc != RCL_RET_OK)
  {
      ERROR("rcl_init_options_init failed: " << rc);
      return;
  }

  rc = rcl_init(0, NULL, &bridge_node_init_, &bridge_node_context_);
  if (rc != RCL_RET_OK)
  {
      ERROR("rcl_init failed: " << rc);
      return;
  }
  bridge_node_ = new BridgeNode(&bridge_node_alloc_, &bridge_node_context_, get_namespace());
}

bool CloudBridge::Connect() 
{
  if(m_bIsServer) {
    m_strCloudIp = "*";
  }
  std::string subAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iSubPort);
  std::string pubAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iPubPort);
  std::string reqAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iReqPort);
  std::string repAddress = "tcp://" + m_strCloudIp + ":" + std::to_string(m_iRepPort);

  // Connect Pub Socket
  if(m_bIsServer) {
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
  } else {
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
  }

  if (zmq_setsockopt(m_pSubSocket, ZMQ_SUBSCRIBE, "", 0))
  {
    ERROR("SetSock Err:" << std::string(zmq_strerror(zmq_errno())));
  }

  client = new BridgeClient(*bridge_node_, m_pPubSocket, m_pSubSocket);
  
  for(unsigned int vi=0; vi<m_vectorSubTopic.size(); vi++) {
    std::string source = m_vectorSubTopic[vi];
    std::cout << "CloudBridge::"<< __FUNCTION__ << ", add subscriber: " << source << std::endl;
    std::string topic;
    std::string msg;
    topic = declare_parameter(source + "." + "topic", "");
    msg = declare_parameter(source + "." + "msg", "");
    if(topic.compare("") != 0 && msg.compare("") != 0) {
      bridge_node_->add_subscriber(topic, msg, client);
    }
    undeclare_parameter(source + "." + "topic");
    undeclare_parameter(source + "." + "msg");
  }

  for(unsigned int vi=0; vi<m_vectorPubTopic.size(); vi++) {
    std::string source = m_vectorPubTopic[vi];
    std::cout << "CloudBridge::"<< __FUNCTION__ << ", add publisher: " << source << std::endl;
    std::string topic;
    std::string msg;
    topic = declare_parameter(source + "." + "topic", "");
    msg = declare_parameter(source + "." + "msg", "");
    if(topic.compare("") != 0 && msg.compare("") != 0) {
      bridge_node_->add_publisher(topic, msg, client);
    }
    undeclare_parameter(source + "." + "topic");
    undeclare_parameter(source + "." + "msg");
  }

  for(unsigned int vi=0; vi<m_vectorTfLookup.size(); vi++) {
    std::string source = m_vectorTfLookup[vi];
    std::string baseFrame;
    std::string childFrame;
    baseFrame = declare_parameter(source + "." + "base_frame", "");
    childFrame = declare_parameter(source + "." + "child_frame", "");

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(m_pNodeHandle->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    LOG("TransformListen "<< baseFrame << " to " << childFrame);
    if(baseFrame.compare("") != 0 && childFrame.compare("") != 0) {
      std::string tf_topic_name = "tf_"+source;
      auto tfPub = m_pNodeHandle->create_publisher<tf2_msgs::msg::TFMessage>(tf_topic_name, rclcpp::SystemDefaultsQoS());
      auto tfThread2 = std::thread(
        [this](std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::string baseFrame, std::string childFrame) -> void
        { 
          while (m_bRun)
          {
            try {
              tf2::TimePoint tf2_time(std::chrono::nanoseconds(get_clock()->now().nanoseconds()));
              geometry_msgs::msg::TransformStamped transform = 
                tf_buffer->lookupTransform(baseFrame, childFrame, tf2_time, tf2::durationFromSec(1.0));

              bridge_node_->handle_tf(transform, client);
              rclcpp::sleep_for(1ms);
            } catch (tf2::TransformException & e) {
              ERROR("Failed to transform :" << e.what());
            }
          }
        }, tf_buffer, baseFrame, childFrame
      );
      m_vectorTfThread.push_back(std::move(tfThread2));
      m_vectorTfListener.push_back(std::move(tf_listener));
      m_vectorTfBuffer.push_back(std::move(tf_buffer));
      
      undeclare_parameter(source + "." + "base_frame");
      undeclare_parameter(source + "." + "child_frame");
    }
  }
  return true;
}

void CloudBridge::Disconnect()
{
  LOG("Zmq Disconnect... ");
  if (m_pSubSocket != nullptr) {
    zmq_close(m_pSubSocket);
    m_pSubSocket = nullptr;
  }

  if (m_pPubSocket != nullptr) {
    zmq_close(m_pPubSocket);
    m_pPubSocket = nullptr;
  }
}

bool CloudBridge::Reconnect()
{
  Disconnect();
  return Connect();
}

void CloudBridge::Start()
{
  m_bRun = true;
  client->start();
  if(m_bIsServer) {
    m_threadManageProc = std::thread([=]() { ReadManageProc(); });
  }
}
