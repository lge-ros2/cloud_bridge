/**
 *  @file   cloud_bridge_base.cpp
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

#include "cloud_bridge/cloud_bridge_base.hpp"

using namespace std::chrono_literals;

CloudBridgeBase::CloudBridgeBase(string nodeName)
  : Node(nodeName, rclcpp::NodeOptions())
  , m_pZmqCtx(nullptr)
{

}

CloudBridgeBase::~CloudBridgeBase()
{

}

uint32_t CloudBridgeBase::get32le(std::vector<uint8_t> buffer, size_t size) const
{
  return buffer[size + 0] | (buffer[size + 1] << 8) | (buffer[size + 2] << 16) | (buffer[size + 3] << 24);
}

void CloudBridgeBase::initBridgeRclNode() {
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
  bridge_rcl_node_ = new BridgeRclNode(&bridge_node_alloc_, &bridge_node_context_, get_namespace());
  zmq_transport_ = new ZmqTransport(*bridge_rcl_node_, 
    m_pPubSocket, m_pSubSocket,
    m_pReqSocket, m_pRepSocket);
}

void CloudBridgeBase::initBridgeParams() {

  // Get Parameters for common parameters
  std::vector<string> deault_param_list;
  m_vectorParams = declare_parameter("param_list", deault_param_list);

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

  std::map<std::string, std::string> qos_map;
  for(unsigned int vi=0; vi<m_vectorParams.size(); vi++) {
    std::string source = m_vectorParams[vi];
    std::string topic;
    std::string qos;
    topic = declare_parameter(source + "." + "topic", "");
    qos = declare_parameter(source + "." + "qos", "");
    if(topic.compare("") != 0 && qos.compare("") != 0) {
      qos_map.insert(make_pair(topic, qos));
    }
    undeclare_parameter(source + "." + "topic");
    undeclare_parameter(source + "." + "qos");
  }
  zmq_transport_->set_qos_map(qos_map);
  
  for(unsigned int vi=0; vi<m_vectorSubTopic.size(); vi++) {
    std::string source = m_vectorSubTopic[vi];
    LOG("CloudBridgeBase add subscriber: " << source);
    std::string topic;
    std::string msg;
    std::string qos;
    topic = declare_parameter(source + "." + "topic", "");
    msg = declare_parameter(source + "." + "msg", "");
    qos = declare_parameter(source + "." + "qos", "");
    DEBUG("  - topic: " << topic << ", type: " << msg << ", qos_string: "<< qos);
    if(topic.compare("") != 0 && msg.compare("") != 0) {
      bridge_rcl_node_->add_subscriber(topic, msg, zmq_transport_, qos);
    }
    undeclare_parameter(source + "." + "topic");
    undeclare_parameter(source + "." + "msg");
    undeclare_parameter(source + "." + "qos");
  }

  for(unsigned int vi=0; vi<m_vectorPubTopic.size(); vi++) {
    std::string source = m_vectorPubTopic[vi];
    LOG("CloudBridgeBase add publisher: " << source);
    std::string topic;
    std::string msg;
    std::string qos;
    topic = declare_parameter(source + "." + "topic", "");
    msg = declare_parameter(source + "." + "msg", "");
    qos = declare_parameter(source + "." + "qos", "");
    DEBUG("  - topic: " << topic << ", type: " << msg << ", qos_string: "<< qos);
    if(topic.compare("") != 0 && msg.compare("") != 0) {
      bridge_rcl_node_->add_publisher(topic, msg, zmq_transport_, qos);
    }
    undeclare_parameter(source + "." + "topic");
    undeclare_parameter(source + "." + "msg");
    undeclare_parameter(source + "." + "qos");
  }

  for(unsigned int vi=0; vi<m_vectorSrvServer.size(); vi++) {
    std::string source = m_vectorSrvServer[vi];
    LOG("CloudBridgeBase add srv server: " << source);
    std::string service;
    std::string srv;
    service = declare_parameter(source + "." + "service", "");
    srv = declare_parameter(source + "." + "srv", "");
    DEBUG("  - service: " << service << ", type: " << srv);
    if(service.compare("") != 0 && srv.compare("") != 0) {
      bridge_rcl_node_->add_service_server(service, srv, zmq_transport_);
    }
    undeclare_parameter(source + "." + "service");
    undeclare_parameter(source + "." + "srv");
  }

  for(unsigned int vi=0; vi<m_vectorSrvClient.size(); vi++) {
    std::string source = m_vectorSrvClient[vi];
    LOG("CloudBridgeBase add srv client: " << source);
    std::string service;
    std::string srv;
    service = declare_parameter(source + "." + "service", "");
    srv = declare_parameter(source + "." + "srv", "");
    DEBUG("  - service: " << service << ", type: " << srv);
    if(service.compare("") != 0 && srv.compare("") != 0) {
      bridge_rcl_node_->add_service_client(service, srv, zmq_transport_);
    }
    undeclare_parameter(source + "." + "service");
    undeclare_parameter(source + "." + "srv");
  }

  for(unsigned int vi=0; vi<m_vectorTfLookup.size(); vi++) {
    std::string source = m_vectorTfLookup[vi];
    std::string baseFrame;
    std::string childFrame;
    baseFrame = declare_parameter(source + "." + "base_frame", "");
    childFrame = declare_parameter(source + "." + "child_frame", "");

    LOG("CloudBridgeBase TransformListen "<< baseFrame << " to " << childFrame);
    if(baseFrame.compare("") != 0 && childFrame.compare("") != 0) {
      auto tfThread2 = std::thread(
        [this](std::string baseFrame, std::string childFrame,  
              std::vector<std::shared_ptr<tf2_ros::TransformListener>> m_vectorTfListener,
              std::vector<std::shared_ptr<tf2_ros::Buffer>> m_vectorTfBuffer) -> void
        { 
          auto tf_buffer = std::make_shared<tf2_ros::Buffer>(m_pNodeHandle->get_clock());
          auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
          geometry_msgs::msg::TransformStamped last_transform;
          while (m_bRun)
          {
            try {
              tf2::TimePoint tf2_time(std::chrono::nanoseconds(get_clock()->now().nanoseconds()));
              geometry_msgs::msg::TransformStamped transform = 
                tf_buffer->lookupTransform(baseFrame, childFrame, tf2_time, tf2::durationFromSec(1.0));
              if(last_transform != transform) {
                last_transform = transform;
                bridge_rcl_node_->handle_tf(transform, zmq_transport_);
              }
              rclcpp::sleep_for(10ms);
            } catch (tf2::TransformException & e) {
              ERROR("Failed to transform :" << e.what());
            }
          }
          m_vectorTfListener.push_back(std::move(tf_listener));
          m_vectorTfBuffer.push_back(std::move(tf_buffer));
        }, baseFrame, childFrame, m_vectorTfListener, m_vectorTfBuffer
      );
      m_vectorTfThread.push_back(std::move(tfThread2));
      
    }
    undeclare_parameter(source + "." + "base_frame");
    undeclare_parameter(source + "." + "child_frame");
  }
}
void CloudBridgeBase::Disconnect()
{
  LOG("Zmq Disconnect... ");
  if (m_pManageSocket != nullptr) {
    zmq_close(m_pManageSocket);
    m_pManageSocket = nullptr;
  }

  if (m_pSubSocket != nullptr) {
    zmq_close(m_pSubSocket);
    m_pSubSocket = nullptr;
  }

  if (m_pPubSocket != nullptr) {
    zmq_close(m_pPubSocket);
    m_pPubSocket = nullptr;
  }

  if (m_pReqSocket != nullptr) {
    zmq_close(m_pReqSocket);
    m_pReqSocket = nullptr;
  }

  if (m_pRepSocket != nullptr) {
    zmq_close(m_pRepSocket);
    m_pRepSocket = nullptr;
  }
}
