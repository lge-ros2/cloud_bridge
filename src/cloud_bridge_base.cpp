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
  // Get Robot Name
  string default_robot_name = NONE_ROBOT_NAME;
  m_robotName = declare_parameter("robot_name", default_robot_name);
  if (m_robotName == "") {
    m_robotName = NONE_ROBOT_NAME;
    ERROR("robot_name is not set - setup none robot name");
  } 
  ERROR("robot_name: " << m_robotName);
  m_namespace = get_namespace();
  ERROR(" namespace: " << m_namespace);
}

CloudBridgeBase::~CloudBridgeBase()
{

}

void CloudBridgeBase::setServerName() {
  m_robotName = SERVER_ROBOT_NAME;
  ERROR("robot_name: " << m_robotName);
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
  bridge_rcl_node_ = new BridgeRclNode(&bridge_node_alloc_, &bridge_node_context_, m_namespace);
  zmq_transport_ = new ZmqTransport(
    m_namespace,
    m_robotName,
    *bridge_rcl_node_, 
    m_pPubSocket, m_pSubSocket,
    m_pReqSocket, m_pRepSocket);
}

void CloudBridgeBase::initBridgeParams() {

  // Get Parameters for common parameters
  std::vector<string> deault_param_list;
  m_vectorParams = declare_parameter("param_list", deault_param_list);

  // Get Parameters for topic
  std::vector<string> deault_ns_list;
  std::vector<string> deault_sub_list;
  std::vector<string> deault_pub_list;
  m_vectorNamespace = declare_parameter("namespace_list", deault_ns_list);
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

  bool default_sub_clock = true;
  sub_clock_ = declare_parameter("sub_clock", default_sub_clock);
  if(sub_clock_) {
    ERROR("  sub_clock_ true");
  } else {
    ERROR("  sub_clock_ false");
  }

  std::map<std::string, std::string> qos_map;
  for(unsigned int vi=0; vi<m_vectorParams.size(); vi++) {
    std::string source = m_vectorParams[vi];
    std::string topic;
    std::string msg;
    std::string qos;
    std::string service;
    std::string srv;
    std::string base_frame;
    std::string child_frame;
    topic = declare_parameter(source + "." + "topic", "");
    msg = declare_parameter(source + "." + "msg", "");
    qos = declare_parameter(source + "." + "qos", "");
    service = declare_parameter(source + "." + "service", "");
    srv = declare_parameter(source + "." + "srv", "");
    base_frame = declare_parameter(source + "." + "base_frame", "");
    child_frame = declare_parameter(source + "." + "child_frame", "");
    if(topic.compare("") != 0 && qos.compare("") != 0) {
      qos_map.insert(make_pair(topic, qos));
    }
    // undeclare_parameter(source + "." + "topic");
    // undeclare_parameter(source + "." + "qos");
  }
  zmq_transport_->set_qos_map(qos_map);
  zmq_transport_->set_robotnames(m_vectorNamespace);
  
  for(unsigned int vi=0; vi<m_vectorSubTopic.size(); vi++) {
    std::string source = m_vectorSubTopic[vi];
    LOG("CloudBridgeBase add subscriber: " << source);
    std::string topic;
    std::string msg;
    std::string qos;
    get_parameter(source + "." + "topic", topic);
    get_parameter(source + "." + "msg", msg);
    get_parameter(source + "." + "qos", qos);
    if(topic.compare("") != 0 && msg.compare("") != 0) {
      if(!sub_clock_ && topic.compare("/clock") == 0) {
        ERROR("  did not add clock by sub_clock parameter");
      } else {
        if (topic.rfind("/", 0) != 0 && m_vectorNamespace.size() > 0) {
          for(unsigned int ni=0; ni<m_vectorNamespace.size(); ni++) {
            string namespaced_topic = "/"+m_vectorNamespace[ni]+"/"+topic;
            LOG("  - topic: " << namespaced_topic << ", type: " << msg << ", qos_string: "<< qos);
            bridge_rcl_node_->add_subscriber(namespaced_topic, msg, zmq_transport_, qos);
          }
        } else {
          LOG("  - topic: " << topic << ", type: " << msg << ", qos_string: "<< qos);
          bridge_rcl_node_->add_subscriber(topic, msg, zmq_transport_, qos);
        }
      }
    }
  }

  for(unsigned int vi=0; vi<m_vectorPubTopic.size(); vi++) {
    std::string source = m_vectorPubTopic[vi];
    LOG("CloudBridgeBase add publisher: " << source);
    std::string topic;
    std::string msg;
    std::string qos;
    get_parameter(source + "." + "topic", topic);
    get_parameter(source + "." + "msg", msg);
    get_parameter(source + "." + "qos", qos);
    DEBUG("  - topic: " << topic << ", type: " << msg << ", qos_string: "<< qos);
    if(topic.compare("") != 0 && msg.compare("") != 0) {
      if (topic.rfind("/", 0) != 0 && m_vectorNamespace.size() > 0) {
        for(unsigned int ni=0; ni<m_vectorNamespace.size(); ni++) {
          string namespaced_topic = "/"+m_vectorNamespace[ni]+"/"+topic;
          DEBUG("  - topic: " << namespaced_topic << ", type: " << msg << ", qos_string: "<< qos);
          bridge_rcl_node_->add_publisher(namespaced_topic, msg, zmq_transport_, qos);
        }
      } else {
        DEBUG("  - topic: " << topic << ", type: " << msg << ", qos_string: "<< qos);
        bridge_rcl_node_->add_publisher(topic, msg, zmq_transport_, qos);
      }
    }
  }

  for(unsigned int vi=0; vi<m_vectorSrvServer.size(); vi++) {
    std::string source = m_vectorSrvServer[vi];
    LOG("CloudBridgeBase add srv server: " << source);
    std::string service;
    std::string srv;
    get_parameter(source + "." + "service", service);
    get_parameter(source + "." + "srv", srv);
    DEBUG("  - service: " << service << ", type: " << srv);
    if(service.compare("") != 0 && srv.compare("") != 0) {
      if (service.rfind("/", 0) != 0 && m_vectorNamespace.size() > 0) {
        for(unsigned int ni=0; ni<m_vectorNamespace.size(); ni++) {
          string namespaced_service = "/"+m_vectorNamespace[ni]+"/"+service;
          DEBUG("  - service: " << namespaced_service << ", type: " << srv);
          bridge_rcl_node_->add_service_server(namespaced_service, srv, zmq_transport_);
        }
      } else {
        DEBUG("  - service: " << service << ", srv: " << srv);
        bridge_rcl_node_->add_service_server(service, srv, zmq_transport_);
      }
    }
  }

  for(unsigned int vi=0; vi<m_vectorSrvClient.size(); vi++) {
    std::string source = m_vectorSrvClient[vi];
    LOG("CloudBridgeBase add srv client: " << source);
    std::string service;
    std::string srv;
    get_parameter(source + "." + "service", service);
    get_parameter(source + "." + "srv", srv);
    DEBUG("  - service: " << service << ", type: " << srv);
    if(service.compare("") != 0 && srv.compare("") != 0) {
      if (service.rfind("/", 0) != 0 && m_vectorNamespace.size() > 0) {
        for(unsigned int ni=0; ni<m_vectorNamespace.size(); ni++) {
          string namespaced_service = "/"+m_vectorNamespace[ni]+"/"+service;
          DEBUG("  - service: " << namespaced_service << ", type: " << srv);
          bridge_rcl_node_->add_service_client(namespaced_service, srv, zmq_transport_);
        }
      } else {
        DEBUG("  - service: " << service << ", srv: " << srv);
        bridge_rcl_node_->add_service_client(service, srv, zmq_transport_);
      }
    }
  }

  for(unsigned int vi=0; vi<m_vectorTfLookup.size(); vi++) {
    std::string source = m_vectorTfLookup[vi];
    std::string baseFrame;
    std::string childFrame;
    get_parameter(source + "." + "base_frame", baseFrame);
    get_parameter(source + "." + "child_frame", childFrame);

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
    // undeclare_parameter(source + "." + "base_frame");
    // undeclare_parameter(source + "." + "child_frame");
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
