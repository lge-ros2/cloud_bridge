/**
 *  @file   cloud_bridge_base.hpp
 *  @date   2020-04-16
 *  @author Sungkyu Kang
 *  @brief
 *          Cloud Bridge class Transfer ROS2 message to cloud using ZeroMQ
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2020 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 *
 *         SPDX-License-Identifier: MIT
 */

#ifndef _CLOUD_BRIDGE_BRIDGE_H_
#define _CLOUD_BRIDGE_BRIDGE_H_

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/transform_listener.h>
#include "cloud_bridge/bridge_rcl_node.hpp"
#include "cloud_bridge/zmq_transport.hpp"
#include "cloud_bridge/logging.hpp"
#include <zmq.h>
#include <vector>
#include <mutex>

using namespace std;


class CloudBridgeBase: public rclcpp::Node
{
public:
  CloudBridgeBase(string nodeName);
  virtual ~CloudBridgeBase();

protected:
  virtual bool Setup() { return true; };
  virtual bool Connect() { return true; };
  void initBridgeRclNode();
  void initBridgeParams();

  uint32_t get32le(std::vector<uint8_t> buffer, size_t size) const;

  std::string m_strCloudIp;
  int m_iManagePort;
  int m_iSubPort;
  int m_iPubPort;
  int m_iReqPort;
  int m_iRepPort;

  int m_iHostSubPort;
  int m_iHostPubPort;
  int m_iHostReqPort;
  int m_iHostRepPort;

  void* m_pZmqCtx;
  void* m_pManageSocket;
  void* m_pSubSocket;
  void* m_pPubSocket;
  void* m_pReqSocket;
  void* m_pRepSocket;
  bool m_bRun;

  zmq_msg_t m_pManageMsg;
  std::mutex m_mutexManage;

  rclcpp::Node::SharedPtr m_pNodeHandle;
  ZmqTransport* zmq_transport_;
private:
  void Disconnect();

  rcl_context_t bridge_node_context_;
  rcl_init_options_t bridge_node_init_;
  rcl_allocator_t bridge_node_alloc_;
  BridgeRclNode* bridge_rcl_node_;
  
  std::vector<std::string> m_vectorSubTopic;
  std::vector<std::string> m_vectorPubTopic;
  std::vector<std::string> m_vectorParams;
  std::vector<std::string> m_vectorSrvServer;
  std::vector<std::string> m_vectorSrvClient;

  std::vector<std::string> m_vectorTfLookup;
  std::vector<std::thread> m_vectorTfThread;
  std::vector<std::shared_ptr<tf2_ros::TransformListener>> m_vectorTfListener;
  std::vector<std::shared_ptr<tf2_ros::Buffer>> m_vectorTfBuffer;

};
#endif // _CLOUD_BRIDGE_BRIDGE_H_
