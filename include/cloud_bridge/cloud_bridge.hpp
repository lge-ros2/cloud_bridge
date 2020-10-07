/**
 *  @file   cloud_bridge.hpp
 *  @date   2020-04-16
 *  @author Sungkyu Kang
 *  @brief
 *          Cloud Bridge class Transfer ROS2 message to cloud using ZeroMQ
 *  @remark
 *  @warning
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
#include "cloud_bridge/bridge_node.hpp"
#include "cloud_bridge/bridge_client.hpp"
#include "cloud_bridge/logging.hpp"
#include <zmq.h>
#include <vector>
#include <mutex>

using namespace std;


class CloudBridge: public rclcpp::Node
{
public:
  CloudBridge(string nodeName, bool isBind);
  virtual ~CloudBridge();

private:
  void initBridgeNode();
  void DeinitSimConnection();
  void InitSimConnection();
  bool Setup();
  void SetPorts();
  bool Connect();
  void Disconnect();
  bool Reconnect();
  void Start();
  void Stop();

  void ReadProc();

  void ReadManageProc();
  std::vector<uint8_t> SetManageReq(uint8_t op, std::string topic, std::string type);
  bool ReceiveManageRequest(void** buffer, int& bufferLength, bool isNonBlockingMode);
  bool SendManageReply(const void* buffer, const int bufferLength, bool isNonBlockingMode);
  bool SendManageRequest(const void* buffer, const int bufferLength, bool isNonBlockingMode, void** recvBuffer, int& recvBufferLength);
  uint32_t get32le(std::vector<uint8_t> buffer, size_t size) const;

  void TfProc(std::string frame_id, std::string child_frame_id);

private:
  rcl_context_t bridge_node_context_;
  rcl_init_options_t bridge_node_init_;
  rcl_allocator_t bridge_node_alloc_;
  BridgeNode* bridge_node_;

  BridgeClient* client;
  bool m_bIsServer;
  // from ros parameters  
  std::string m_strCloudIp;
  int m_iManagePort;
  int m_iSubPort;
  int m_iPubPort;
  int m_iReqPort;
  int m_iRepPort;

  std::vector<std::string> m_vectorSubTopic;
  std::vector<std::string> m_vectorPubTopic;
  std::vector<std::string> m_vectorSrvServer;
  std::vector<std::string> m_vectorSrvClient;

  rclcpp::Node::SharedPtr m_pNodeHandle;

  void* m_pZmqCtx;
  void* m_pManageSocket;
  void* m_pSubSocket;
  void* m_pPubSocket;
  void* m_pReqSocket;
  void* m_pRepSocket;
  bool m_bRun;

  std::thread m_threadProc;
  std::thread m_threadReqProc;
  std::thread m_threadManageProc;

  zmq_msg_t m_pManageMsg;
  zmq_msg_t m_pSubMsg;
  zmq_msg_t m_pReqMsg;
  zmq_msg_t m_pRepMsg;

  int m_iRetryRequestTime;

  std::mutex m_mutexManage;
  std::mutex m_mutexSend;

  std::vector<std::string> m_vectorTfLookup;
  std::vector<std::thread> m_vectorTfThread;
  std::vector<std::shared_ptr<tf2_ros::TransformListener>> m_vectorTfListener;
  std::vector<std::shared_ptr<tf2_ros::Buffer>> m_vectorTfBuffer;

};
#endif // _CLOUD_BRIDGE_BRIDGE_H_
