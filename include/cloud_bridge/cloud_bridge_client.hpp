/**
 *  @file   cloud_bridge_client.hpp
 *  @date   2021-01-28
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

#ifndef _CLOUD_BRIDGE_BRIDGE_CLIENT_H_
#define _CLOUD_BRIDGE_BRIDGE_CLIENT_H_

#include "cloud_bridge/cloud_bridge_base.hpp"

using namespace std;

class CloudBridgeClient: public CloudBridgeBase
{
public:
  CloudBridgeClient(string nodeName);
  virtual ~CloudBridgeClient();

protected:
  bool Setup();
  bool Connect();

private:
  void SetPorts();
  std::vector<uint8_t> SetManageReq(uint8_t op, std::string topic, std::string type);
  bool SendManageRequest(const void* buffer, const int bufferLength, bool isNonBlockingMode, void** recvBuffer, int& recvBufferLength);

};
#endif // _CLOUD_BRIDGE_BRIDGE_CLIENT_H_
