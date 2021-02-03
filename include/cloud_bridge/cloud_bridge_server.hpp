/**
 *  @file   cloud_bridge_server.hpp
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

#ifndef _CLOUD_BRIDGE_BRIDGE_SERVER_H_
#define _CLOUD_BRIDGE_BRIDGE_SERVER_H_

#include "cloud_bridge/cloud_bridge_base.hpp"

using namespace std;

class CloudBridgeServer: public CloudBridgeBase
{
public:
  CloudBridgeServer(string nodeName);
  virtual ~CloudBridgeServer();

protected:
  bool Setup();
  bool Connect();

private:
  int m_iHostSubPort;
  int m_iHostPubPort;
  int m_iHostReqPort;
  int m_iHostRepPort;

  std::thread m_threadManageProc;

  void ReadManageProc();
  bool ReceiveManageRequest(void** buffer, int& bufferLength, bool isNonBlockingMode);
  bool SendManageReply(const void* buffer, const int bufferLength, bool isNonBlockingMode);

};
#endif // _CLOUD_BRIDGE_BRIDGE_SERVER_H_
