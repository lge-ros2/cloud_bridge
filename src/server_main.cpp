/**
 *  @file   server_main.cpp
 *  @date   2020-11-23
 *  @author Sungkyu Kang
 *  @brief
 *        Transfer ROS2 message bridge server using ZeroMQ
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2020 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 *
 *         SPDX-License-Identifier: MIT
 */

#include "cloud_bridge/cloud_bridge_server.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::sleep_for(3000ms);

    auto bridge = std::make_shared<CloudBridgeServer>("cloud_bridge_server");

    rclcpp::spin(bridge);
    rclcpp::shutdown();
}