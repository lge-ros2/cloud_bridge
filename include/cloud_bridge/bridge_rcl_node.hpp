/**
 *  @file   bridge_node.hpp
 *  @date   2020-09-21
 *  @author Sungkyu Kang
 *  @brief
 *        Bridge Node class for Cloud Bridge
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 *
 *         SPDX-License-Identifier: MIT
 */

#ifndef _CLOUD_BRIDGE_RCL_NODE_H_
#define _CLOUD_BRIDGE_RCL_NODE_H_

#include "cloud_bridge/message_types.hpp"

#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <memory>
#include <queue>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include <rcl/rcl.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ZmqTransport;

class BridgeRclNode
{
public:
    BridgeRclNode(rcl_allocator_t* alloc, rcl_context_t* context, std::string ns);
    ~BridgeRclNode();

    void add_subscriber(const std::string& topic, const std::string& type,
        ZmqTransport* zmq_transport, std::string& qos);
    void add_publisher(const std::string& topic, const std::string& type,
        ZmqTransport* zmq_transport, std::string& qos);
    void publish(const std::string& topic, const std::vector<uint8_t>& data);

    void add_service_server(const std::string& service, const std::string& type,
        ZmqTransport* zmq_transport);
    void add_service_client(const std::string& service, const std::string& type,
        ZmqTransport* zmq_transport);
    void service(const std::string& topic, 
        const std::vector<uint8_t>& req_data, std::vector<uint8_t>& res_data);

    void add_tf_listener(const std::string& topic, ZmqTransport* zmq_transport);
    void handle_tf(geometry_msgs::msg::TransformStamped& transform, ZmqTransport* zmq_transport);

private:
    rcl_allocator_t* alloc;
    rcl_context_t* context;
    rcl_node_t node;
    rcl_wait_set_t wait_set_sub_;
    rcl_wait_set_t wait_set_service_;

    std::mutex sub_mutex;
    std::queue<std::function<void()>> add_sub_actions;
    std::mutex service_mutex;
    std::queue<std::function<void()>> add_service_actions;

    volatile bool running;
    std::thread thread;
    std::thread service_thread;
    
    struct Publisher
    {
        rcl_publisher_t pub;
        const MessageType* type;
        ZmqTransport* zmq_transport;
    };
    typedef std::unordered_map<std::string, Publisher> Publishers;
    Publishers publishers;

    struct Subscriber
    {
        rcl_subscription_t sub;
        const MessageType* type;
        std::string topic;
        ZmqTransport* zmq_transport;
    };
    typedef std::vector<std::unique_ptr<Subscriber>> Subscribers;
    Subscribers subscribers;

    struct ServiceServer
    {
        rcl_service_t rcl_service;
        const MessageType* type_req;
        const MessageType* type_res;
        std::string topic;
        ZmqTransport* zmq_transport;
    };
    typedef std::vector<std::unique_ptr<ServiceServer>> ServiceServers;
    ServiceServers service_servers;

    struct ServiceClient
    {
        rcl_client_t rcl_client;
        const MessageType* type_req;
        const MessageType* type_res;
        std::string topic;
        ZmqTransport* zmq_transport;
    };
    typedef std::unordered_map<std::string, ServiceClient> ServiceClients;
    ServiceClients service_clients;

    MessageTypes types;

    void spin_subscribe();
    void handle_message(Subscriber* sub);

    void spin_service_server();
    void handle_service(ServiceServer *activate_server);

    rmw_qos_profile_t parseQosString(std::string qos_string);

    BridgeRclNode(const BridgeRclNode&) = delete;
    BridgeRclNode& operator = (const BridgeRclNode&) = delete;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // _CLOUD_BRIDGE_RCL_NODE_H_
