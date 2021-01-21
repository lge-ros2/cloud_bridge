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

#ifndef _CLOUD_BRIDGE_NODE_H_
#define _CLOUD_BRIDGE_NODE_H_

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

class BridgeClient;

class BridgeNode
{
public:
    BridgeNode(rcl_allocator_t* alloc, rcl_context_t* context, std::string ns);
    ~BridgeNode();

    void remove(BridgeClient* client);

    void add_subscriber(const std::string& topic, const std::string& type,
        BridgeClient* client, std::string& qos);
    void add_publisher(const std::string& topic, const std::string& type,
        BridgeClient* client, std::string& qos);

    void add_service_server(const std::string& service, const std::string& type,
        BridgeClient* client);
    void add_service_client(const std::string& service, const std::string& type,
        BridgeClient* client);

    void add_tf_listener(const std::string& topic, BridgeClient* client);

    void publish(const std::string& topic, const std::vector<uint8_t>& data);
    void handle_tf(geometry_msgs::msg::TransformStamped& transform, BridgeClient* client);

    rmw_qos_profile_t parseQosString(std::string qos_string);

private:
    rcl_allocator_t* alloc;
    rcl_context_t* context;
    rcl_node_t node;
    rcl_wait_set_t wait_set_;

    std::mutex mutex;
    std::queue<std::function<void()>> actions;

    volatile bool running;
    std::thread thread;
    struct Publisher
    {
        rcl_publisher_t pub;
        const MessageType* type;
        std::unordered_set<BridgeClient*> clients;
    };
    typedef std::unordered_map<std::string, Publisher> Publishers;
    Publishers publishers;

    struct Subscriber
    {
        rcl_subscription_t sub;
        const MessageType* type;
        std::string topic;
        std::unordered_set<BridgeClient*> clients;
    };
    typedef std::vector<std::unique_ptr<Subscriber>> Subscribers;
    Subscribers subscribers;

    struct ServiceServer
    {
        rcl_service_t rcl_service;
        const MessageType* type_req;
        const MessageType* type_res;
        std::string topic;
        std::unordered_set<BridgeClient*> clients;
    };
    typedef std::vector<std::unique_ptr<ServiceServer>> ServiceServers;
    ServiceServers service_servers;

    struct ServiceClient
    {
        rcl_client_t rcl_client;
        const MessageType* type_req;
        const MessageType* type_res;
        std::string topic;
        std::unordered_set<BridgeClient*> clients;
    };
    typedef std::unordered_map<std::string, ServiceClient> ServiceClients;
    ServiceClients service_clients;

    MessageTypes types;

    void execute();
    void handle_message(Subscriber* sub);
    void handle_service(ServiceServer* ss);

    BridgeNode(const BridgeNode&) = delete;
    BridgeNode& operator = (const BridgeNode&) = delete;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // _CLOUD_BRIDGE_NODE_H_