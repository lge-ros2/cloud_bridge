/**
 *  @file   bridge_node.hpp
 *  @date   2020-09-21
 *  @author Sungkyu Kang
 *  @brief
 *        Bridge Node class for Cloud Bridge
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
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

    void add_subscriber(const std::string& topic, const std::string& type, BridgeClient* client);
    void add_publisher(const std::string& topic, const std::string& type, BridgeClient* client);
    void add_tf_listener(const std::string& topic, BridgeClient* client);

    void publish(const std::string& topic, const std::vector<uint8_t>& data);
    void handle_tf(geometry_msgs::msg::TransformStamped& transform, BridgeClient* client);

private:
    rcl_allocator_t* alloc;
    rcl_context_t* context;
    rcl_node_t node;
    rcl_wait_set_t wait;
    std::unordered_set<std::string> tf_topics;

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
        // keep rcl_subscription_t as first member of this struct
        // other code relies on this fact!
        rcl_subscription_t sub;
        const MessageType* type;
        std::string topic;
        std::unordered_set<BridgeClient*> clients;
    };

    typedef std::vector<std::unique_ptr<Subscriber>> Subscribers;
    Subscribers subscribers;

    MessageTypes types;

    void execute();
    void handle_message(Subscriber* sub);

    BridgeNode(const BridgeNode&) = delete;
    BridgeNode& operator = (const BridgeNode&) = delete;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // _CLOUD_BRIDGE_NODE_H_