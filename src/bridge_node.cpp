/**
 *  @file   bridge_node.cpp
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

#include "cloud_bridge/bridge_node.hpp"
#include "cloud_bridge/bridge_client.hpp"
#include "cloud_bridge/logging.hpp"

#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <cstring>
#include <chrono>

BridgeNode::BridgeNode(rcl_allocator_t *alloc, rcl_context_t *context, std::string ns)
    : running(true), alloc(alloc), context(context), node(rcl_get_zero_initialized_node()), wait(rcl_get_zero_initialized_wait_set())
{
    rcl_node_options_t opts = rcl_node_get_default_options();
    rcl_ret_t rc;

    rc = rcl_node_init(&node, "bridge", ns.c_str(), context, &opts);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_node_init failed: " << rc);
    }
    thread = std::thread(&BridgeNode::execute, this);
    auto node_handle = std::make_shared<rclcpp::Node>("tf_bridge", ns.c_str());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle);
}

BridgeNode::~BridgeNode()
{
    running = false;
    thread.join();

    rcl_ret_t rc;

    for (const auto &it : subscribers)
    {
        rc = rcl_subscription_fini(&it->sub, &node);
        if (rc != RCL_RET_OK)
        {
            ERROR("= rcl_subscription_fini failed: " << rc);
        }
    }

    for (auto it : publishers)
    {
        rc = rcl_publisher_fini(&it.second.pub, &node);
        if (rc != RCL_RET_OK)
        {
            ERROR("rcl_publisher_fini failed: " << rc);
        }
    }

    rc = rcl_wait_set_fini(&wait);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_wait_set_fini failed: " << rc);
    }

    rc = rcl_node_fini(&node);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_node_fini failed: " << rc);
    }
}

void BridgeNode::execute()
{
    size_t sub_count = 0;
    rcl_ret_t rc;

    const unsigned int timeout = 100; // msec
    while (running)
    {
        size_t new_sub_count;
        {
            std::lock_guard<std::mutex> lock(mutex);
            while (!actions.empty())
            {
                auto action = actions.front();
                action();
                actions.pop();
            }
            new_sub_count = subscribers.size();
        }

        if (new_sub_count == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
            continue;
        }

        if (new_sub_count != sub_count)
        {
            sub_count = new_sub_count;

            rc = rcl_wait_set_fini(&wait);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_fini failed: " << rc);
            }

            rc = rcl_wait_set_init(&wait, sub_count, 0, 0, 0, 0, 0, context, *alloc);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_init failed: " << rc);
            }
        }
        else
        {
            rc = rcl_wait_set_clear(&wait);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_clear failed: " << rc);
                return;
            }
        }

        for (const auto &it : subscribers)
        {
            size_t index;
            rc = rcl_wait_set_add_subscription(&wait, &it->sub, &index);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_add_subscription failed: " << rc);
            }
        }

        rc = rcl_wait(&wait, RCL_MS_TO_NS(timeout));
        if (rc == RCL_RET_TIMEOUT)
        {
            continue;
        }
        if (rc != RCL_RET_OK)
        {
            ERROR("rcl_wait failed: " << rc);
        }
        for (size_t i = 0; i < sub_count; i++)
        {
            if (wait.subscriptions[i])
            {
                Subscriber *sub = (Subscriber *)wait.subscriptions[i];
                handle_message(sub);
            }
        }
    }
}

void BridgeNode::remove(BridgeClient *client)
{
    auto action = [=]() {
        for (auto it = subscribers.begin(); it != subscribers.end(); /* empty */)
        {
            Subscriber *sub = it->get();
            if (sub->clients.find(client) != sub->clients.end())
            {
                DEBUG("Removing client from subscribers on " << sub->topic << " topic");
                sub->clients.erase(client);
                if (sub->clients.empty())
                {
                    LOG("Removing subscriber on " << sub->topic << " topic");

                    rcl_ret_t rc = rcl_subscription_fini(&sub->sub, &node);
                    if (rc != RCL_RET_OK)
                    {
                        ERROR("rcl_subscription_fini failed: " << rc);
                    }

                    it = subscribers.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            else
            {
                ++it;
            }
        }
    };

    {
        std::lock_guard<std::mutex> lock(mutex);
        actions.push(action);
    }

    for (auto it = publishers.begin(), eit = publishers.end(); it != eit; /* empty */)
    {
        if (it->second.clients.find(client) != it->second.clients.end())
        {
            DEBUG("Removing client from publishers on " << it->first << " topic");
            it->second.clients.erase(client);
            if (it->second.clients.empty())
            {
                LOG("Removing publisher on " << it->first << " topic");

                rcl_ret_t rc = rcl_publisher_fini(&it->second.pub, &node);
                if (rc != RCL_RET_OK)
                {
                    ERROR("rcl_publisher_fini failed: " << rc);
                }

                it = publishers.erase(it);
            }
            else
            {
                ++it;
            }
        }
        else
        {
            ++it;
        }
    }
}

void BridgeNode::add_subscriber(const std::string &topic, const std::string &type, BridgeClient *client)
{
    const MessageType *message_type = types.get(type);
    if (message_type == NULL)
    {
        return;
    }
    auto action = [=]() {
        for (auto it = subscribers.begin(), eit = subscribers.end(); it != eit; ++it)
        {
            Subscriber *sub = it->get();
            if (sub->topic == topic)
            {
                sub->clients.insert(client);
                return;
            }
        }

        std::unique_ptr<Subscriber> s(new Subscriber);
        s->sub = rcl_get_zero_initialized_subscription();
        s->type = message_type;
        s->topic = topic;
        s->clients.insert(client);

        rcl_subscription_options_t sub_opt = rcl_subscription_get_default_options();
        sub_opt.qos = rmw_qos_profile_sensor_data;
        sub_opt.qos.depth = 1;
        rcl_ret_t rc = rcl_subscription_init(&s->sub, &node, message_type->type_support, topic.c_str(), &sub_opt);
        if (rc != RCL_RET_OK)
        {
            ERROR("rcl_subscription_init failed: " << rc);
            return;
        }

        auto it = subscribers.insert(subscribers.end(), std::move(s));
        assert((void *)it->get() == (void *)&it->get()->sub);

        LOG("BridgeNode, Subscribed " << type << " on " << topic);
    };

    std::lock_guard<std::mutex> lock(mutex);
    actions.push(action);
}

void BridgeNode::add_publisher(const std::string &topic, const std::string &type, BridgeClient *client)
{
    auto it = publishers.find(topic);
    if (it != publishers.end())
    {
        it->second.clients.insert(client);
        return;
    }

    const MessageType *message_type = types.get(type);
    if (message_type == NULL)
    {
        return;
    }

    rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();

    rcl_ret_t rc = rcl_publisher_init(&pub, &node, message_type->type_support, topic.c_str(), &pub_opt);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_publisher_init failed: " << rc);
        return;
    }

    Publisher p;
    p.pub = pub;
    p.type = message_type;
    p.clients.insert(client);

    publishers.insert(std::make_pair(topic, p));

    LOG("Publishing " << type << " on " << topic);
}

void BridgeNode::publish(const std::string &topic, const std::vector<uint8_t> &data)
{
    if (topic == "tf")
    {
        rcl_serialized_message_t serialized_msg_ = rmw_get_zero_initialized_serialized_message();
        serialized_msg_.buffer = static_cast<unsigned char *>((void *)data.data());
        serialized_msg_.buffer_length = data.size();
        serialized_msg_.buffer_capacity = serialized_msg_.buffer_length;
        auto allocator = rcutils_get_default_allocator();
        serialized_msg_.allocator = allocator;
        auto transform_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
        auto tf_ts = rosidl_typesupport_cpp::get_message_type_support_handle<geometry_msgs::msg::TransformStamped>();
        auto ret = rmw_deserialize(&serialized_msg_, tf_ts, transform_msg.get());
        if (ret != RMW_RET_OK)
        {
            ERROR("failed to serialize serialized message topic tf");
            return;
        }
        tf_broadcaster_->sendTransform(*transform_msg);
    }
    else
    {
        auto it = publishers.find(topic);
        if (it == publishers.end())
        {
            ERROR("No publisher registered on topic " << topic << ", ignorning message");
            return;
        }

        const MessageType *type = it->second.type;
        rcl_publisher_t *pub = &it->second.pub;

        void *msg = malloc(type->size);
        if (msg)
        {
            if (type->init(msg))
            {
                DEBUG("Unserializing message for " << topic << " topic");
                if (Unserialize(msg, type->introspection, data))
                {
                    rcl_ret_t rc = rcl_publish(pub, msg, NULL);
                    if (rc != RCL_RET_OK)
                    {
                        ERROR("rcl_publish failed: " << rc);
                    }
                }

                type->fini(msg);
            }
            else
            {
                ERROR("Message init failed for " << topic << " topic");
            }

            free(msg);
        }
        else
        {
            ERROR("Out of memory when unserializing message on " << topic << " topic");
        }
    }
}

void BridgeNode::handle_tf(geometry_msgs::msg::TransformStamped& transform, BridgeClient *client)
{
    rcl_serialized_message_t serialized_msg_ = rmw_get_zero_initialized_serialized_message();
    size_t tf_msg_size = 0;
    tf_msg_size += 8 + transform.header.frame_id.size(); // std_msgs/Header/**
    tf_msg_size += transform.child_frame_id.size();      // child_frame_id
    tf_msg_size += 8 * 3 + 8 * 4;                        // geometry_msgs/Transform
    auto allocator = rcutils_get_default_allocator();
    auto initial_capacity = 0u;
    auto ret = rmw_serialized_message_init(
        &serialized_msg_,
        initial_capacity,
        &allocator);
    if (ret != RCL_RET_OK)
    {
        throw std::runtime_error("failed to initialize serialized message");
    }
    auto message_header_length = 8u;
    auto message_payload_length = static_cast<size_t>(tf_msg_size);
    ret = rmw_serialized_message_resize(
        &serialized_msg_, message_header_length + message_payload_length);
    if (ret != RCL_RET_OK)
    {
        throw std::runtime_error("failed to resize serialized message");
    }
    auto tf_ts =
        rosidl_typesupport_cpp::get_message_type_support_handle<geometry_msgs::msg::TransformStamped>();

    ret = rmw_serialize(&transform, tf_ts, &serialized_msg_);
    if (ret != RMW_RET_OK)
    {
        ERROR("failed to serialize serialized message topic tf");
        return;
    }
    std::vector<uint8_t> data;
    data.insert(data.end(), serialized_msg_.buffer, serialized_msg_.buffer + serialized_msg_.buffer_length);
    client->publish("tf", "geometry_msgs/TransformStamped", data);
    // Send(serialized_msg_.buffer, serialized_msg_.buffer_length, false, hashKey);
}

void BridgeNode::handle_message(Subscriber *sub)
{
    rcl_ret_t rc;

    void *msg = malloc(sub->type->size);
    if (msg)
    {
        if (sub->type->init(msg))
        {
            rmw_message_info_t info;
            rc = rcl_take(&sub->sub, msg, &info, NULL);
            if (rc == RCL_RET_OK)
            {
                // LOG("New message received " << sub->topic << " topic");

                std::vector<uint8_t> data;
                // data.reserve(4096);

                DEBUG("Serializing message for " << sub->topic << " topic");
                Serialize(msg, sub->type->introspection, data);

                std::string target_topic = sub->topic;
                if (sub->type->type_string == "tf2_msgs/TFMessage")
                {
                    for (auto tf_topic : tf_topics)
                    {
                        if (target_topic == tf_topic)
                        {
                            target_topic = "tf";
                            break;
                        }
                    }
                }
                for (auto client : sub->clients)
                {
                    client->publish(target_topic, sub->type->type_string, data);
                }
            }
            else if (rc == RCL_RET_SUBSCRIPTION_TAKE_FAILED)
            {
                // everything's fine, no message, ignore this
            }
            else
            {
                ERROR("rcl_take failed: " << rc);
            }

            sub->type->fini(msg);
        }
        else
        {
            ERROR("Message init failed on " << sub->topic << " topic");
        }

        free(msg);
    }
    else
    {
        ERROR("Out of memory when receiving message " << sub->topic << " topic");
    }
}
