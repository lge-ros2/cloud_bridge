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

#include <std_msgs/msg/string.hpp>

BridgeNode::BridgeNode(rcl_allocator_t *alloc, rcl_context_t *context, std::string ns)
    : running(true), alloc(alloc), context(context), node(rcl_get_zero_initialized_node()), wait_set_(rcl_get_zero_initialized_wait_set())
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

    rc = rcl_wait_set_fini(&wait_set_);
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
    size_t service_count = 0;
    rcl_ret_t rc;

    const unsigned int timeout = 100; // msec
    while (running)
    {
        size_t new_sub_count;
        size_t new_service_count;
        {
            std::lock_guard<std::mutex> lock(mutex);
            while (!actions.empty())
            {
                auto action = actions.front();
                action();
                actions.pop();
            }
            new_sub_count = subscribers.size();
            new_service_count = service_servers.size();
        }

        if (new_sub_count == 0 && new_service_count == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
            continue;
        }

        if (new_sub_count != sub_count || new_service_count != service_count)
        {
            sub_count = new_sub_count;
            service_count = new_service_count;

            rc = rcl_wait_set_fini(&wait_set_);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_fini failed: " << rc);
            }

            rc = rcl_wait_set_init(&wait_set_, sub_count, 0, 0, 0, service_count, 0, context, *alloc);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_init failed: " << rc);
            }
        }
        else
        {
            rc = rcl_wait_set_clear(&wait_set_);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_clear failed: " << rc);
                return;
            }
        }

        for (const auto &it : subscribers)
        {
            size_t index;
            rc = rcl_wait_set_add_subscription(&wait_set_, &it->sub, &index);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_add_subscription failed: " << rc);
            }
        }
        // std::cout << "*************************" << std::endl;
        rc = rcl_wait(&wait_set_, RCL_MS_TO_NS(timeout));
        if (rc == RCL_RET_TIMEOUT)
        {
            // std::cout << "RCL_RET_TIMEOUT: " << sub_count << std::endl;
            continue;
        }
        if (rc != RCL_RET_OK)
        {
            ERROR("rcl_wait failed: " << rc);
        }
        // std::cout << "sub_count: " << sub_count << std::endl;
        for (size_t i = 0; i < sub_count; i++)
        {
            if (wait_set_.subscriptions[i])
            {
                Subscriber *sub = (Subscriber *)wait_set_.subscriptions[i];
                // std::cout << "handle_message index: " << i << std::endl;
                handle_message(sub);
            }
        }
        for (size_t i = 0; i < service_count; ++i) {
            if (wait_set_.services[i])
            {
                ServiceServer *ss = (ServiceServer *)wait_set_.services[i];
                // std::cout << "handle_message index: " << i << std::endl;
                handle_service(ss);
            }
        }
        // std::cout << "-----------------------------" << std::endl;
    }
}

rmw_qos_profile_t BridgeNode::parseQosString(std::string qos_string)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  if(qos_string == "sensor_data") {
    LOG("parseQosString rmw_qos_profile_sensor_data");
    qos = rmw_qos_profile_sensor_data;
  } else if(qos_string == "parameters") {
    LOG("parseQosString rmw_qos_profile_parameters");
    qos = rmw_qos_profile_parameters;
  } else if(qos_string == "parameter_events") {
    LOG("parseQosString rmw_qos_profile_parameter_events");
    qos = rmw_qos_profile_parameter_events;
  } else if(qos_string == "system_default") {
    LOG("parseQosString rmw_qos_profile_system_default");
    qos = rmw_qos_profile_system_default;
  } else if(qos_string == "services_default") {
    LOG("parseQosString rmw_qos_profile_services_default");
    qos = rmw_qos_profile_services_default;
  } else{
    LOG("parseQosString rmw_qos_profile_default");
  }

  return qos;
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

void BridgeNode::add_subscriber(const std::string &topic, const std::string &type,
    BridgeClient *client, std::string &qos)
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
        sub_opt.qos = parseQosString(qos);

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

void BridgeNode::add_publisher(const std::string &topic, const std::string &type,
    BridgeClient *client, std::string &qos)
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
    pub_opt.qos = parseQosString(qos);

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

void BridgeNode::add_service_server(const std::string& topic, const std::string& type,
    BridgeClient* client)
{
    const MessageType *type_req = types.get(type, "request");
    const MessageType *type_res = types.get(type, "response");
    const rosidl_service_type_support_t *srv_type_support = types.get_srv_type_support(type);
    if (type_req == NULL || type_res == NULL)
    {
        return;
    }
    auto action = [=]() {
        for (auto it = service_servers.begin(), eit = service_servers.end(); it != eit; ++it)
        {
            ServiceServer *service_server = it->get();
            if (service_server->topic == topic)
            {
                service_server->clients.insert(client);
                return;
            }
        }

        std::unique_ptr<ServiceServer> new_service_server(new ServiceServer);
        new_service_server->rcl_service = rcl_get_zero_initialized_service();
        new_service_server->type_req = type_req;
        new_service_server->type_res = type_res;
        new_service_server->topic = topic;
        new_service_server->clients.insert(client);

        rcl_service_options_t service_options = rcl_service_get_default_options();

        rcl_ret_t rc = rcl_service_init(&new_service_server->rcl_service, &node, 
            srv_type_support, topic.c_str(), &service_options);
        if (rc != RCL_RET_OK)
        {
            ERROR("rcl_service_init failed: " << rc);
            return;
        }

        auto it = service_servers.insert(service_servers.end(), std::move(new_service_server));
        assert((void *)it->get() == (void *)&it->get()->rcl_service);

        LOG("Service Served " << type << " on " << topic);
    };

    std::lock_guard<std::mutex> lock(mutex);
    actions.push(action);

}

void BridgeNode::add_service_client(const std::string& topic, const std::string& type,
    BridgeClient* client)
{
    auto it = service_clients.find(topic);
    if (it != service_clients.end())
    {
        it->second.clients.insert(client);
        return;
    }

    const MessageType *type_req = types.get(type, "request");
    const MessageType *type_res = types.get(type, "response");
    const rosidl_service_type_support_t *srv_type_support = types.get_srv_type_support(type);
    if (type_req == NULL || type_res == NULL)
    {
        return;
    }
    rcl_client_t rcl_client = rcl_get_zero_initialized_client();
    rcl_client_options_t client_options = rcl_client_get_default_options();
    rcl_ret_t rc = rcl_client_init(&rcl_client, &node, 
        srv_type_support, topic.c_str(), &client_options);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_client_init failed: " << rc);
        return;
    }
    ServiceClient sc;
    sc.rcl_client = rcl_client;
    sc.type_req = type_req;
    sc.type_res = type_res;
    sc.topic = topic;
    sc.clients.insert(client);
    service_clients.insert(std::make_pair(topic, sc));

    LOG("Service Client " << type << " on " << topic);

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
        DEBUG("BridgeNode::publish tf_broadcaster_->sendTransform");
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
                    LOG("BridgeNode rcl_publish topic "<< topic);
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
    DEBUG("bridgeNode::handle_message handle_tf publish client->publish");
    client->publish("tf", "geometry_msgs/TransformStamped", data);
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
                // LOG("Removing subscriber on " << sub->topic << " topic");
                // LOG("New message received " << sub->topic << " topic");

                std::vector<uint8_t> data;
                // data.reserve(4096);

                DEBUG("Serializing message for " << sub->topic << " topic");
                Serialize(msg, sub->type->introspection, data);

                std::string target_topic = sub->topic;
                for (auto client : sub->clients)
                {
                    LOG("BridgeNode client->publish topic: " << target_topic);
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

void BridgeNode::handle_service(ServiceServer* service_server)
{
    rcl_ret_t rc;

    void *req_msg = malloc(service_server->type_req->size);
    void *res_msg = malloc(service_server->type_res->size);
    if (req_msg)
    {
        if (service_server->type_req->init(req_msg))
        {
            rmw_request_id_t header;
            rc = rcl_take_request(&service_server->rcl_service, &header, &req_msg);
            if (rc == RCL_RET_OK)
            {
                LOG("New Service received " << service_server->topic << " service");

                std::vector<uint8_t> req_data;
                // data.reserve(4096);

                LOG("Serializing message for " << service_server->topic << " service");
                Serialize(req_msg, service_server->type_req->introspection, req_data);

                std::string target_topic = service_server->topic;
                for (auto client : service_server->clients)
                {
                    LOG("BridgeNode client->send_request topic: " << service_server->topic);
                    std::vector<uint8_t> res_data;
                    client->send_request(
                        target_topic, service_server->type_req->type_string, req_data, res_data);

                    LOG("Unserializing message for " << service_server->topic << " topic");
                    if (Unserialize(res_msg, service_server->type_res->introspection, res_data))
                    {
                        LOG("BridgeNode rcl_publish topic "<< service_server->topic);
                        rc = rcl_send_response(&service_server->rcl_service, &header, res_msg);
                        if (rc != RCL_RET_OK)
                        {
                            ERROR("rcl_publish failed: " << rc);
                        }
                    }
                }
            }
            else
            {
                ERROR("rcl_take_request failed: " << rc);
            }

            service_server->type_req->fini(req_msg);
            service_server->type_res->fini(res_msg);
        }
        else
        {
            ERROR("Message init failed on " << service_server->topic << " topic");
        }

        free(req_msg);
        free(res_msg);
    }
    else
    {
        ERROR("Out of memory when receiving message " << service_server->topic << " topic");
    }
}