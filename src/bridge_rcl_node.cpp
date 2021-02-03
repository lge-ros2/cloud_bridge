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

#include "cloud_bridge/bridge_rcl_node.hpp"
#include "cloud_bridge/zmq_transport.hpp"
#include "cloud_bridge/logging.hpp"

#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <cstring>
#include <chrono>
#include <unistd.h>

#include <std_msgs/msg/string.hpp>

BridgeRclNode::BridgeRclNode(rcl_allocator_t *alloc, rcl_context_t *context, std::string ns)
    : running(true), alloc(alloc), context(context), node(rcl_get_zero_initialized_node()), 
    wait_set_sub_(rcl_get_zero_initialized_wait_set()),
    wait_set_service_(rcl_get_zero_initialized_wait_set())
{
    running = true;
    rcl_node_options_t opts = rcl_node_get_default_options();
    rcl_ret_t rc;

    rc = rcl_node_init(&node, "cloud_bridge_rcl_node", ns.c_str(), context, &opts);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_node_init failed: " << rc);
    }
    thread = std::thread(&BridgeRclNode::spin_subscribe, this);
    service_thread = std::thread(&BridgeRclNode::spin_service_server, this);
    auto node_handle = std::make_shared<rclcpp::Node>("cloud_bridge_tf_node", ns.c_str());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle);
}

BridgeRclNode::~BridgeRclNode()
{
    running = false;
    thread.join();
    service_thread.join();

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

    rc = rcl_wait_set_fini(&wait_set_sub_);
    if (rc != RCL_RET_OK)
    {
        ERROR("sub rcl_wait_set_fini failed: " << rc);
    }

    rc = rcl_wait_set_fini(&wait_set_service_);
    if (rc != RCL_RET_OK)
    {
        ERROR("service rcl_wait_set_fini failed: " << rc);
    }

    rc = rcl_node_fini(&node);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_node_fini failed: " << rc);
    }
}

void BridgeRclNode::spin_subscribe()
{
    size_t sub_count = 0;
    rcl_ret_t rc;

    const unsigned int timeout = 100; // msec
    running = true;
    while (running)
    {
        size_t new_sub_count;
        {
            std::lock_guard<std::mutex> lock(sub_mutex);
            while (!add_sub_actions.empty())
            {
                auto action = add_sub_actions.front();
                action();
                add_sub_actions.pop();
            }
            new_sub_count = subscribers.size();
        }

        if (new_sub_count == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
            continue;
        }

        if (new_sub_count != sub_count )
        {
            sub_count = new_sub_count;

            rc = rcl_wait_set_fini(&wait_set_sub_);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_fini failed: " << rc);
            }

            rc = rcl_wait_set_init(&wait_set_sub_, sub_count, 0, 0, 0, 0, 0, context, *alloc);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_init failed: " << rc);
            }
        }
        else
        {
            rc = rcl_wait_set_clear(&wait_set_sub_);
            if (rc != RCL_RET_OK)
            {
                ERROR("sub rcl_wait_set_clear failed: " << rc);
                return;
            }
        }

        for (const auto &it : subscribers)
        {
            size_t index;
            rc = rcl_wait_set_add_subscription(&wait_set_sub_, &it->sub, &index);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_add_subscription failed: " << rc);
            }
        }

        rc = rcl_wait(&wait_set_sub_, RCL_MS_TO_NS(timeout));
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
            if (wait_set_sub_.subscriptions[i])
            {
                Subscriber *sub = (Subscriber *)wait_set_sub_.subscriptions[i];
                handle_message(sub);
            }
        }
    }
}

void BridgeRclNode::spin_service_server()
{
    size_t service_count = 0;
    rcl_ret_t rc;

    const unsigned int timeout = 100; // msec
    while (running)
    {
        size_t new_service_count;
        {
            std::lock_guard<std::mutex> lock(service_mutex);
            while (!add_service_actions.empty())
            {
                auto action = add_service_actions.front();
                action();
                add_service_actions.pop();
            }
            new_service_count = service_servers.size();
        }

        if (new_service_count == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
            continue;
        }

        if (new_service_count != service_count)
        {
            service_count = new_service_count;

            rc = rcl_wait_set_fini(&wait_set_service_);
            if (rc != RCL_RET_OK)
            {
                ERROR("service rcl_wait_set_fini failed: " << rc);
            }

            rc = rcl_wait_set_init(&wait_set_service_, 0, 0, 0, 0, service_count, 0, context, *alloc);
            if (rc != RCL_RET_OK)
            {
                ERROR("service rcl_wait_set_init failed: " << rc);
            }
        }
        else
        {
            rc = rcl_wait_set_clear(&wait_set_service_);
            if (rc != RCL_RET_OK)
            {
                ERROR("service rcl_wait_set_clear failed: " << rc);
                return;
            }
        }

        for (const auto &it : service_servers)
        {
            size_t index;
            rc = rcl_wait_set_add_service(&wait_set_service_, &it->rcl_service, &index);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_add_service failed: " << rc);
            }
        }
        
        rc = rcl_wait(&wait_set_service_, RCL_MS_TO_NS(timeout));
        if (rc == RCL_RET_TIMEOUT)
        {
            continue;
        }
        if (rc != RCL_RET_OK)
        {
            ERROR("rcl_wait failed: " << rc);
        }

        for (size_t i = 0; i < service_count; ++i) {
            if (wait_set_service_.services[i])
            {
                ServiceServer *service_server = (ServiceServer *) wait_set_service_.services[i];
                handle_service(service_server);
            }
        }
    }
}



rmw_qos_profile_t BridgeRclNode::parseQosString(std::string qos_string)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
//   qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
//   if(qos_string == "sensor_data") {
//     qos = rmw_qos_profile_sensor_data;
//   } else if(qos_string == "parameters") {
//     qos = rmw_qos_profile_parameters;
//   } else if(qos_string == "parameter_events") {
//     qos = rmw_qos_profile_parameter_events;
//   } else if(qos_string == "system_default") {
//     qos = rmw_qos_profile_system_default;
//   } else if(qos_string == "services_default") {
//     qos = rmw_qos_profile_services_default;
//   }

  return qos;
}

void BridgeRclNode::add_subscriber(const std::string &topic, const std::string &type,
    ZmqTransport *zmq_transport, std::string &qos)
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
                return;
            }
        }

        std::unique_ptr<Subscriber> s(new Subscriber);
        s->sub = rcl_get_zero_initialized_subscription();
        s->type = message_type;
        s->topic = topic;
        s->zmq_transport = zmq_transport;

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

        DEBUG("BridgeRclNode, Subscribed " << type << " on " << topic);
    };

    std::lock_guard<std::mutex> lock(sub_mutex);
    add_sub_actions.push(action);
}

void BridgeRclNode::add_publisher(const std::string &topic, const std::string &type,
    ZmqTransport *zmq_transport, std::string &qos)
{
    auto it = publishers.find(topic);
    if (it != publishers.end())
    {
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
    p.zmq_transport = zmq_transport;

    publishers.insert(std::make_pair(topic, p));

    DEBUG("Publishing " << type << " on " << topic);
}

void BridgeRclNode::add_service_server(const std::string& topic, const std::string& type,
    ZmqTransport* zmq_transport)
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
                return;
            }
        }

        std::unique_ptr<ServiceServer> new_service_server(new ServiceServer);
        new_service_server->rcl_service = rcl_get_zero_initialized_service();
        new_service_server->type_req = type_req;
        new_service_server->type_res = type_res;
        new_service_server->topic = topic;
        new_service_server->zmq_transport = zmq_transport;

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

        DEBUG("Service Served " << type << " on " << topic);
    };

    std::lock_guard<std::mutex> lock(service_mutex);
    add_service_actions.push(action);
}

void BridgeRclNode::add_service_client(const std::string& topic, const std::string& type,
    ZmqTransport* zmq_transport)
{
    auto it = service_clients.find(topic);
    if (it != service_clients.end())
    {
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
    // wait_for_server_to_be_available(&node, &rcl_client, 10, 1000);

    ServiceClient service_client;
    service_client.rcl_client = rcl_client;
    service_client.type_req = type_req;
    service_client.type_res = type_res;
    service_client.topic = topic;
    service_client.zmq_transport = zmq_transport;
    service_clients.insert(std::make_pair(topic, service_client));

    DEBUG("Service Client " << type << " on " << topic);
}

void BridgeRclNode::publish(const std::string &topic, const std::vector<uint8_t> &data)
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
        DEBUG("BridgeRclNode::publish tf_broadcaster_->sendTransform");
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
                    DEBUG("BridgeRclNode rcl_publish topic "<< topic);
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

void BridgeRclNode::handle_tf(geometry_msgs::msg::TransformStamped& transform, ZmqTransport *zmq_transport)
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
    DEBUG("BridgeRclNode::handle_tf zmq_transport->send_publish");
    zmq_transport->send_publish("tf", "geometry_msgs/TransformStamped", data);
}

void BridgeRclNode::handle_message(Subscriber *sub)
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
                std::vector<uint8_t> data;
                // data.reserve(4096);

                DEBUG("Serializing message for " << sub->topic << " topic");
                Serialize(msg, sub->type->introspection, data);
            
                std::string target_topic = sub->topic;
                DEBUG("BridgeRclNode zmq_transport->send_publish topic: " << target_topic);
                sub->zmq_transport->send_publish(target_topic, sub->type->type_string, data);
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

void BridgeRclNode::service(const std::string& topic, 
    const std::vector<uint8_t>& req_data, std::vector<uint8_t>& res_data)
{
    auto it = service_clients.find(topic);
    if (it == service_clients.end())
    {
        ERROR("No service client registered on topic " << topic << ", ignorning message");
        return;
    }
    
    const MessageType *type_req = it->second.type_req;
    const MessageType *type_res = it->second.type_res;
    rcl_client_t *rcl_client = &it->second.rcl_client;

    void *req_msg = malloc(type_req->size);
    void *res_msg = malloc(type_res->size);
    if (req_msg)
    {
        if (type_req->init(req_msg))
        {
            DEBUG("Unserializing req_msg for " << topic << " topic");
            if (Unserialize(req_msg, type_req->introspection, req_data))
            {
                int64_t sequence_number = 0;
                rcl_ret_t rc = rcl_send_request(rcl_client, req_msg, &sequence_number);
                DEBUG("BridgeRclNode rcl_send_request sequence_number: "<< sequence_number);
                if (rc != RCL_RET_OK)
                {
                    ERROR("rcl_send_request failed: " << rc);
                }
                rmw_request_id_t request_id;
                rc = rcl_take_response(rcl_client, &request_id, res_msg);
                while (rc != RCL_RET_OK)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // ERROR("rcl_take_response failed: " << rc );
                    rc = rcl_take_response(rcl_client, &request_id, res_msg);
                }
                DEBUG("BridgeRclNode rcl_take_response sequence_number: "<< request_id.sequence_number);
                DEBUG("Serialize response msg topic "<< topic);
                Serialize(res_msg, type_res->introspection, res_data);
            }

            type_req->fini(req_msg);
            type_res->fini(res_msg);
        }
        else
        {
            ERROR("Message init failed for " << topic << " topic");
        }

        free(req_msg);
        free(res_msg);
    }
    else
    {
        ERROR("Out of memory when unserializing message on " << topic << " topic");
    }
}

void BridgeRclNode::handle_service(ServiceServer *activate_server)
{
    DEBUG("handle_service " << activate_server->topic << " service");
    rcl_ret_t rc;

    void *req_msg = malloc(activate_server->type_req->size);
    void *res_msg = malloc(activate_server->type_res->size);
    if (req_msg)
    {
        if (activate_server->type_req->init(req_msg))
        {
            DEBUG("rcl_take_request " << activate_server->topic << " service");
            rmw_request_id_t request_id; 
            rc = rcl_take_request(&activate_server->rcl_service, &request_id, req_msg);
            if (rc == RCL_RET_OK)
            {
                std::vector<uint8_t> req_data;
                // req_data.reserve(4096);

                DEBUG("Serializing req_msg for " << activate_server->topic << " service");
                Serialize(req_msg, activate_server->type_req->introspection, req_data);

                std::string target_topic = activate_server->topic;
                auto zmq_transport = activate_server->zmq_transport;

                DEBUG("BridgeRclNode client->send_request_and_get_response topic: " << activate_server->topic);
                std::vector<uint8_t> res_data;
                zmq_transport->send_request_and_get_response(target_topic, req_data, res_data);

                DEBUG("Unserializing res_msg for " << activate_server->topic << " topic, res_size: " << res_data.size());
                if (Unserialize(res_msg, activate_server->type_res->introspection, res_data))
                {
                    DEBUG("BridgeRclNode rcl_send_response topic "<< activate_server->topic);
                    rc = rcl_send_response(&activate_server->rcl_service, &request_id, res_msg);
                    if (rc != RCL_RET_OK)
                    {
                        ERROR("rcl_publish failed: " << rc);
                    }
                }
            }
            else
            {
                ERROR("rcl_take_request failed: " << rc);
            }

            activate_server->type_req->fini(req_msg);
            activate_server->type_res->fini(res_msg);
        }
        else
        {
            ERROR("Message init failed on " << activate_server->topic << " topic");
        }

        free(req_msg);
        free(res_msg);
    }
    else
    {
        ERROR("Out of memory when receiving message " << activate_server->topic << " topic");
    }
}