/**
 *  @file   message_types.hpp
 *  @date   2020-09-21
 *  @author Sungkyu Kang
 *  @brief
 *        Message type for converting string to msg struct
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 *
 *         SPDX-License-Identifier: MIT
 */

#ifndef _CLOUD_BRIDGE_MESSAGE_TYPE_H_
#define _CLOUD_BRIDGE_MESSAGE_TYPE_H_

#include <string>
#include <vector>
#include <mutex>
#include <unordered_map>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>

struct MessageType
{
    const rosidl_message_type_support_t* type_support;
    const rosidl_message_type_support_t* introspection;

    std::string type_string;
    size_t size;
    bool (*init)(void* msg);
    void (*fini)(void* msg);
};

class MessageTypes
{
public:
    MessageTypes();
    ~MessageTypes();

    // thread-safe
    const MessageType* get(const std::string& type, const std::string& category="message");
    const rosidl_service_type_support_t* get_srv_type_support(const std::string& type);

private:
    typedef std::unordered_map<std::string, void*> Libraries;
    typedef std::unordered_map<std::string, MessageType> Messages;
    
    Libraries libraries;
    Messages messages;

    std::mutex mutex;

    void* load_lib(const std::string& name);
    static void unload_lib(void* handle);
    static void* getsym(void* lib, const std::string& name);

    MessageTypes(const MessageTypes&) = delete;
    MessageTypes& operator = (const MessageTypes&) = delete;
};

bool Unserialize(void* msg, const rosidl_message_type_support_t* type, const std::vector<uint8_t>& data, size_t offset = 0);
void Serialize(void* msg, const rosidl_message_type_support_t* type, std::vector<uint8_t>& data);

#endif // _CLOUD_BRIDGE_MESSAGE_TYPE_H_