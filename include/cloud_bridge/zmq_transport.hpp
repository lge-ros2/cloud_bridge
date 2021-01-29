/**
 *  @file   zmq_transport.hpp
 *  @date   2020-09-21
 *  @author Sungkyu Kang
 *  @brief
 *        Bridge Client class for Cloud Bridge
 *  @remark
 *  @copyright
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 *
 *         SPDX-License-Identifier: MIT
 */
#ifndef _CLOUD_BRIDGE_ZMQ_TRANSPORT_H_
#define _CLOUD_BRIDGE_ZMQ_TRANSPORT_H_

#include <map>
#include <cstdint>
#include <vector>
#include <string>
#include <zmq.h>
#include <mutex>
#include <thread>
#include <unordered_set>

class BridgeRclNode;

class ZmqTransport
{
public:
    ZmqTransport(BridgeRclNode& node, 
        void* m_pPubSocket, void* m_pSubSocket,
        void* m_pReqSocket, void* m_pRepSocket);
    ~ZmqTransport();

    void start();
    void stop();

    void send_publish(const std::string& topic, const std::string &type, 
        const std::vector<uint8_t>& msg);
    void send_request_and_get_response(const std::string& topic,
        const std::vector<uint8_t>& req_msg, std::vector<uint8_t>& res_msg);
    void set_qos_map(std::map<std::string, std::string> map);

private:
    struct TopicData
    {
        std::string topic;
        std::string type;
    };
    std::unordered_set<TopicData*> topicDatas;

    BridgeRclNode& rcl_node_;
    void* m_pPubSocket;
    void* m_pSubSocket;
    void* m_pReqSocket;
    void* m_pRepSocket;
    zmq_msg_t m_pMsgSub;
    zmq_msg_t m_pMsgReq;
    zmq_msg_t m_pMsgRep;

    std::thread m_threadProc;
    std::thread m_threadServiceProc;
    std::mutex m_mutexSend;
    std::mutex m_mutexReq;
    std::mutex m_mutexRep;

    uint8_t temp[1024*1024];
    std::vector<uint8_t> buffer;
    std::vector<uint8_t> rep_buffer;
    std::map<std::string, std::string> qos_map;

    void read_zmq_sub();
    bool receive_zmq(void** buffer, int& bufferLength);
    bool send_zmq(const void* buffer, const int bufferLength);

    void handle_publish();
    void handle_add_subscriber();
    void handle_add_publisher();
    
    void read_zmq_service();
    bool receive_zmq_service(void** buffer, int& bufferLength);
    bool send_zmq_request(const void* buffer, const int bufferLength, 
        void** recvBuffer, int& recvBufferLength);
    bool send_zmq_reply(const void* buffer, const int bufferLength);

    void handle_service();

    void check_topic_data(std::string topic, std::string type);

    uint32_t get32le(size_t offset) const;
    uint32_t get32le_rep(size_t offset) const;
};

#endif // _CLOUD_BRIDGE_ZMQ_TRANSPORT_H_