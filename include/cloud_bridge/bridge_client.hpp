/**
 *  @file   bridge_client.hpp
 *  @date   2020-09-21
 *  @author Sungkyu Kang
 *  @brief
 *        Bridge Client class for Cloud Bridge
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#ifndef _CLOUD_BRIDGE_CLIENT_H_
#define _CLOUD_BRIDGE_CLIENT_H_

#include <cstdint>
#include <vector>
#include <string>
#include <zmq.h>
#include <mutex>
#include <thread>
#include <unordered_set>

class BridgeNode;

class BridgeClient
{
public:
    BridgeClient(BridgeNode& node, void* m_pPubSocket, void* m_pSubSocket);
    ~BridgeClient();

    void start();
    void stop();

    void publish(const std::string& topic, const std::string &type, const std::vector<uint8_t>& msg);

private:
    struct TopicData
    {
        std::string topic;
        std::string type;
    };
    std::unordered_set<TopicData*> topicDatas;

    BridgeNode& node;
    void* m_pPubSocket;
    void* m_pSubSocket;
    zmq_msg_t m_pMsgSub;

    std::thread m_threadProc;
    std::mutex m_mutexSend;

    uint8_t temp[1024*1024];
    std::vector<uint8_t> buffer;

    void handle_read();
    bool receive_zmq(void** buffer, int& bufferLength);
    bool send_zmq(const void* buffer, const int bufferLength);

    void handle_add_subscriber();
    void handle_add_publisher();
    void handle_publish();

    void check_topic_data(std::string topic, std::string type);

    uint32_t get32le(size_t offset) const;
};

#endif // _CLOUD_BRIDGE_CLIENT_H_