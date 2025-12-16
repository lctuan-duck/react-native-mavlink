/**
 * ConnectionManager.hpp
 * Manages MAVLink connections (UDP, TCP, Serial)
 * Based on QGroundControl LinkInterface
 */

#pragma once

#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>

#ifdef _WIN32
    #include <winsock2.h>
#else
    #include <netinet/in.h>
#endif

#include "../mavlink/v2.0/common/mavlink.h"

namespace margelo::nitro::mavlink {

enum class ConnectionType {
    SERIAL = 0,
    UDP = 1,
    TCP = 2
};

using MessageCallback = std::function<void(const mavlink_message_t&)>;
using ConnectionCallback = std::function<void(bool connected)>;

class ConnectionManager {
public:
    ConnectionManager();
    ~ConnectionManager();

    // ============================================================================
    // Connection Management
    // ============================================================================
    
    bool connect(ConnectionType type, const std::string& address, int port, int baudRate);
    void disconnect();
    bool isConnected() const;
    
    // ============================================================================
    // Message Handling
    // ============================================================================
    
    void setMessageCallback(MessageCallback callback);
    void setConnectionCallback(ConnectionCallback callback);
    
    // Send MAVLink message
    bool sendMessage(const mavlink_message_t& message);
    
    // Send message with retries (for commands)
    bool sendMessageWithRetry(const mavlink_message_t& message, int retries = 3, int retryDelayMs = 100);
    
    // ============================================================================
    // System Configuration
    // ============================================================================
    
    void setSystemId(uint8_t systemId);
    void setComponentId(uint8_t componentId);
    uint8_t getSystemId() const;
    uint8_t getComponentId() const;
    
    // ============================================================================
    // Statistics
    // ============================================================================
    
    uint64_t getMessagesReceived() const;
    uint64_t getMessagesSent() const;
    uint64_t getMessagesLost() const;
    
private:
    // Connection state
    std::atomic<bool> _connected{false};
    std::atomic<bool> _shouldRun{false};
    ConnectionType _connectionType;
    std::string _address;
    int _port;
    int _baudRate;
    
    // MAVLink system info
    std::atomic<uint8_t> _systemId{255}; // GCS system ID
    std::atomic<uint8_t> _componentId{MAV_COMP_ID_MISSIONPLANNER};
    uint8_t _targetSystemId{0}; // Vehicle system ID
    
    // Threads
    std::thread _receiveThread;
    std::thread _sendThread;
    
    // Callbacks
    MessageCallback _messageCallback;
    ConnectionCallback _connectionCallback;
    std::mutex _callbackMutex;
    
    // Message queue
    std::queue<std::vector<uint8_t>> _sendQueue;
    std::mutex _sendQueueMutex;
    
    // Statistics
    std::atomic<uint64_t> _messagesReceived{0};
    std::atomic<uint64_t> _messagesSent{0};
    std::atomic<uint64_t> _messagesLost{0};
    
    // Socket/Connection handle
    int _socketFd{-1};
    struct sockaddr_in _remoteAddr; // Store remote address for UDP
    
    // ============================================================================
    // Private Methods
    // ============================================================================
    
    // Connection type specific
    bool connectUDP();
    bool connectTCP();
    bool connectSerial();
    
    void disconnectSocket();
    
    // Thread methods
    void receiveThreadFunction();
    void sendThreadFunction();
    
    // Message parsing
    void parseReceivedData(const uint8_t* data, size_t length);
    
    // Helpers
    void notifyConnection(bool connected);
    bool sendData(const uint8_t* data, size_t length);
    
    // MAVLink state
    mavlink_status_t _mavlinkStatus{};
    mavlink_message_t _mavlinkMessage{};
};

} // namespace margelo::nitro::mavlink
