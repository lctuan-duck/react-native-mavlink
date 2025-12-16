/**
 * ConnectionManager.cpp
 * Implementation - UDP, TCP, and Serial support
 */

#include "ConnectionManager.hpp"
#include "../MAVLink/v2.0/common/mavlink.h"
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <termios.h>
    #define SOCKET_ERROR -1
    #define INVALID_SOCKET -1
    #define closesocket close
#endif

namespace margelo::nitro::mavlink {

ConnectionManager::ConnectionManager() {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

ConnectionManager::~ConnectionManager() {
    disconnect();
#ifdef _WIN32
    WSACleanup();
#endif
}

// ============================================================================
// Connection Management
// ============================================================================

bool ConnectionManager::connect(ConnectionType type, const std::string& address, int port, int baudRate) {
    if (_connected) {
        disconnect();
    }
    
    _connectionType = type;
    _address = address;
    _port = port;
    _baudRate = baudRate;
    
    bool success = false;
    
    switch (type) {
        case ConnectionType::UDP:
            success = connectUDP();
            break;
        case ConnectionType::TCP:
            success = connectTCP();
            break;
        case ConnectionType::SERIAL:
            success = connectSerial();
            break;
    }
    
    if (success) {
        _connected = true;
        _shouldRun = true;
        
        // Start threads
        _receiveThread = std::thread(&ConnectionManager::receiveThreadFunction, this);
        _sendThread = std::thread(&ConnectionManager::sendThreadFunction, this);
        
        notifyConnection(true);
    }
    
    return success;
}

void ConnectionManager::disconnect() {
    if (_connected) {
        _shouldRun = false;
        _connected = false;
        
        // Wait for threads
        if (_receiveThread.joinable()) {
            _receiveThread.join();
        }
        if (_sendThread.joinable()) {
            _sendThread.join();
        }
        
        disconnectSocket();
        notifyConnection(false);
    }
}

bool ConnectionManager::isConnected() const {
    return _connected.load();
}

// ============================================================================
// Message Handling
// ============================================================================

void ConnectionManager::setMessageCallback(MessageCallback callback) {
    std::lock_guard<std::mutex> lock(_callbackMutex);
    _messageCallback = callback;
}

void ConnectionManager::setConnectionCallback(ConnectionCallback callback) {
    std::lock_guard<std::mutex> lock(_callbackMutex);
    _connectionCallback = callback;
}

bool ConnectionManager::sendMessage(const mavlink_message_t& message) {
    if (!_connected) {
        return false;
    }
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
    
    std::vector<uint8_t> data(buffer, buffer + length);
    
    {
        std::lock_guard<std::mutex> lock(_sendQueueMutex);
        _sendQueue.push(data);
    }
    
    return true;
}

bool ConnectionManager::sendMessageWithRetry(const mavlink_message_t& message, int retries, int retryDelayMs) {
    for (int i = 0; i < retries; i++) {
        if (sendMessage(message)) {
            if (i > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(retryDelayMs));
            }
            return true;
        }
    }
    return false;
}

// ============================================================================
// System Configuration
// ============================================================================

void ConnectionManager::setSystemId(uint8_t systemId) {
    _systemId = systemId;
}

void ConnectionManager::setComponentId(uint8_t componentId) {
    _componentId = componentId;
}

uint8_t ConnectionManager::getSystemId() const {
    return _systemId.load();
}

uint8_t ConnectionManager::getComponentId() const {
    return _componentId.load();
}

// ============================================================================
// Statistics
// ============================================================================

uint64_t ConnectionManager::getMessagesReceived() const {
    return _messagesReceived.load();
}

uint64_t ConnectionManager::getMessagesSent() const {
    return _messagesSent.load();
}

uint64_t ConnectionManager::getMessagesLost() const {
    return _messagesLost.load();
}

// ============================================================================
// Private Methods - Connection
// ============================================================================

bool ConnectionManager::connectUDP() {
    _socketFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_socketFd == INVALID_SOCKET) {
        return false;
    }
    
    // Set non-blocking
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(_socketFd, FIONBIO, &mode);
#else
    int flags = fcntl(_socketFd, F_GETFL, 0);
    fcntl(_socketFd, F_SETFL, flags | O_NONBLOCK);
#endif
    
    // Bind to receive port
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(_port);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(_socketFd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        closesocket(_socketFd);
        _socketFd = -1;
        return false;
    }
    
    return true;
}

bool ConnectionManager::connectTCP() {
    // Create TCP socket
    _socketFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (_socketFd == INVALID_SOCKET) {
        std::cerr << "Failed to create TCP socket" << std::endl;
        return false;
    }
    
    // Set socket options
    int opt = 1;
    setsockopt(_socketFd, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
    
    // Set connect timeout (5 seconds)
#ifdef _WIN32
    DWORD timeout = 5000;
    setsockopt(_socketFd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    setsockopt(_socketFd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));
#else
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(_socketFd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    setsockopt(_socketFd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));
#endif
    
    // Setup server address
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(_port);
    
    // Convert address
#ifdef _WIN32
    InetPtonA(AF_INET, _address.c_str(), &serverAddr.sin_addr);
#else
    inet_pton(AF_INET, _address.c_str(), &serverAddr.sin_addr);
#endif
    
    // Connect to server (use :: to disambiguate from method name)
    std::cout << "Connecting to TCP " << _address << ":" << _port << std::endl;
    if (::connect(_socketFd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        int error = 
#ifdef _WIN32
            WSAGetLastError();
#else
            errno;
#endif
        std::cerr << "TCP connect failed with error: " << error << std::endl;
        closesocket(_socketFd);
        _socketFd = -1;
        return false;
    }
    
    std::cout << "TCP connected successfully" << std::endl;
    
    // Set non-blocking after connection established
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(_socketFd, FIONBIO, &mode);
#else
    int flags = fcntl(_socketFd, F_GETFL, 0);
    fcntl(_socketFd, F_SETFL, flags | O_NONBLOCK);
#endif
    
    // Store remote address for sending
    _remoteAddr = serverAddr;
    
    return true;
}

bool ConnectionManager::connectSerial() {
#ifdef _WIN32
    // Windows: Use CreateFile API
    std::string portName = "\\\\.\\" + _address; // e.g., "\\\\.\\COM3"
    
    HANDLE hSerial = CreateFileA(
        portName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,    // No sharing
        NULL, // No security
        OPEN_EXISTING,
        0,    // Not overlapped
        NULL
    );
    
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to open serial port " << _address 
                  << " Error: " << GetLastError() << std::endl;
        return false;
    }
    
    // Configure DCB (Device Control Block)
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Failed to get COM state" << std::endl;
        CloseHandle(hSerial);
        return false;
    }
    
    // Set baud rate and parameters
    dcbSerialParams.BaudRate = _baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    
    if (!SetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Failed to set COM state" << std::endl;
        CloseHandle(hSerial);
        return false;
    }
    
    // Set timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    
    if (!SetCommTimeouts(hSerial, &timeouts)) {
        std::cerr << "Failed to set COM timeouts" << std::endl;
        CloseHandle(hSerial);
        return false;
    }
    
    _socketFd = (int)(intptr_t)hSerial;
    std::cout << "Serial port " << _address << " opened at " << _baudRate << " baud" << std::endl;
    return true;
    
#else
    // Linux/macOS: Use termios
    _socketFd = open(_address.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if (_socketFd == -1) {
        std::cerr << "Failed to open serial port " << _address 
                  << " Error: " << strerror(errno) << std::endl;
        return false;
    }
    
    // Configure termios
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(_socketFd, &tty) != 0) {
        std::cerr << "Failed to get termios attributes" << std::endl;
        close(_socketFd);
        _socketFd = -1;
        return false;
    }
    
    // Set baud rate
    speed_t baudSpeed = B57600; // Default
    switch (_baudRate) {
        case 9600: baudSpeed = B9600; break;
        case 19200: baudSpeed = B19200; break;
        case 38400: baudSpeed = B38400; break;
        case 57600: baudSpeed = B57600; break;
        case 115200: baudSpeed = B115200; break;
        case 230400: baudSpeed = B230400; break;
        case 460800: baudSpeed = B460800; break;
        case 921600: baudSpeed = B921600; break;
        default:
            std::cerr << "Unsupported baud rate: " << _baudRate << std::endl;
            break;
    }
    
    cfsetospeed(&tty, baudSpeed);
    cfsetispeed(&tty, baudSpeed);
    
    // 8N1 mode
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 data bits
    
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines
    
    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST; // Raw output
    
    // Non-blocking reads
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 0.1 second timeout
    
    if (tcsetattr(_socketFd, TCSANOW, &tty) != 0) {
        std::cerr << "Failed to set termios attributes" << std::endl;
        close(_socketFd);
        _socketFd = -1;
        return false;
    }
    
    std::cout << "Serial port " << _address << " opened at " << _baudRate << " baud" << std::endl;
    return true;
#endif
}

void ConnectionManager::disconnectSocket() {
    if (_socketFd != -1) {
        if (_connectionType == ConnectionType::SERIAL) {
#ifdef _WIN32
            HANDLE hSerial = (HANDLE)(intptr_t)_socketFd;
            CloseHandle(hSerial);
#else
            close(_socketFd);
#endif
        } else {
            // UDP/TCP socket
            closesocket(_socketFd);
        }
        _socketFd = -1;
    }
}

// ============================================================================
// Private Methods - Threads
// ============================================================================

void ConnectionManager::receiveThreadFunction() {
    uint8_t buffer[2048];
    
    while (_shouldRun) {
        if (_socketFd == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        int bytesReceived = 0;
        
        if (_connectionType == ConnectionType::UDP) {
            // UDP: use recvfrom to get sender address
            struct sockaddr_in senderAddr;
            socklen_t senderAddrLen = sizeof(senderAddr);
            
            bytesReceived = recvfrom(_socketFd, (char*)buffer, sizeof(buffer), 0,
                                     (struct sockaddr*)&senderAddr, &senderAddrLen);
            
            // Store remote address for UDP replies
            if (bytesReceived > 0) {
                _remoteAddr = senderAddr;
            }
        } 
        else if (_connectionType == ConnectionType::TCP) {
            // TCP: use recv (connection-oriented)
            bytesReceived = recv(_socketFd, (char*)buffer, sizeof(buffer), 0);
            
            // Check for connection closed
            if (bytesReceived == 0) {
                std::cerr << "TCP connection closed by remote" << std::endl;
                notifyConnection(false);
                break;
            }
        }
        else if (_connectionType == ConnectionType::SERIAL) {
            // Serial: platform-specific read
#ifdef _WIN32
            HANDLE hSerial = (HANDLE)(intptr_t)_socketFd;
            DWORD dwBytesRead = 0;
            
            if (ReadFile(hSerial, buffer, sizeof(buffer), &dwBytesRead, NULL)) {
                bytesReceived = (int)dwBytesRead;
            } else {
                DWORD error = GetLastError();
                if (error != ERROR_IO_PENDING && error != ERROR_SUCCESS) {
                    std::cerr << "Serial read error: " << error << std::endl;
                }
                bytesReceived = -1;
            }
#else
            bytesReceived = read(_socketFd, buffer, sizeof(buffer));
#endif
        }
        
        if (bytesReceived > 0) {
            parseReceivedData(buffer, bytesReceived);
        } else if (bytesReceived < 0) {
            // Error occurred
#ifdef _WIN32
            int error = WSAGetLastError();
            if (error != WSAEWOULDBLOCK) {
                // Don't spam logs for non-blocking would-block
            }
#else
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                // Don't spam logs for non-blocking would-block
            }
#endif
            // Sleep briefly on error
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            // No data, sleep briefly
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void ConnectionManager::sendThreadFunction() {
    while (_shouldRun) {
        std::vector<uint8_t> data;
        
        {
            std::lock_guard<std::mutex> lock(_sendQueueMutex);
            if (!_sendQueue.empty()) {
                data = _sendQueue.front();
                _sendQueue.pop();
            }
        }
        
        if (!data.empty()) {
            if (sendData(data.data(), data.size())) {
                _messagesSent++;
            } else {
                _messagesLost++;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void ConnectionManager::parseReceivedData(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &_mavlinkMessage, &_mavlinkStatus)) {
            // Message received
            _messagesReceived++;
            
            // Store target system ID from first message
            if (_targetSystemId == 0) {
                _targetSystemId = _mavlinkMessage.sysid;
            }
            
            // Callback
            {
                std::lock_guard<std::mutex> lock(_callbackMutex);
                if (_messageCallback) {
                    _messageCallback(_mavlinkMessage);
                }
            }
        }
    }
}

// ============================================================================
// Private Methods - Helpers
// ============================================================================

void ConnectionManager::notifyConnection(bool connected) {
    std::lock_guard<std::mutex> lock(_callbackMutex);
    if (_connectionCallback) {
        _connectionCallback(connected);
    }
}

bool ConnectionManager::sendData(const uint8_t* data, size_t length) {
    if (_socketFd == -1 || !_connected) {
        return false;
    }
    
    if (_connectionType == ConnectionType::UDP) {
        struct sockaddr_in destAddr;
        memset(&destAddr, 0, sizeof(destAddr));
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(_port);
        
        if (_address.empty() || _address == "0.0.0.0") {
            // Broadcast or use last received address
            destAddr.sin_addr.s_addr = INADDR_BROADCAST;
        } else {
            inet_pton(AF_INET, _address.c_str(), &destAddr.sin_addr);
        }
        
        int bytesSent = sendto(_socketFd, (const char*)data, length, 0,
                               (struct sockaddr*)&destAddr, sizeof(destAddr));
        
        return bytesSent == (int)length;
    } 
    else if (_connectionType == ConnectionType::TCP) {
        // TCP: use send (connection-oriented)
        int bytesSent = send(_socketFd, (const char*)data, length, 0);
        
        if (bytesSent < 0) {
#ifdef _WIN32
            int error = WSAGetLastError();
            if (error != WSAEWOULDBLOCK) {
                std::cerr << "TCP send error: " << error << std::endl;
                notifyConnection(false);
            }
#else
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "TCP send error: " << errno << std::endl;
                notifyConnection(false);
            }
#endif
            return false;
        }
        
        return bytesSent == (int)length;
    }
    else if (_connectionType == ConnectionType::SERIAL) {
        // Serial: platform-specific write
#ifdef _WIN32
        HANDLE hSerial = (HANDLE)(intptr_t)_socketFd;
        DWORD dwBytesWritten = 0;
        
        if (WriteFile(hSerial, data, length, &dwBytesWritten, NULL)) {
            return dwBytesWritten == (DWORD)length;
        } else {
            std::cerr << "Serial write error: " << GetLastError() << std::endl;
            return false;
        }
#else
        int bytesSent = write(_socketFd, data, length);
        
        if (bytesSent < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "Serial write error: " << errno << std::endl;
            }
            return false;
        }
        
        return bytesSent == (int)length;
#endif
    }
    
    return false;
}

} // namespace margelo::nitro::mavlink
