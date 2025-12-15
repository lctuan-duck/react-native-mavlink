#include "TCPTransport.hpp"
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

namespace margelo::nitro::mavlink {

TCPTransport::TCPTransport(const std::string& host, int port)
    : host_(host), port_(port) {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

TCPTransport::~TCPTransport() {
    stop();
#ifdef _WIN32
    WSACleanup();
#endif
}

bool TCPTransport::start() {
    if (running_) return false;
    
    socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_ < 0) {
        notifyError("Failed to create TCP socket");
        return false;
    }
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = inet_addr(host_.c_str());
    
    if (connect(socket_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        notifyError("Failed to connect TCP socket");
#ifdef _WIN32
        closesocket(socket_);
#else
        close(socket_);
#endif
        return false;
    }
    
    running_ = true;
    receiveThread_ = std::thread(&TCPTransport::receiveLoop, this);
    return true;
}

void TCPTransport::stop() {
    if (!running_) return;
    
    running_ = false;
    if (receiveThread_.joinable()) {
        receiveThread_.join();
    }
    
    if (socket_ >= 0) {
#ifdef _WIN32
        closesocket(socket_);
#else
        close(socket_);
#endif
        socket_ = -1;
    }
}

bool TCPTransport::send(const uint8_t* data, size_t length) {
    if (!running_ || socket_ < 0) return false;
    
    int sent = ::send(socket_, (const char*)data, static_cast<int>(length), 0);
    return sent == static_cast<int>(length);
}

void TCPTransport::receiveLoop() {
    uint8_t buffer[2048];
    
    while (running_) {
        int received = recv(socket_, (char*)buffer, sizeof(buffer), 0);
        
        if (received > 0) {
            std::vector<uint8_t> data(buffer, buffer + static_cast<size_t>(received));
            notifyData(data);
        } else if (received == 0) {
            notifyError("TCP connection closed");
            break;
        }
    }
}

} // namespace margelo::nitro::mavlink
