#include "UDPTransport.hpp"
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

UDPTransport::UDPTransport(int port, const std::string& address)
    : port_(port), address_(address), remotePort_(14550) {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

UDPTransport::~UDPTransport() {
    stop();
#ifdef _WIN32
    WSACleanup();
#endif
}

void UDPTransport::setRemote(const std::string& host, int port) {
    remoteHost_ = host;
    remotePort_ = port;
}

bool UDPTransport::start() {
    if (running_) return false;
    
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) {
        notifyError("Failed to create UDP socket");
        return false;
    }
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = inet_addr(address_.c_str());
    
    if (bind(socket_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        notifyError("Failed to bind UDP socket");
#ifdef _WIN32
        closesocket(socket_);
#else
        close(socket_);
#endif
        return false;
    }
    
    running_ = true;
    receiveThread_ = std::thread(&UDPTransport::receiveLoop, this);
    return true;
}

void UDPTransport::stop() {
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

bool UDPTransport::send(const uint8_t* data, size_t length) {
    if (!running_ || socket_ < 0 || remoteHost_.empty()) return false;
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(remotePort_);
    addr.sin_addr.s_addr = inet_addr(remoteHost_.c_str());
    
    int sent = sendto(socket_, (const char*)data, static_cast<int>(length), 0, 
                          (sockaddr*)&addr, sizeof(addr));
    return sent == static_cast<int>(length);
}

void UDPTransport::receiveLoop() {
    uint8_t buffer[2048];
    
    while (running_) {
        sockaddr_in from{};
        socklen_t fromLen = sizeof(from);
        
        int received = recvfrom(socket_, (char*)buffer, sizeof(buffer), 0,
                                    (sockaddr*)&from, &fromLen);
        
        if (received > 0) {
            std::vector<uint8_t> data(buffer, buffer + static_cast<size_t>(received));
            notifyData(data);
        }
    }
}

} // namespace margelo::nitro::mavlink
