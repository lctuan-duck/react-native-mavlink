#pragma once
#include "Transport.hpp"
#include <thread>
#include <atomic>
#include <string>

namespace margelo::nitro::mavlink {

class UDPTransport : public Transport {
public:
    explicit UDPTransport(int port, const std::string& address = "0.0.0.0");
    ~UDPTransport() override;
    
    bool start() override;
    void stop() override;
    bool isRunning() const override { return running_; }
    bool send(const uint8_t* data, size_t length) override;
    
    void setRemote(const std::string& host, int port);
    
private:
    void receiveLoop();
    
    int port_;
    std::string address_;
    std::string remoteHost_;
    int remotePort_;
    int socket_{-1};
    std::thread receiveThread_;
    std::atomic<bool> running_{false};
};

} // namespace margelo::nitro::mavlink
