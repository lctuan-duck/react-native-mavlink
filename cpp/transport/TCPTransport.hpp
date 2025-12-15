#pragma once
#include "Transport.hpp"
#include <thread>
#include <atomic>
#include <string>

namespace margelo::nitro::mavlink {

class TCPTransport : public Transport {
public:
    explicit TCPTransport(const std::string& host, int port);
    ~TCPTransport() override;
    
    bool start() override;
    void stop() override;
    bool isRunning() const override { return running_; }
    bool send(const uint8_t* data, size_t length) override;
    
private:
    void receiveLoop();
    
    std::string host_;
    int port_;
    int socket_{-1};
    std::thread receiveThread_;
    std::atomic<bool> running_{false};
};

} // namespace margelo::nitro::mavlink
