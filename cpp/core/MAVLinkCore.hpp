#pragma once
#include <memory>
#include <string>
#include <functional>
#include <cstdint>
#include <vector>
#include <any>
#include "MAVLinkState.hpp"

namespace margelo::nitro::mavlink {

// Forward declarations
class Transport;
class MessageParser;
class TelemetryManager;

// Options structs (must match TypeScript spec)
struct UdpOptions {
    int port;              // Local bind port
    std::string host;      // Local bind address (default '0.0.0.0')
    std::string remoteHost; // Remote drone address to send to
    int remotePort;        // Remote drone port to send to
    int heartbeatTimeoutMs;
};

struct TcpOptions {
    std::string host;
    int port;
    int heartbeatTimeoutMs;
};

class MAVLinkCore {
public:
    explicit MAVLinkCore();
    ~MAVLinkCore();
    
    // Prevent copying
    MAVLinkCore(const MAVLinkCore&) = delete;
    MAVLinkCore& operator=(const MAVLinkCore&) = delete;
    
    // Transport management
    bool startUDP(const UdpOptions& options);
    void stopUDP();
    bool startTCP(const TcpOptions& options);
    void stopTCP();
    
    // Send raw data
    bool sendData(const uint8_t* data, size_t length);
    
    // Telemetry access
    TelemetryState getTelemetrySnapshot() const;
    
    // Event callbacks
    using RawDataCallback = std::function<void(const std::vector<uint8_t>&)>;
    using EventCallback = std::function<void(const std::string& eventType, const std::string& data)>;
    using AnyEventCallback = std::function<void(const std::string& eventType, const std::any& data)>;
    using ErrorCallback = std::function<void(const std::string& error)>;
    
    void setRawDataCallback(RawDataCallback callback);
    void setEventCallback(EventCallback callback);
    void setErrorCallback(ErrorCallback callback);
    
    // Register event listeners with std::any data (for typed events)
    void onEvent(const std::string& eventName, AnyEventCallback callback);
    
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    
    void handleIncomingData(const std::vector<uint8_t>& data);
};

} // namespace margelo::nitro::mavlink
