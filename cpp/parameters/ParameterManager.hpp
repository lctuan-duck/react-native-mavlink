/**
 * ParameterManager.hpp
 * Manages MAVLink parameter operations with cache
 */

#pragma once

#include <map>
#include <string>
#include <mutex>
#include <memory>
#include <functional>
#include <chrono>
#include "../mavlink/v2.0/common/mavlink.h"

namespace margelo::nitro::mavlink {

// Forward declarations
class ConnectionManager;

/**
 * Parameter value with type information
 */
struct ParameterValue {
    float value;
    uint8_t type; // MAV_PARAM_TYPE
    uint16_t index;
    uint16_t count;
    std::chrono::steady_clock::time_point lastUpdate;
    
    ParameterValue() 
        : value(0.0f)
        , type(MAV_PARAM_TYPE_REAL32)
        , index(0)
        , count(0)
        , lastUpdate(std::chrono::steady_clock::now()) {}
};

/**
 * Pending parameter request
 */
struct ParameterRequest {
    std::string name;
    std::function<void(bool success, float value)> callback;
    std::chrono::steady_clock::time_point sentTime;
    int retriesLeft;
    
    ParameterRequest()
        : sentTime(std::chrono::steady_clock::now())
        , retriesLeft(0) {}
};

/**
 * ParameterManager class
 * Handles parameter get/set with caching and retry logic
 */
class ParameterManager {
public:
    explicit ParameterManager(std::shared_ptr<ConnectionManager> connectionManager,
                             uint8_t targetSystem, uint8_t targetComponent);
    ~ParameterManager() = default;

    // Parameter operations
    void getParameter(const std::string& name, 
                     std::function<void(bool success, float value)> callback,
                     int retries = 3);
    
    void setParameter(const std::string& name, 
                     float value,
                     uint8_t type,
                     std::function<void(bool success)> callback,
                     int retries = 3);
    
    void requestAllParameters();
    
    // Message handling
    void handleParamValue(const mavlink_param_value_t& paramValue);
    
    // Cache management
    bool hasParameter(const std::string& name) const;
    float getCachedValue(const std::string& name) const;
    void clearCache();
    
    // Timeout checking
    void checkTimeouts();
    
    // Target management
    void setTarget(uint8_t system, uint8_t component);
    
    // Statistics
    int getCachedParameterCount() const { return _parameterCache.size(); }
    int getTotalParameterCount() const { return _totalParameterCount; }
    
private:
    // Send MAVLink messages
    void sendParamRequestRead(const std::string& name);
    void sendParamRequestList();
    void sendParamSet(const std::string& name, float value, uint8_t type);
    
    // Type conversion helpers
    static float paramUnionToFloat(const mavlink_param_union_t& paramUnion, uint8_t type);
    static void floatToParamUnion(float value, uint8_t type, mavlink_param_union_t& paramUnion);
    
    // Thread safety
    mutable std::mutex _cacheMutex;
    mutable std::mutex _requestsMutex;
    
    // Dependencies
    std::shared_ptr<ConnectionManager> _connectionManager;
    uint8_t _targetSystem;
    uint8_t _targetComponent;
    
    // Parameter cache: name -> value
    std::map<std::string, ParameterValue> _parameterCache;
    
    // Pending requests: name -> request
    std::map<std::string, ParameterRequest> _pendingGetRequests;
    std::map<std::string, ParameterRequest> _pendingSetRequests;
    
    // Stats
    int _totalParameterCount = 0;
    
    // Timeouts
    static constexpr int PARAM_REQUEST_TIMEOUT_MS = 1000;
    static constexpr int PARAM_SET_TIMEOUT_MS = 1000;
};

} // namespace margelo::nitro::mavlink
