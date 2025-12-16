/**
 * ParameterManager.cpp
 * Implementation of parameter management with caching
 */

#include "ParameterManager.hpp"
#include "../connection/ConnectionManager.hpp"
#include <cstring>
#include <iostream>
#include <cmath>

namespace margelo::nitro::mavlink {

ParameterManager::ParameterManager(std::shared_ptr<ConnectionManager> connectionManager,
                                 uint8_t targetSystem, uint8_t targetComponent)
    : _connectionManager(connectionManager)
    , _targetSystem(targetSystem)
    , _targetComponent(targetComponent) {
}

// ============================================================================
// Parameter Operations
// ============================================================================

void ParameterManager::getParameter(const std::string& name, 
                                    std::function<void(bool success, float value)> callback,
                                    int retries) {
    // Check cache first
    {
        std::lock_guard<std::mutex> lock(_cacheMutex);
        if (_parameterCache.find(name) != _parameterCache.end()) {
            const auto& param = _parameterCache[name];
            if (callback) {
                callback(true, param.value);
            }
            return;
        }
    }
    
    // Not in cache, request from vehicle
    ParameterRequest request;
    request.name = name;
    request.callback = callback;
    request.sentTime = std::chrono::steady_clock::now();
    request.retriesLeft = retries;
    
    {
        std::lock_guard<std::mutex> lock(_requestsMutex);
        _pendingGetRequests[name] = request;
    }
    
    sendParamRequestRead(name);
}

void ParameterManager::setParameter(const std::string& name, 
                                    float value,
                                    uint8_t type,
                                    std::function<void(bool success)> callback,
                                    int retries) {
    ParameterRequest request;
    request.name = name;
    request.callback = [callback](bool success, float) {
        if (callback) callback(success);
    };
    request.sentTime = std::chrono::steady_clock::now();
    request.retriesLeft = retries;
    
    {
        std::lock_guard<std::mutex> lock(_requestsMutex);
        _pendingSetRequests[name] = request;
    }
    
    sendParamSet(name, value, type);
}

void ParameterManager::requestAllParameters() {
    sendParamRequestList();
}

// ============================================================================
// Message Handling
// ============================================================================

void ParameterManager::handleParamValue(const mavlink_param_value_t& paramValue) {
    // Extract parameter name (ensure null termination)
    char paramName[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1] = {0};
    std::memcpy(paramName, paramValue.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    std::string name(paramName);
    
    // Convert param_value (float wire format) to actual float value based on type
    float value = paramUnionToFloat(paramValue.param_value, paramValue.param_type);
    
    // Update cache
    {
        std::lock_guard<std::mutex> lock(_cacheMutex);
        
        ParameterValue& param = _parameterCache[name];
        param.value = value;
        param.type = paramValue.param_type;
        param.index = paramValue.param_index;
        param.count = paramValue.param_count;
        param.lastUpdate = std::chrono::steady_clock::now();
        
        // Update total count
        if (paramValue.param_count > 0 && _totalParameterCount != paramValue.param_count) {
            _totalParameterCount = paramValue.param_count;
        }
    }
    
    // Complete pending get request
    {
        std::lock_guard<std::mutex> lock(_requestsMutex);
        auto it = _pendingGetRequests.find(name);
        if (it != _pendingGetRequests.end()) {
            if (it->second.callback) {
                it->second.callback(true, value);
            }
            _pendingGetRequests.erase(it);
        }
        
        // Complete pending set request
        auto setIt = _pendingSetRequests.find(name);
        if (setIt != _pendingSetRequests.end()) {
            if (setIt->second.callback) {
                setIt->second.callback(true, 0.0f);
            }
            _pendingSetRequests.erase(setIt);
        }
    }
}

// ============================================================================
// Cache Management
// ============================================================================

bool ParameterManager::hasParameter(const std::string& name) const {
    std::lock_guard<std::mutex> lock(_cacheMutex);
    return _parameterCache.find(name) != _parameterCache.end();
}

float ParameterManager::getCachedValue(const std::string& name) const {
    std::lock_guard<std::mutex> lock(_cacheMutex);
    auto it = _parameterCache.find(name);
    if (it != _parameterCache.end()) {
        return it->second.value;
    }
    return 0.0f;
}

void ParameterManager::clearCache() {
    std::lock_guard<std::mutex> lock(_cacheMutex);
    _parameterCache.clear();
    _totalParameterCount = 0;
}

// ============================================================================
// Timeout Checking
// ============================================================================

void ParameterManager::checkTimeouts() {
    auto now = std::chrono::steady_clock::now();
    
    std::lock_guard<std::mutex> lock(_requestsMutex);
    
    // Check get requests
    std::vector<std::string> toRetryGet;
    std::vector<std::string> toTimeoutGet;
    
    for (auto& pair : _pendingGetRequests) {
        auto& request = pair.second;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - request.sentTime).count();
        
        if (elapsed > PARAM_REQUEST_TIMEOUT_MS) {
            if (request.retriesLeft > 0) {
                toRetryGet.push_back(pair.first);
            } else {
                toTimeoutGet.push_back(pair.first);
            }
        }
    }
    
    // Retry get requests
    for (const auto& name : toRetryGet) {
        auto& request = _pendingGetRequests[name];
        request.retriesLeft--;
        request.sentTime = now;
        sendParamRequestRead(name);
    }
    
    // Timeout get requests
    for (const auto& name : toTimeoutGet) {
        auto& request = _pendingGetRequests[name];
        if (request.callback) {
            request.callback(false, 0.0f);
        }
        _pendingGetRequests.erase(name);
    }
    
    // Check set requests
    std::vector<std::string> toRetrySet;
    std::vector<std::string> toTimeoutSet;
    
    for (auto& pair : _pendingSetRequests) {
        auto& request = pair.second;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - request.sentTime).count();
        
        if (elapsed > PARAM_SET_TIMEOUT_MS) {
            if (request.retriesLeft > 0) {
                toRetrySet.push_back(pair.first);
            } else {
                toTimeoutSet.push_back(pair.first);
            }
        }
    }
    
    // Retry set requests (need to get value from cache)
    for (const auto& name : toRetrySet) {
        auto& request = _pendingSetRequests[name];
        request.retriesLeft--;
        request.sentTime = now;
        
        // Get cached value to retry
        std::lock_guard<std::mutex> cacheLock(_cacheMutex);
        if (_parameterCache.find(name) != _parameterCache.end()) {
            const auto& param = _parameterCache[name];
            sendParamSet(name, param.value, param.type);
        }
    }
    
    // Timeout set requests
    for (const auto& name : toTimeoutSet) {
        auto& request = _pendingSetRequests[name];
        if (request.callback) {
            request.callback(false, 0.0f);
        }
        _pendingSetRequests.erase(name);
    }
}

void ParameterManager::setTarget(uint8_t system, uint8_t component) {
    _targetSystem = system;
    _targetComponent = component;
}

// ============================================================================
// Private Methods - MAVLink Messages
// ============================================================================

void ParameterManager::sendParamRequestRead(const std::string& name) {
    mavlink_message_t msg;
    char paramId[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN] = {0};
    std::strncpy(paramId, name.c_str(), MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN);
    
    mavlink_msg_param_request_read_pack(
        _connectionManager->getSystemId(),
        _connectionManager->getComponentId(),
        &msg,
        _targetSystem,
        _targetComponent,
        paramId,
        -1 // Use name, not index
    );
    
    _connectionManager->sendMessage(msg);
}

void ParameterManager::sendParamRequestList() {
    mavlink_message_t msg;
    
    mavlink_msg_param_request_list_pack(
        _connectionManager->getSystemId(),
        _connectionManager->getComponentId(),
        &msg,
        _targetSystem,
        _targetComponent
    );
    
    _connectionManager->sendMessage(msg);
}

void ParameterManager::sendParamSet(const std::string& name, float value, uint8_t type) {
    mavlink_message_t msg;
    mavlink_param_set_t paramSet = {};
    
    paramSet.target_system = _targetSystem;
    paramSet.target_component = _targetComponent;
    paramSet.param_type = static_cast<MAV_PARAM_TYPE>(type);
    
    // Copy parameter name
    std::strncpy(paramSet.param_id, name.c_str(), MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN);
    
    // Convert float value to param_value wire format based on type
    paramSet.param_value = floatToParamUnion(value, type);
    
    mavlink_msg_param_set_encode(
        _connectionManager->getSystemId(),
        _connectionManager->getComponentId(),
        &msg,
        &paramSet
    );
    
    _connectionManager->sendMessage(msg);
}

// ============================================================================
// Type Conversion Helpers
// ============================================================================

float ParameterManager::paramUnionToFloat(float paramValue, uint8_t type) {
    // MAVLink sends all parameters as 4-byte float on the wire
    // We need to reinterpret those bytes based on the actual type
    mavlink_param_union_t paramUnion;
    paramUnion.param_float = paramValue; // Copy the 4 bytes
    
    switch (type) {
        case MAV_PARAM_TYPE_UINT8:
            return static_cast<float>(paramUnion.param_uint8);
        case MAV_PARAM_TYPE_INT8:
            return static_cast<float>(paramUnion.param_int8);
        case MAV_PARAM_TYPE_UINT16:
            return static_cast<float>(paramUnion.param_uint16);
        case MAV_PARAM_TYPE_INT16:
            return static_cast<float>(paramUnion.param_int16);
        case MAV_PARAM_TYPE_UINT32:
            return static_cast<float>(paramUnion.param_uint32);
        case MAV_PARAM_TYPE_INT32:
            return static_cast<float>(paramUnion.param_int32);
        case MAV_PARAM_TYPE_REAL32:
        default:
            return paramUnion.param_float;
    }
}

float ParameterManager::floatToParamUnion(float value, uint8_t type) {
    // MAVLink sends all parameters as 4-byte float on the wire
    // We need to encode the typed value into those 4 bytes
    mavlink_param_union_t paramUnion;
    
    switch (type) {
        case MAV_PARAM_TYPE_UINT8:
            paramUnion.param_uint8 = static_cast<uint8_t>(std::round(value));
            break;
        case MAV_PARAM_TYPE_INT8:
            paramUnion.param_int8 = static_cast<int8_t>(std::round(value));
            break;
        case MAV_PARAM_TYPE_UINT16:
            paramUnion.param_uint16 = static_cast<uint16_t>(std::round(value));
            break;
        case MAV_PARAM_TYPE_INT16:
            paramUnion.param_int16 = static_cast<int16_t>(std::round(value));
            break;
        case MAV_PARAM_TYPE_UINT32:
            paramUnion.param_uint32 = static_cast<uint32_t>(std::round(value));
            break;
        case MAV_PARAM_TYPE_INT32:
            paramUnion.param_int32 = static_cast<int32_t>(std::round(value));
            break;
        case MAV_PARAM_TYPE_REAL32:
        default:
            paramUnion.param_float = value;
            break;
    }
    
    // Return the 4 bytes as float for wire transmission
    return paramUnion.param_float;
}

} // namespace margelo::nitro::mavlink
