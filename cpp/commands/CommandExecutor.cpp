/**
 * CommandExecutor.cpp
 * Implementation
 */

#include "CommandExecutor.hpp"
#include "../connection/ConnectionManager.hpp"
#include "../mavlink/v2.0/common/mavlink.h"
#include <iostream>

namespace margelo::nitro::mavlink {

CommandExecutor::CommandExecutor(std::shared_ptr<ConnectionManager> connectionManager,
                                 uint8_t targetSystem, uint8_t targetComponent)
    : _connectionManager(connectionManager)
    , _targetSystem(targetSystem)
    , _targetComponent(targetComponent) {
}

// ============================================================================
// Command Execution
// ============================================================================

uint16_t CommandExecutor::sendCommand(
    uint16_t command,
    float param1, float param2, float param3, float param4,
    float param5, float param6, float param7,
    CommandCallback callback,
    int retries) {
    
    uint16_t sequence = getNextSequence();
    
    PendingCommand pending;
    pending.command.command = command;
    pending.command.target_system = _targetSystem;
    pending.command.target_component = _targetComponent;
    pending.command.confirmation = 0;
    pending.command.param1 = param1;
    pending.command.param2 = param2;
    pending.command.param3 = param3;
    pending.command.param4 = param4;
    pending.command.param5 = param5;
    pending.command.param6 = param6;
    pending.command.param7 = param7;
    pending.callback = callback;
    pending.sentTime = std::chrono::steady_clock::now();
    pending.retriesLeft = retries;
    pending.sequence = sequence;
    
    {
        std::lock_guard<std::mutex> lock(_commandsMutex);
        _pendingCommands[sequence] = pending;
    }
    
    sendCommandMessage(pending);
    
    return sequence;
}

void CommandExecutor::handleCommandAck(const mavlink_command_ack_t& ack) {
    std::lock_guard<std::mutex> lock(_commandsMutex);
    
    // Find matching command (match by command ID since we don't have sequence in ACK)
    for (auto it = _pendingCommands.begin(); it != _pendingCommands.end(); ++it) {
        if (it->second.command.command == ack.command) {
            CommandResult result;
            
            switch (ack.result) {
                case MAV_RESULT_ACCEPTED:
                    result = CommandResult::SUCCESS;
                    break;
                case MAV_RESULT_IN_PROGRESS:
                    result = CommandResult::IN_PROGRESS;
                    if (it->second.callback) {
                        it->second.callback(result, ack.progress);
                    }
                    return; // Don't remove from pending
                case MAV_RESULT_DENIED:
                    result = CommandResult::DENIED;
                    break;
                case MAV_RESULT_UNSUPPORTED:
                case MAV_RESULT_FAILED:
                case MAV_RESULT_TEMPORARILY_REJECTED:
                default:
                    result = CommandResult::FAILED;
                    break;
            }
            
            // Complete command
            if (it->second.callback) {
                it->second.callback(result, 100);
            }
            
            _pendingCommands.erase(it);
            return;
        }
    }
}

void CommandExecutor::checkTimeouts() {
    std::lock_guard<std::mutex> lock(_commandsMutex);
    auto now = std::chrono::steady_clock::now();
    
    std::vector<uint16_t> toRetry;
    std::vector<uint16_t> toTimeout;
    
    for (auto& pair : _pendingCommands) {
        auto& pending = pair.second;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - pending.sentTime).count();
        
        if (elapsed > COMMAND_TIMEOUT_MS) {
            if (pending.retriesLeft > 0) {
                toRetry.push_back(pair.first);
            } else {
                toTimeout.push_back(pair.first);
            }
        }
    }
    
    // Retry commands
    for (uint16_t seq : toRetry) {
        auto& pending = _pendingCommands[seq];
        pending.retriesLeft--;
        pending.sentTime = now;
        pending.command.confirmation++; // Increment confirmation for retry
        sendCommandMessage(pending);
    }
    
    // Timeout commands
    for (uint16_t seq : toTimeout) {
        auto& pending = _pendingCommands[seq];
        if (pending.callback) {
            pending.callback(CommandResult::TIMEOUT, 0);
        }
        _pendingCommands.erase(seq);
    }
}

void CommandExecutor::setTarget(uint8_t system, uint8_t component) {
    _targetSystem = system;
    _targetComponent = component;
}

// ============================================================================
// Private Methods
// ============================================================================

uint16_t CommandExecutor::getNextSequence() {
    return _nextCommandSequence++;
}

void CommandExecutor::sendCommandMessage(const PendingCommand& pending) {
    mavlink_message_t msg;
    mavlink_msg_command_long_encode(
        _connectionManager->getSystemId(),
        _connectionManager->getComponentId(),
        &msg,
        &pending.command
    );
    
    _connectionManager->sendMessage(msg);
}

void CommandExecutor::completeCommand(uint16_t sequence, CommandResult result, uint8_t progress) {
    std::lock_guard<std::mutex> lock(_commandsMutex);
    
    auto it = _pendingCommands.find(sequence);
    if (it != _pendingCommands.end()) {
        if (it->second.callback) {
            it->second.callback(result, progress);
        }
        _pendingCommands.erase(it);
    }
}

std::string CommandExecutor::commandResultToString(CommandResult result) const {
    switch (result) {
        case CommandResult::IN_PROGRESS: return "IN_PROGRESS";
        case CommandResult::SUCCESS: return "SUCCESS";
        case CommandResult::FAILED: return "FAILED";
        case CommandResult::TIMEOUT: return "TIMEOUT";
        case CommandResult::DENIED: return "DENIED";
        default: return "UNKNOWN";
    }
}

} // namespace margelo::nitro::mavlink
