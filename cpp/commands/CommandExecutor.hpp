/**
 * CommandExecutor.hpp
 * Executes MAVLink commands with retry and acknowledgment handling
 * Based on QGroundControl Vehicle command execution
 */

#pragma once

#include <string>
#include <functional>
#include <memory>
#include <mutex>
#include <map>
#include <chrono>
#include "../MAVLink/v2.0/common/mavlink.h"

namespace margelo::nitro::mavlink {

class ConnectionManager;

enum class CommandResult {
    IN_PROGRESS,
    SUCCESS,
    FAILED,
    TIMEOUT,
    DENIED
};

using CommandCallback = std::function<void(CommandResult result, uint8_t progress)>;

struct PendingCommand {
    mavlink_command_long_t command;
    CommandCallback callback;
    std::chrono::steady_clock::time_point sentTime;
    int retriesLeft;
    uint16_t sequence;
};

class CommandExecutor {
public:
    CommandExecutor(std::shared_ptr<ConnectionManager> connectionManager,
                   uint8_t targetSystem, uint8_t targetComponent);
    ~CommandExecutor() = default;

    // ============================================================================
    // Command Execution
    // ============================================================================
    
    /**
     * Send a MAVLink command and wait for acknowledgment
     * @param command Command ID (MAV_CMD_*)
     * @param param1-7 Command parameters
     * @param callback Called when command completes or fails
     * @param retries Number of retries if no ack received
     * @return Command sequence number for tracking
     */
    uint16_t sendCommand(
        uint16_t command,
        float param1, float param2, float param3, float param4,
        float param5, float param6, float param7,
        CommandCallback callback = nullptr,
        int retries = 3
    );
    
    /**
     * Handle COMMAND_ACK message
     */
    void handleCommandAck(const mavlink_command_ack_t& ack);
    
    /**
     * Check for command timeouts and retry
     */
    void checkTimeouts();
    
    /**
     * Set target system/component
     */
    void setTarget(uint8_t system, uint8_t component);
    
private:
    std::shared_ptr<ConnectionManager> _connectionManager;
    uint8_t _targetSystem;
    uint8_t _targetComponent;
    
    // Pending commands
    std::map<uint16_t, PendingCommand> _pendingCommands;
    std::mutex _commandsMutex;
    uint16_t _nextCommandSequence{0};
    
    // Timeouts
    static constexpr int COMMAND_TIMEOUT_MS = 3000;
    static constexpr int COMMAND_RETRY_MS = 1000;
    
    // Helpers
    uint16_t getNextSequence();
    void sendCommandMessage(const PendingCommand& pending);
    void completeCommand(uint16_t sequence, CommandResult result, uint8_t progress = 0);
    std::string commandResultToString(CommandResult result) const;
};

} // namespace margelo::nitro::mavlink
