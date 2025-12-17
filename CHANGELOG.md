# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Flight Mode Control**
  - Implemented `setFlightMode()` with ArduPilot Copter mode mappings
  - Support for 28 flight modes (STABILIZE, GUIDED, RTL, AUTO, etc.)
  - Case-insensitive mode name matching
  - Alternative mode names (e.g., "ALT_HOLD" or "ALTITUDE HOLD")
  - Sends MAV_CMD_DO_SET_MODE with proper custom_mode values
  - Based on QGroundControl ArduCopterFirmwarePlugin implementation

### Fixed

- **UDP Connection**
  - Fixed bidirectional UDP communication by sending GCS HEARTBEAT
  - Vehicle now correctly receives HEARTBEAT and replies with its own
  - Added background thread for periodic GCS HEARTBEAT transmission (1Hz)
  - Resolves issue where `isConnected()` remained false after UDP connect

### Planned

- TCP connection support (implementation ready, needs testing)
- Serial connection support (platform-specific)
- PX4 flight mode mappings (currently ArduPilot only)
- Full parameter management with caching
- Mission upload/download protocol
- Event callbacks for telemetry updates
- Auto-reconnection logic
- Multi-vehicle support
- FTP file transfer

## [0.0.1] - 2024-12-XX

### Added

#### Core Features

- **MAVLink v2.0 Protocol Support**
  - Complete message parsing and generation
  - Standard dialect (common messages)
  - Checksum validation
  - Sequence number tracking

#### Connection Management

- **UDP Connection** (fully implemented)
  - Non-blocking socket I/O
  - Dedicated send/receive threads
  - Queue-based message sending
  - Connection status monitoring
- **TCP Connection** (stubbed, not implemented)
- **Serial Connection** (stubbed, not implemented)
- **Connection API**
  - `connectWithConfig()` - Connect with configuration
  - `disconnect()` - Close connection
  - `isConnected()` - Check connection status

#### Vehicle State & Telemetry

- **Thread-safe State Management**
  - Atomic variables for all telemetry
  - Mutex-protected operations
  - Real-time message handling
- **Telemetry Getters** (18 methods)
  - Position: latitude, longitude, altitude, heading
  - Velocity: ground speed, air speed, climb rate
  - Attitude: roll, pitch, yaw
  - Battery: voltage, remaining percentage
  - GPS: fix type, satellite count
  - Status: armed, flying, flight mode
- **Message Handlers**
  - HEARTBEAT (system ID, armed status, flight mode)
  - GLOBAL_POSITION_INT (GPS position)
  - ATTITUDE (roll, pitch, yaw)
  - BATTERY_STATUS (voltage, remaining)
  - GPS_RAW_INT (fix type, satellites)
  - VFR_HUD (speeds, heading)
  - SYS_STATUS (system status)
- **Flight Mode Mapping**
  - ArduPilot modes (STABILIZE, LOITER, RTL, etc.)
  - PX4 modes (MANUAL, POSCTL, AUTO_MISSION, etc.)

#### Vehicle Control

- **Basic Control**
  - `setArmed()` - Arm/disarm with force option
  - `setFlightMode()` - Change flight mode by name
- **Guided Commands** (11 methods)
  - `guidedTakeoff()` - Takeoff to altitude
  - `guidedLand()` - Land at current position
  - `guidedRTL()` - Return to launch (with smart RTL option)
  - `guidedGotoCoordinate()` - Fly to GPS coordinate
  - `guidedChangeAltitude()` - Change altitude by offset
  - `guidedChangeHeading()` - Change heading
  - `guidedOrbitParams()` - Orbit around point
  - `guidedROICoordinate()` - Point camera at location
  - `guidedClearROI()` - Clear region of interest
  - `pauseVehicle()` - Hold position
  - `emergencyStop()` - Emergency disarm

#### Command Execution

- **Reliable Command Delivery**
  - Automatic retry logic (3 attempts)
  - Timeout handling (3 seconds)
  - ACK/NACK processing
  - Callback-based completion
- **Command Results**
  - ACCEPTED - Command successful
  - IN_PROGRESS - Command executing
  - DENIED - Command rejected
  - FAILED - Command failed
  - TIMEOUT - No response received

#### Mission Management

- **Mission Control**
  - `startMission()` - Start current mission
  - `setCurrentMissionItem()` - Jump to waypoint
  - `getCurrentMissionItem()` - Get current waypoint
  - `clearMission()` - Clear all waypoints
- **Messages Supported**
  - MISSION_SET_CURRENT
  - MISSION_CLEAR_ALL
  - MISSION_CURRENT (monitoring)

#### Camera & Gimbal

- **Camera Control**
  - `triggerCamera()` - Take photo
  - `startVideoRecording()` - Start video
  - `stopVideoRecording()` - Stop video
- **Gimbal Control**
  - `setGimbalAttitudeParams()` - Control gimbal attitude (pitch, roll, yaw)

#### Parameters

- **Basic Parameter Operations**
  - `setParameter()` - Set parameter value
  - `setParameterValue()` - Set with ParameterSet object
  - `refreshParameters()` - Request full parameter list
  - `getParameter()` - Get parameter (stubbed, not fully implemented)

#### Advanced Features

- **Manual Control**
  - `sendManualControlInput()` - RC override
  - 4-axis control (pitch, roll, throttle, yaw)
  - Button support
- **Data Streams**
  - `requestDataStreamParams()` - Request specific data rates
  - Support for all MAV_DATA_STREAM types
- **System Commands**
  - `rebootAutopilot()` - Reboot autopilot
  - `sendCommandParams()` - Send raw MAVLink commands

#### TypeScript API

- **Type Definitions** (8 types)
  - `ConnectionConfig` - Connection configuration
  - `Coordinate` - GPS coordinate with altitude
  - `ManualControlInput` - Manual control values
  - `OrbitParams` - Orbit command parameters
  - `CommandParams` - Raw command parameters
  - `GimbalAttitude` - Gimbal attitude angles
  - `DataStreamRequest` - Data stream configuration
  - `ParameterSet` - Parameter name/value pair
- **Helper Functions**
  - `connectUDP()` - Quick UDP connection
  - `connectTCP()` - Quick TCP connection
  - `connectSerial()` - Quick serial connection
  - `getTelemetry()` - Get all telemetry at once

#### Build System

- **CMake Configuration**
  - C++17 standard
  - Platform-specific linking (Winsock, pthread, Foundation)
  - MAVLink include paths
  - Nitro Modules integration

#### Documentation

- **Comprehensive Docs**
  - README.md - Full API reference
  - QUICKSTART.md - 5-minute getting started guide
  - IMPLEMENTATION.md - Architecture and implementation details
  - TODO.md - Feature roadmap and task list
  - Example App - Complete working example

#### Examples

- **Sample Application**
  - Connection management UI
  - Real-time telemetry display
  - Vehicle control buttons
  - SITL testing instructions

### Technical Details

#### Architecture

- **Modular Design**
  - Separation of concerns (connection, state, commands)
  - Single responsibility principle
  - Dependency injection pattern
- **Thread Safety**
  - Atomic variables for lock-free reads
  - Mutex protection for writes
  - Separate threads for send/receive
- **Performance**
  - Zero-copy message passing where possible
  - Queue-based async operations
  - Cached telemetry for instant reads

#### Platform Support

- **iOS**: Supported via Nitro Modules
- **Android**: Supported via Nitro Modules
- **Tested**: Windows (development), SITL

#### Dependencies

- react-native-nitro-modules: ^0.x.x
- React Native: >= 0.70.0
- CMake: >= 3.9.0
- C++17 compiler

### Known Issues

- TCP connection not implemented (stub only)
- Serial connection not implemented (stub only)
- Parameter get requires full implementation
- No parameter caching
- Mission upload/download not supported
- Single vehicle only (no multi-vehicle support)

### Breaking Changes

- None (initial release)

## Release Notes

### v0.0.1 - Initial Beta Release

This is the first beta release of React Native MAVLink. The core functionality is complete and tested with ArduPilot SITL.

**What's Working:**
✅ UDP connection to drone/SITL
✅ Real-time telemetry (position, velocity, attitude, battery, GPS)
✅ Vehicle control (arm/disarm, flight modes)
✅ Guided commands (takeoff, land, RTL, goto)
✅ Mission control (start, set current, clear)
✅ Camera/gimbal control
✅ Command execution with retry/ACK

**What's Next:**
🔄 TCP and Serial connections
🔄 Full parameter management
🔄 Mission upload/download
🔄 Event callbacks
🔄 Multi-vehicle support

**Testing:**
Tested with ArduPilot SITL (ArduCopter 4.x). Ready for integration testing with real hardware.

**Installation:**

```bash
npm install react-native-mavlink
```

**Quick Start:**

```typescript
import { mavlink, connectUDP, getTelemetry } from 'react-native-mavlink'

// Connect
await connectUDP('127.0.0.1', 14550)

// Get telemetry
const telemetry = getTelemetry()

// Arm and takeoff
await mavlink.setFlightMode('GUIDED')
await mavlink.setArmed(true, false)
await mavlink.guidedTakeoff(10)
```

See [QUICKSTART.md](./QUICKSTART.md) for complete guide.

---

[Unreleased]: https://github.com/lctuan-duck/react-native-mavlink/compare/v0.0.1...HEAD
[0.0.1]: https://github.com/lctuan-duck/react-native-mavlink/releases/tag/v0.0.1
