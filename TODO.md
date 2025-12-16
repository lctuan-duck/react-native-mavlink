# TODO List - React Native MAVLink

## ✅ Completed (v0.0.1)

### Core Implementation
- [x] TypeScript API design (MAVLink.nitro.ts)
- [x] Nitrogen code generation (HybridMAVLinkSpec)
- [x] Folder structure (connection/, vehicle/, commands/, parameters/, mavlink_utils/)
- [x] VehicleState implementation (thread-safe telemetry)
- [x] ConnectionManager UDP implementation
- [x] CommandExecutor with retry/ACK
- [x] HybridMAVLink main class (wire up all components)
- [x] All telemetry getters (18 methods)
- [x] Guided commands (takeoff, land, RTL, goto, orbit, ROI)
- [x] Basic mission control (start, set current, clear)
- [x] Camera/gimbal control
- [x] Manual control input
- [x] Data stream requests
- [x] Build system (CMakeLists.txt)
- [x] Example app with UI
- [x] Documentation (README, QUICKSTART, IMPLEMENTATION, CHANGELOG)

## ✅ Completed (v0.0.2)

### Connection Enhancements
- [x] TCP Connection Implementation
  - Blocking connect with 5s timeout
  - Non-blocking I/O after connection
  - Platform-specific (Winsock2 + POSIX)
  - Tested connection flow
  
- [x] Serial Connection Implementation
  - Windows: DCB configuration with CreateFile/ReadFile/WriteFile
  - Linux/macOS: termios configuration
  - Baud rate support (9600-921600)
  - Platform-specific disconnect handling
  
- [x] Connection-aware receive/send logic
  - UDP: recvfrom/sendto
  - TCP: recv/send  
  - Serial: ReadFile/read (platform-specific)

### Parameter Management
- [x] ParameterManager class created
- [x] Parameter cache with timestamp tracking
- [x] `getParameter()` with retry + timeout (1s)
- [x] `setParameter()` with PARAM_SET + ACK
- [x] `requestAllParameters()` (PARAM_REQUEST_LIST)
- [x] All MAV_PARAM_TYPE support (UINT8/16/32, INT8/16/32, REAL32)
- [x] Type conversion helpers
- [x] Thread-safe operations
- [x] Integration with HybridMAVLink
- [x] Promise-based API

### Code Quality
- [x] Fixed hardcoded ArduCopter flight mode mappings
- [x] Removed firmware-specific strings from setFlightMode()
- [x] Made all commands firmware-agnostic
- [x] Verified conversions match QGC (lat/lon, altitude, velocity)
- [x] Logic validation against QGroundControl reference

## 🚀 High Priority (Next Release)

### Testing & Validation

### 1. SITL Testing
**Status**: Ready to test
**Tasks**:
- [ ] Set up ArduPilot SITL environment
- [ ] Test UDP connection (127.0.0.1:14550)
- [ ] Verify all telemetry data
- [ ] Test parameter get/set operations
- [ ] Test guided commands (arm, takeoff, goto, RTL)
- [ ] Verify command ACK handling
- [ ] Test connection loss recovery
- [ ] Profile performance

### 2. Mission Upload/Download
**Files**: New `cpp/mission/MissionManager.hpp`
**Status**: Not implemented
**Tasks**:
- [ ] Create MissionManager class
- [ ] Implement mission upload protocol:
  - Send MISSION_COUNT
  - Handle MISSION_REQUEST_INT
  - Send MISSION_ITEM_INT
  - Wait for MISSION_ACK
- [ ] Implement mission download protocol:
  - Send MISSION_REQUEST_LIST
  - Handle MISSION_COUNT
  - Send MISSION_REQUEST_INT for each item
  - Collect all MISSION_ITEM_INT
- [ ] Add mission validation
- [ ] Support different mission types (waypoints, rally, fence)

**Messages**: MISSION_COUNT, MISSION_ITEM_INT, MISSION_REQUEST_INT, MISSION_ACK

## Medium Priority (Enhanced Features)

### 3. Event Callbacks
**File**: `src/specs/MAVLink.nitro.ts`, `cpp/HybridMAVLink.hpp`
**Status**: Not implemented
**Tasks**:
- [ ] Add callback types to TypeScript:
  ```typescript
  type TelemetryCallback = (telemetry: Telemetry) => void
  type StatusCallback = (armed: boolean, mode: string) => void
  ```
- [ ] Implement callback registration:
  ```cpp
  void registerTelemetryCallback(std::function<void(...)> callback);
  ```
- [ ] Call callbacks from message handlers
- [ ] Handle callback lifecycle (registration/deregistration)
- [ ] Test callback performance (avoid blocking)

### 4. Auto-Reconnection
**File**: `cpp/connection/ConnectionManager.cpp`
**Status**: Not implemented
**Tasks**:
- [ ] Detect connection loss (no HEARTBEAT for N seconds)
- [ ] Implement reconnection attempts (exponential backoff)
- [ ] Add max retry limit
- [ ] Notify React Native of reconnection events
- [ ] Resume data streams after reconnection

### 5. Multi-Vehicle Support
**Files**: All (architectural change)
**Status**: Not implemented
**Tasks**:
- [ ] Support multiple HybridMAVLink instances
- [ ] Filter messages by system ID
- [ ] Add vehicle discovery mechanism
- [ ] Handle multiple connections simultaneously
- [ ] Update API to accept systemId parameter

### 6. Mission Validation
**File**: New `cpp/mission/MissionValidator.hpp`
**Status**: Not implemented
**Tasks**:
- [ ] Validate mission item sequences
- [ ] Check coordinate bounds
- [ ] Verify command parameters
- [ ] Estimate mission duration
- [ ] Check geofence conflicts

## Low Priority (Nice to Have)

### 7. FTP File Transfer
**File**: New `cpp/ftp/FTPManager.hpp`
**Status**: Not implemented
**Tasks**:
- [ ] Implement MAVLink FTP protocol
- [ ] Support file upload/download
- [ ] Handle large files (chunking)
- [ ] Add progress callbacks
- [ ] Implement file listing

**Messages**: FILE_TRANSFER_PROTOCOL

### 10. Advanced Camera Control
**File**: `cpp/camera/CameraManager.hpp`
**Status**: Basic trigger implemented
**Tasks**:
- [ ] Implement CAMERA_INFORMATION handling
- [ ] Support camera capabilities query
- [ ] Add zoom control
- [ ] Handle image capture modes
- [ ] Support storage management

**Messages**: CAMERA_INFORMATION, CAMERA_SETTINGS, CAMERA_CAPTURE_STATUS

### 11. Geofence Management
**File**: New `cpp/geofence/GeofenceManager.hpp`
**Status**: Not implemented
**Tasks**:
- [ ] Upload polygon geofences
- [ ] Upload circular geofences
- [ ] Download existing geofences
- [ ] Monitor fence breach status

### 12. Rally Point Management
**File**: New `cpp/rally/RallyPointManager.hpp`
**Status**: Not implemented
**Tasks**:
- [ ] Upload rally points
- [ ] Download rally points
- [ ] Set active rally point

### 13. RC Override
**File**: `cpp/HybridMAVLink.cpp`
**Status**: Manual control implemented, RC channels not
**Tasks**:
- [ ] Add `sendRCChannelsOverride()` method
- [ ] Support 18 RC channels
- [ ] Handle PWM values (1000-2000)

**Messages**: RC_CHANNELS_OVERRIDE

### 14. Logging
**File**: New `cpp/logging/Logger.hpp`
**Status**: Not implemented
**Tasks**:
- [ ] Implement structured logging
- [ ] Add log levels (DEBUG, INFO, WARN, ERROR)
- [ ] Write logs to file
- [ ] Expose logs to React Native
- [ ] Add log filtering

### 15. Performance Monitoring
**File**: New `cpp/monitoring/PerformanceMonitor.hpp`
**Status**: Not implemented
**Tasks**:
- [ ] Track message receive rate
- [ ] Monitor command latency
- [ ] Detect message drops
- [ ] Expose metrics to React Native

## Testing & Documentation

### 16. Unit Tests
**Files**: New `cpp/tests/` directory
**Status**: Not implemented
**Tasks**:
- [ ] Set up Google Test framework
- [ ] Write ConnectionManager tests
- [ ] Write VehicleState tests
- [ ] Write CommandExecutor tests
- [ ] Add mock MAVLink message generation
- [ ] Test edge cases (timeouts, errors)

### 17. Integration Tests
**Files**: New `example/integration-tests/`
**Status**: Not implemented
**Tasks**:
- [ ] Create SITL test harness
- [ ] Test full flight scenarios
- [ ] Test mission execution
- [ ] Test parameter operations
- [ ] Measure performance

### 18. API Documentation
**Files**: `docs/` directory
**Status**: README exists, detailed docs missing
**Tasks**:
- [ ] Create API reference for each method
- [ ] Add usage examples for all features
- [ ] Document error codes
- [ ] Add troubleshooting guide
- [ ] Create video tutorials

### 19. Code Examples
**Files**: `examples/` directory
**Status**: Basic App.tsx exists
**Tasks**:
- [ ] Add mission planning example
- [ ] Add camera control example
- [ ] Add parameter tuning example
- [ ] Add telemetry logging example

## Build & Distribution

### 20. CI/CD Pipeline
**Files**: `.github/workflows/`
**Status**: Not implemented
**Tasks**:
- [ ] Set up GitHub Actions
- [ ] Add TypeScript lint checks
- [ ] Add C++ compilation checks
- [ ] Run unit tests on PR
- [ ] Auto-publish to npm on release

### 21. Platform Testing
**Status**: Not tested
**Tasks**:
- [ ] Test on iOS (physical device)
- [ ] Test on Android (physical device)
- [ ] Test with real drone hardware
- [ ] Test on different autopilots (PX4, ArduPilot)
- [ ] Test on different connection types

### 22. NPM Package
**Status**: Package structure ready, not published
**Tasks**:
- [ ] Verify package.json files list
- [ ] Test installation from npm
- [ ] Add installation instructions
- [ ] Create changelog
- [ ] Publish v0.1.0

## Bug Fixes & Improvements

### 23. Error Handling Improvements
**Tasks**:
- [ ] Add error codes enum
- [ ] Improve error messages
- [ ] Handle all edge cases
- [ ] Add retry logic for failures
- [ ] Validate all inputs

### 24. Memory Management
**Tasks**:
- [ ] Audit for memory leaks
- [ ] Add smart pointer usage everywhere
- [ ] Profile memory usage
- [ ] Optimize message parsing

### 25. Thread Safety Audit
**Tasks**:
- [ ] Review all atomic operations
- [ ] Check mutex usage
- [ ] Verify no race conditions
- [ ] Test under high load

## Future Features

### 26. Video Streaming Integration
- [ ] Research React Native video streaming
- [ ] Integrate with MAVLink video metadata
- [ ] Support RTSP/UDP video

### 27. Terrain Following
- [ ] Add terrain height queries
- [ ] Support terrain following mode
- [ ] Handle terrain data downloads

### 28. Swarm Support
- [ ] Multi-vehicle coordination
- [ ] Formation flying commands
- [ ] Collective mission planning

### 29. Custom MAVLink Dialects
- [ ] Support loading custom XML definitions
- [ ] Generate code for custom messages
- [ ] Runtime message registration

### 30. Web Support (React Native Web)
- [ ] WebSocket transport
- [ ] Browser-compatible implementation
- [ ] SharedArrayBuffer for performance

---

## Progress Tracking

- **Total Tasks**: 30 major features
- **Completed**: 1 (Core MAVLink implementation)
- **In Progress**: 0
- **Not Started**: 29

## Next Steps (Recommended Order)

1. **TCP Connection** - Quick win, similar to UDP
2. **Parameter Management** - Highly requested feature
3. **Mission Upload** - Core functionality for autonomous flight
4. **Event Callbacks** - Better UX for React Native
5. **Unit Tests** - Ensure stability before adding more features

---

**Last Updated**: 2024
**Maintainer**: Le Cong Tuan <lctuan.dev@gmail.com>
