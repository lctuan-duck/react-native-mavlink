# react-native-mavlink

React Native Nitro Module tích hợp MAVLink protocol cho ứng dụng GCS (Ground Control Station).

## ✨ Features

### ✅ Fully Implemented (Production Ready)

- **Transport Layer**
  - ✅ UDP transport (local bind + remote target)
  - ✅ TCP transport (client connection)
  - ✅ Thread-safe connection management
  - ✅ Cross-platform support (iOS/Android/Windows)

- **Telemetry & Events (17/20 events working)**
  - ✅ Real-time GPS position & velocity
  - ✅ Attitude (roll/pitch/yaw + rates)
  - ✅ Battery status (voltage/current/remaining)
  - ✅ Heartbeat with system info
  - ✅ Aggregated telemetry stream
  - ✅ System status monitoring
  - ✅ Connection state tracking

- **Commands & Control**
  - ✅ COMMAND_LONG with individual params (param1-7)
  - ✅ COMMAND_INT with position (x/y/z)
  - ✅ Command acknowledgment (ACK) events
  - ✅ Flight mode detection & events
  - ✅ Arm/disarm state detection & events

- **Parameters**
  - ✅ Request all parameters
  - ✅ Set parameter values
  - ✅ Parameter value events

- **Mission Planning**
  - ✅ Request mission list
  - ✅ Send mission count
  - ✅ Upload mission items (MISSION_ITEM_INT)
  - ✅ Clear all missions
  - ✅ Set current mission
  - ✅ Mission events (count/request/item/ack)

- **Data Logging**
  - ✅ Request log list
  - ✅ Download log data
  - ✅ Log entry & data events

### ⚠️ Stub Implementation (Extensible)

- **Camera/Gimbal Control** (3 events)
  - API ready, message parsing can be added when needed

## 📚 Documentation

- **[CODE_ORGANIZATION.md](CODE_ORGANIZATION.md)** - Cấu trúc code và tổ chức folder
- **[ARCHITECTURE_REFACTOR.md](ARCHITECTURE_REFACTOR.md)** - Chi tiết kiến trúc và design patterns
- **[cpp/bridge/README.md](cpp/bridge/README.md)** - Bridge implementations documentation

## 🚀 Installation

```bash
npm install react-native-mavlink
# or
yarn add react-native-mavlink

# iOS only
cd ios && pod install
```

## 📖 Usage Guide

### Basic Setup

```typescript
import { MAVLink } from 'react-native-mavlink'

// 1. Start UDP connection
// - port: Local port to RECEIVE MAVLink packets from drone
// - remoteHost/remotePort: Drone address to SEND commands to
await MAVLink.startUdp({
  port: 14550, // Listen port
  host: '0.0.0.0', // Bind address (optional, default)
  remoteHost: '192.168.1.100', // Drone IP
  remotePort: 14551, // Drone port
})

// OR: Start TCP connection to drone/server
await MAVLink.startTcp({
  host: '192.168.1.100', // Drone/server IP
  port: 5760, // Drone/server port
})

// Stop connection
await MAVLink.stopUdp()
await MAVLink.stopTcp()
```

### Telemetry & Monitoring

```typescript
// Listen for GPS position (includes velocity)
const gpsToken = MAVLink.onGps((gps) => {
  console.log(`Position: ${gps.latitude}, ${gps.longitude}`)
  console.log(`Altitude: ${gps.altitude}m`)
  console.log(`Satellites: ${gps.satellitesVisible}`)
})

// Listen for attitude (orientation)
const attToken = MAVLink.onAttitude((att) => {
  console.log(`Roll: ${att.roll}, Pitch: ${att.pitch}, Yaw: ${att.yaw}`)
})

// Listen for battery status
const batToken = MAVLink.onBattery((bat) => {
  console.log(`Battery: ${bat.voltage}V, ${bat.remaining}%`)
})

// Listen for heartbeat (system info)
const hbToken = MAVLink.onHeartbeat((hb) => {
  console.log(`System: ${hb.systemId}, Mode: ${hb.customMode}`)
})

// Aggregated telemetry (GPS + Attitude combined)
const telToken = MAVLink.onTelemetry((tel) => {
  if (tel.gps) {
    console.log(`Lat: ${tel.gps.latE7 / 1e7}`)
  }
  if (tel.attitude) {
    console.log(`Heading: ${tel.attitude.yaw}`)
  }
})

// Unsubscribe when done
MAVLink.offGps(gpsToken)
MAVLink.offAttitude(attToken)
```

### Flight Control

```typescript
// 1. ARM the drone
await MAVLink.sendCommandLong({
  command: 400, // MAV_CMD_COMPONENT_ARM_DISARM
  param1: 1, // 1 = ARM, 0 = DISARM
  targetSystem: 1,
})

// 2. Listen for command acknowledgment
MAVLink.onAck((ack) => {
  if (ack.command === 400) {
    console.log(ack.result === 0 ? 'Armed!' : 'Arm failed')
  }
})

// 3. Takeoff
await MAVLink.sendCommandLong({
  command: 22, // MAV_CMD_NAV_TAKEOFF
  param7: 10, // Altitude in meters
})

// 4. Go to position
await MAVLink.sendCommandInt({
  command: 192, // MAV_CMD_DO_REPOSITION
  x: 47.3977419 * 1e7, // Latitude (degrees * 1e7)
  y: 8.5455935 * 1e7, // Longitude (degrees * 1e7)
  z: 50, // Altitude (meters)
  frame: 3, // MAV_FRAME_GLOBAL_RELATIVE_ALT
  param4: -1, // Yaw (degrees, -1 = don't change)
})

// 5. Return to launch
await MAVLink.sendCommandLong({
  command: 20, // MAV_CMD_NAV_RETURN_TO_LAUNCH
})

// 6. Land
await MAVLink.sendCommandLong({
  command: 21, // MAV_CMD_NAV_LAND
})
```

### Mode & State Monitoring

```typescript
// Detect flight mode changes
MAVLink.onMode((mode) => {
  console.log(`Mode changed: base=${mode.baseMode}, custom=${mode.customMode}`)
  // baseMode bit flags: 0x80 = armed, 0x40 = guided, etc.
})

// Detect arm/disarm changes
MAVLink.onArm((arm) => {
  console.log(arm.armed ? 'ARMED!' : 'Disarmed')
})

// System status (battery + more)
MAVLink.onStatus((status) => {
  if (status.battery) {
    console.log(`Battery: ${status.battery.voltage}V`)
  }
  if (status.text) {
    console.log(`Status: ${status.text.message}`)
  }
})
```

### Parameters

```typescript
// Request all parameters from drone
await MAVLink.requestParams()

// Listen for parameter values
MAVLink.onParameter((param) => {
  console.log(`${param.name} = ${param.value}`)
})

// Set a parameter
await MAVLink.setParam('RTL_ALT', 15000) // RTL altitude in cm
await MAVLink.setParam('SYSID_THISMAV', 1)
```

### Mission Planning

```typescript
// 1. Upload a mission
const waypoints: MissionItemInt[] = [
  {
    seq: 0,
    frame: 3, // MAV_FRAME_GLOBAL_RELATIVE_ALT
    command: 22, // MAV_CMD_NAV_TAKEOFF
    current: 1, // First item
    autocontinue: 1,
    x: 0,
    y: 0,
    z: 10, // Takeoff to 10m
  },
  {
    seq: 1,
    frame: 3,
    command: 16, // MAV_CMD_NAV_WAYPOINT
    current: 0,
    autocontinue: 1,
    x: 47.397 * 1e7, // Target latitude
    y: 8.545 * 1e7, // Target longitude
    z: 50, // Altitude 50m
  },
  {
    seq: 2,
    frame: 3,
    command: 20, // MAV_CMD_NAV_RETURN_TO_LAUNCH
    current: 0,
    autocontinue: 1,
    x: 0,
    y: 0,
    z: 0,
  },
]

// Send mission count
await MAVLink.sendMissionCount(waypoints.length)

// Listen for mission requests from drone
MAVLink.onMissionRequest((req) => {
  console.log(`Drone requesting waypoint #${req.seq}`)
  // Send the requested waypoint
  MAVLink.sendMissionItemInt(waypoints[req.seq])
})

// Listen for mission acknowledgment
MAVLink.onMissionAck((ack) => {
  console.log(ack.type === 0 ? 'Mission accepted!' : 'Mission rejected')
})

// 2. Request current mission from drone
await MAVLink.requestMissionList()

MAVLink.onMissionCount((count) => {
  console.log(`Drone has ${count.count} waypoints`)
})

MAVLink.onMissionItem((item) => {
  console.log(`Waypoint #${item.item.seq}:`, item.item)
})

// 3. Clear all missions
await MAVLink.clearAllMissions()

// 4. Set current mission item
await MAVLink.setCurrentMission(2) // Jump to waypoint #2
```

### Data Logging

```typescript
// 1. Request log list
await MAVLink.requestLogList()

MAVLink.onLogging((log) => {
  if (log.entry) {
    console.log(`Log ${log.entry.id}: ${log.entry.size} bytes`)
  }
  if (log.data) {
    // Log data chunk received
    console.log(`Downloaded ${log.data.data.byteLength} bytes`)
  }
})

// 2. Download a specific log
await MAVLink.requestLogData({
  id: 1, // Log ID
  ofs: 0, // Offset
  count: 4096, // Bytes to read
})
```

### Connection Events

```typescript
// Detect connection established
MAVLink.onConnect(() => {
  console.log('Connected to drone!')
})

// Detect disconnection
MAVLink.onDisconnect((reason) => {
  console.log('Disconnected:', reason)
})
```

### Advanced: Raw Message Handling

```typescript
// Listen for raw MAVLink packets
MAVLink.onRaw((raw: ArrayBuffer) => {
  console.log('Received raw packet:', raw.byteLength, 'bytes')
})

// Encode a custom message
const encoded = await MAVLink.encode(0, payload) // messageId, payload

// Decode a raw message
const decoded = await MAVLink.decode(rawBuffer)
console.log('Message ID:', decoded.messageId)
```

### Get Current State Snapshot

```typescript
// Get latest telemetry snapshot
const snapshot = await MAVLink.getTelemetrySnapshot()
if (snapshot) {
  console.log('Current GPS:', snapshot.gps)
  console.log('Current attitude:', snapshot.attitude)
}
```

## 🎯 Complete API Reference

### Connection Methods

- `startUdp(options)` - Start UDP transport
- `stopUdp()` - Stop UDP
- `startTcp(options)` - Start TCP transport
- `stopTcp()` - Stop TCP

### Command Methods

- `sendCommandLong(args)` - Send COMMAND_LONG
- `sendCommandInt(args)` - Send COMMAND_INT with position

### Parameter Methods

- `requestParams()` - Request all parameters
- `setParam(name, value)` - Set parameter value

### Mission Methods

- `requestMissionList()` - Request mission from drone
- `sendMissionCount(count)` - Send mission count
- `sendMissionItemInt(item)` - Upload waypoint
- `clearAllMissions()` - Clear all waypoints
- `setCurrentMission(seq)` - Jump to waypoint

### Logging Methods

- `requestLogList()` - Request log list
- `requestLogData(args)` - Download log data

### Event Listeners (17 working)

- `onHeartbeat(listener)` / `offHeartbeat(token)`
- `onGps(listener)` / `offGps(token)`
- `onAttitude(listener)` / `offAttitude(token)`
- `onBattery(listener)` / `offBattery(token)`
- `onTelemetry(listener)` / `offTelemetry(token)`
- `onStatus(listener)` / `offStatus(token)`
- `onAck(listener)` / `offAck(token)`
- `onParameter(listener)` / `offParameter(token)`
- `onMode(listener)` / `offMode(token)`
- `onArm(listener)` / `offArm(token)`
- `onConnect(listener)` / `offConnect(token)`
- `onDisconnect(listener)` / `offDisconnect(token)`
- `onMissionCount(listener)` / `offMissionCount(token)`
- `onMissionRequest(listener)` / `offMissionRequest(token)`
- `onMissionItem(listener)` / `offMissionItem(token)`
- `onMissionAck(listener)` / `offMissionAck(token)`
- `onLogging(listener)` / `offLogging(token)`

### Utility Methods

- `getTelemetrySnapshot()` - Get current state
- `encode(messageId, payload)` - Encode message
- `decode(raw)` - Decode message
- `onRaw(listener)` / `offRaw(token)` - Raw packets

## 🏗️ Architecture

### Clean Architecture

- **Transport Layer**: UDP/TCP socket management
- **Parser Layer**: MAVLink v2.0 message decoding
- **Core Layer**: State management & event emission
- **Bridge Layer**: C++ ↔ JavaScript integration
- **TypeScript Layer**: Type-safe API

### Key Design Patterns

- **PIMPL Pattern**: Hide implementation details
- **Smart Pointers**: Automatic memory management
- **Event-Driven**: Decoupled event handling
- **Thread-Safe**: Mutex-protected shared state

## 📦 Project Structure

```
react-native-mavlink/
├── cpp/                          # C++ implementations
│   ├── HybirdMAVLink.hpp/cpp    # Main bridge
│   ├── bridge/                   # Feature implementations
│   │   ├── transport/           # startUdp/startTcp
│   │   ├── commands/            # sendCommandLong/Int
│   │   ├── parameters/          # requestParams/setParam
│   │   ├── events/              # Event listeners (17 types)
│   │   ├── mission/             # Mission planning
│   │   └── logging/             # Log download
│   ├── core/                     # Core architecture
│   ├── parser/                   # MAVLink parser (13 msg types)
│   └── transport/                # Socket layer
├── src/                          # TypeScript
│   ├── index.ts                 # Main export
│   └── specs/MAVLink.nitro.ts  # Type definitions
├── android/                      # Android native
└── ios/                          # iOS native
```

## 🐛 Troubleshooting

### Connection Issues

```typescript
// Make sure ports are correct
// UDP: Local bind port (14550) != Remote send port (14551)
await MAVLink.startUdp({
  port: 14550, // RECEIVE on this port
  remoteHost: '192.168.1.100',
  remotePort: 14551, // SEND to this port
})
```

### No Events Received

```typescript
// Check if connection established first
MAVLink.onConnect(() => {
  console.log('Connected! Events should start flowing...')
})

// Verify heartbeat is received
MAVLink.onHeartbeat((hb) => {
  console.log('Heartbeat OK:', hb.systemId)
})
```

## 📝 Development

```bash
# Install dependencies
npm install

# Generate Nitrogen bindings
npm run specs

# Type checking
npm run typecheck

# Build Android
cd android && ./gradlew assembleDebug

# Build iOS
cd ios && pod install
```

## 🤝 Contributing

Contributions welcome! See architecture docs before making changes.

## 📄 License

MIT

## Usage

Clone repo và cập nhật các tên `$$*$$` theo `nitro.json`.

## Contributing

Contribute a change to this template to update it for newer React Native versions.

## Structure

- [`android/`](android): All your `android`-specific implementations.
  - [`build.gradle`](android/build.gradle): The gradle build file. This contains four important pieces:
    1. Standard react-native library boilerplate code
    2. Configures Kotlin (`apply plugin: 'org.jetbrains.kotlin.android'`)
    3. Adds all Nitrogen files (`apply from: '.../NitroMavlink+autolinking.gradle'`)
    4. Triggers the native C++ build (via CMake/`externalNativeBuild`)
  - [`CMakeLists.txt`](android/CMakeLists.txt): The CMake build file to build C++ code. This contains four important pieces:
    1. Creates a library called `NitroMavlink` (same as in `nitro.json`)
    2. Adds all Nitrogen files (`include(.../NitroMavlink+autolinking.cmake)`)
    3. Adds all custom C++ files (only `HybridTestObjectCpp.cpp`)
    4. Adds a `cpp-adapter.cpp` file, which autolinks all C++ HybridObjects (only `HybridTestObjectCpp`)
  - [`src/main/java/com/margelo/nitro/mavlink/`](android/src/main/java/com/margelo/nitro/mavlink/): All Kotlin implementations.
    - [`NitroMavlinkPackage.kt`](android/src/main/java/com/margelo/nitro/mavlink/NitroMavlinkPackage.kt): The react-native package. You need this because the react-native CLI only adds libraries if they have a `*Package.kt` file. In here, you can autolink all Kotlin HybridObjects.
- [`cpp/`](cpp): All your cross-platform implementations. (only `HybridTestObjectCpp.cpp`)
- [`ios/`](ios): All your iOS-specific implementations.
- [`nitrogen/`](nitrogen): All files generated by nitrogen. You should commit this folder to git.
- [`src/`](src): The TypeScript codebase. This defines all HybridObjects and loads them at runtime.
  - [`specs/`](src/specs): All HybridObject types. Nitrogen will run on all `*.nitro.ts` files.
- [`nitro.json`](nitro.json): The configuration file for nitrogen. This will define all native namespaces, as well as the library name.
- [`NitroMavlink.podspec`](NitroMavlink.podspec): The iOS podspec build file to build the iOS code. This contains three important pieces:
  1. Specifies the Pod's name. This must be identical to the name specified in `nitro.json`.
  2. Adds all of your `.swift` or `.cpp` files (implementations).
  3. Adds all Nitrogen files (`add_nitrogen_files(s)`)
- [`package.json`](package.json): The npm package.json file. `react-native-nitro-modules` should be a `peerDependency`.
