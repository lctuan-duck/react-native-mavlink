# React Native MAVLink

A comprehensive MAVLink library for React Native with full TypeScript support, enabling drone/UAV control and telemetry monitoring.

## Features

✅ **Complete MAVLink v2.0 Protocol Support**
- UDP, TCP, and Serial connections
- Thread-safe message handling
- Automatic retry and acknowledgment

✅ **Vehicle Control**
- Arm/Disarm
- Flight mode changes
- Guided commands (Takeoff, Land, RTL, Goto)
- Manual control input

✅ **Real-time Telemetry**
- Position (GPS)
- Attitude (Roll, Pitch, Yaw)
- Velocity (Ground, Air, Climb)
- Battery status
- Flight status

✅ **Mission Management**
- Start/Stop missions
- Set current waypoint
- Clear missions

✅ **Advanced Features**
- Parameter get/set
- Camera trigger
- Gimbal control
- Data stream requests

## Installation

```bash
npm install react-native-mavlink
# or
yarn add react-native-mavlink
```

### iOS

```bash
cd ios && pod install
```

### Android

No additional steps required.

## Quick Start

### 1. Connect to Vehicle

```typescript
import { mavlink, connectUDP, getTelemetry } from 'react-native-mavlink'

// Connect via UDP (default for SITL)
const connected = await connectUDP('127.0.0.1', 14550)

// Or connect via Serial
import { connectSerial } from 'react-native-mavlink'
const connected = await connectSerial('/dev/ttyUSB0', 57600)

// Or connect via TCP
import { connectTCP } from 'react-native-mavlink'
const connected = await connectTCP('192.168.1.100', 5760)
```

### 2. Get Telemetry

```typescript
// Get all telemetry at once
const telemetry = getTelemetry()
console.log(telemetry.position.latitude)
console.log(telemetry.attitude.roll)

// Or get individual values
const lat = mavlink.getLatitude()
const armed = mavlink.isArmed()
const mode = mavlink.getFlightMode()
```

### 3. Control Vehicle

```typescript
// Arm
await mavlink.setArmed(true, false)

// Switch to GUIDED mode
await mavlink.setFlightMode('GUIDED')

// Takeoff to 10 meters
await mavlink.guidedTakeoff(10)

// Goto coordinate
await mavlink.guidedGotoCoordinate({
  latitude: 47.3977,
  longitude: 8.5456,
  altitude: 50
})

// Land
await mavlink.guidedLand()

// Return to launch
await mavlink.guidedRTL(false)

// Disarm
await mavlink.setArmed(false, false)
```

### 4. Real-time Updates

```typescript
import { useEffect, useState } from 'react'

function DroneStatus() {
  const [telemetry, setTelemetry] = useState(null)

  useEffect(() => {
    // Update at 10Hz
    const interval = setInterval(() => {
      if (mavlink.isConnected()) {
        setTelemetry(getTelemetry())
      }
    }, 100)

    return () => clearInterval(interval)
  }, [])

  return (
    <View>
      <Text>Altitude: {telemetry?.position.altitude}m</Text>
      <Text>Speed: {telemetry?.velocity.groundSpeed}m/s</Text>
      <Text>Battery: {telemetry?.battery.remaining}%</Text>
    </View>
  )
}
```

## API Reference

### Connection

#### `connectWithConfig(config: ConnectionConfig): Promise<boolean>`

Connect to vehicle with custom configuration.

```typescript
const config = {
  type: 1, // 0=SERIAL, 1=UDP, 2=TCP
  address: '127.0.0.1',
  port: 14550,
  baudRate: 57600 // Only for serial
}
await mavlink.connectWithConfig(config)
```

#### `disconnect(): Promise<void>`

Disconnect from vehicle.

#### `isConnected(): boolean`

Check if connected to vehicle.

### Telemetry Getters

All getters are synchronous and return current cached values:

- `getLatitude(): number` - GPS latitude (degrees)
- `getLongitude(): number` - GPS longitude (degrees)
- `getAltitude(): number` - Altitude AMSL (meters)
- `getHeading(): number` - Heading (0-360 degrees)
- `getGroundSpeed(): number` - Ground speed (m/s)
- `getAirSpeed(): number` - Air speed (m/s)
- `getClimbRate(): number` - Vertical speed (m/s)
- `getRoll(): number` - Roll angle (degrees)
- `getPitch(): number` - Pitch angle (degrees)
- `getYaw(): number` - Yaw angle (degrees)
- `getBatteryVoltage(id: number): number` - Battery voltage (V)
- `getBatteryRemaining(id: number): number` - Battery remaining (%)
- `getGPSFixType(): number` - GPS fix type (0-6)
- `getGPSSatelliteCount(): number` - Number of satellites
- `isArmed(): boolean` - Armed status
- `isFlying(): boolean` - Flying status
- `getFlightMode(): string` - Current flight mode
- `getSystemId(): number` - MAVLink system ID
- `getComponentId(): number` - MAVLink component ID

### Vehicle Control

#### `setArmed(arm: boolean, force: boolean): Promise<boolean>`

Arm or disarm vehicle.

```typescript
await mavlink.setArmed(true, false) // Arm normally
await mavlink.setArmed(false, true) // Force disarm
```

#### `setFlightMode(mode: string): Promise<boolean>`

Change flight mode.

```typescript
await mavlink.setFlightMode('GUIDED')
await mavlink.setFlightMode('AUTO')
await mavlink.setFlightMode('RTL')
```

Supported modes: `STABILIZE`, `ACRO`, `ALT_HOLD`, `AUTO`, `GUIDED`, `LOITER`, `RTL`, `LAND`, `POSHOLD`

### Guided Commands

#### `guidedTakeoff(altitude: number): Promise<boolean>`

Takeoff to specified altitude (meters).

#### `guidedLand(): Promise<boolean>`

Land at current position.

#### `guidedRTL(smartRTL: boolean): Promise<boolean>`

Return to launch. If `smartRTL` is true, uses smart RTL (ArduCopter).

#### `guidedGotoCoordinate(coordinate: Coordinate): Promise<boolean>`

Fly to specified GPS coordinate.

```typescript
await mavlink.guidedGotoCoordinate({
  latitude: 47.3977,
  longitude: 8.5456,
  altitude: 50
})
```

#### `guidedChangeAltitude(altitudeChange: number): Promise<boolean>`

Change altitude by relative amount (meters).

#### `guidedChangeHeading(heading: number): Promise<boolean>`

Change heading (0-360 degrees).

#### `guidedOrbitParams(params: OrbitParams): Promise<boolean>`

Orbit around a point.

```typescript
await mavlink.guidedOrbitParams({
  center: { latitude: 47.3977, longitude: 8.5456, altitude: 50 },
  radius: 20, // meters
  velocity: 2, // m/s
  yawBehavior: 0 // 0=front to center, 1=hold initial
})
```

#### `guidedROICoordinate(coordinate: Coordinate): Promise<boolean>`

Point camera at coordinate (Region of Interest).

#### `guidedClearROI(): Promise<boolean>`

Clear region of interest.

#### `pauseVehicle(): Promise<boolean>`

Pause vehicle (switch to LOITER mode).

#### `emergencyStop(): Promise<boolean>`

Emergency stop (force disarm).

### Mission Management

#### `startMission(): Promise<boolean>`

Start current mission.

#### `setCurrentMissionItem(sequence: number): Promise<boolean>`

Set current mission waypoint.

#### `getCurrentMissionItem(): number`

Get current mission waypoint number.

#### `clearMission(): Promise<boolean>`

Clear all mission waypoints.

### Parameters

#### `getParameter(name: string): Promise<number>`

Get parameter value (not yet implemented).

#### `setParameter(name: string, value: number): Promise<boolean>`

Set parameter value.

```typescript
await mavlink.setParameter('RTL_ALT', 50)
```

#### `refreshParameters(): Promise<boolean>`

Request full parameter list from vehicle.

### Camera & Gimbal

#### `triggerCamera(): Promise<boolean>`

Trigger camera shutter.

#### `startVideoRecording(): Promise<boolean>`

Start video recording.

#### `stopVideoRecording(): Promise<boolean>`

Stop video recording.

#### `setGimbalAttitudeParams(attitude: GimbalAttitude): Promise<boolean>`

Control gimbal attitude.

```typescript
await mavlink.setGimbalAttitudeParams({
  pitch: -45, // degrees
  roll: 0,
  yaw: 0
})
```

### Manual Control

#### `sendManualControlInput(input: ManualControlInput): void`

Send manual control input (for RC override).

```typescript
mavlink.sendManualControlInput({
  x: 0.5,  // pitch (-1 to 1)
  y: 0.0,  // roll
  z: 0.7,  // throttle
  r: 0.0,  // yaw
  buttons: 0
})
```

### Advanced

#### `rebootAutopilot(): Promise<boolean>`

Reboot the autopilot.

#### `requestDataStreamParams(request: DataStreamRequest): Promise<boolean>`

Request specific data stream at custom rate.

```typescript
await mavlink.requestDataStreamParams({
  streamId: 0, // MAV_DATA_STREAM_ALL
  rateHz: 10   // 10Hz
})
```

#### `sendCommandParams(params: CommandParams): Promise<boolean>`

Send raw MAVLink command.

```typescript
await mavlink.sendCommandParams({
  command: 400, // MAV_CMD_COMPONENT_ARM_DISARM
  param1: 1,
  param2: 0,
  param3: 0,
  param4: 0,
  param5: 0,
  param6: 0,
  param7: 0
})
```

## Testing with SITL

To test without real hardware, use ArduPilot SITL:

```bash
# Install ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Build copter SITL
./waf configure --board sitl
./waf copter

# Run SITL
cd ArduCopter
../Tools/autotest/sim_vehicle.py --console --map

# In your React Native app
await connectUDP('127.0.0.1', 14550)
```

## Architecture

```
┌─────────────────────────────────────┐
│      React Native (TypeScript)      │
│     mavlink.guidedTakeoff(10)       │
└──────────────┬──────────────────────┘
               │ Nitro Bridge
┌──────────────▼──────────────────────┐
│        HybridMAVLink (C++)          │
│  ┌─────────────────────────────┐   │
│  │    ConnectionManager        │   │
│  │  - UDP/TCP/Serial           │   │
│  │  - Send/Receive threads     │   │
│  └─────────────────────────────┘   │
│  ┌─────────────────────────────┐   │
│  │      VehicleState           │   │
│  │  - Thread-safe telemetry    │   │
│  │  - Message handlers         │   │
│  └─────────────────────────────┘   │
│  ┌─────────────────────────────┐   │
│  │    CommandExecutor          │   │
│  │  - Retry logic              │   │
│  │  - ACK handling             │   │
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
               │
┌──────────────▼──────────────────────┐
│         MAVLink v2.0 Protocol       │
│      (UDP/TCP/Serial Transport)     │
└─────────────────────────────────────┘
               │
┌──────────────▼──────────────────────┐
│    ArduPilot / PX4 Autopilot        │
└─────────────────────────────────────┘
```

## Troubleshooting

### Connection Issues

**Problem**: Cannot connect to vehicle

**Solutions**:
- Check IP address and port (default SITL: `127.0.0.1:14550`)
- Verify firewall settings
- For serial, check permissions: `sudo chmod 666 /dev/ttyUSB0`
- Enable MAVLink output in autopilot parameters

### No Telemetry Updates

**Problem**: `mavlink.isConnected()` returns true but telemetry is 0

**Solutions**:
- Wait for HEARTBEAT message (up to 1 second)
- Request data streams: `await mavlink.requestDataStreamParams({ streamId: 0, rateHz: 4 })`
- Check if vehicle is sending messages (use MAVLink inspector)

### Commands Not Working

**Problem**: Commands return true but nothing happens

**Solutions**:
- Ensure vehicle is in correct mode (e.g., GUIDED for goto commands)
- Check if vehicle is armed
- Verify GPS lock (required for position commands)
- Check autopilot logs for errors

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

MIT

## Credits

Built with:
- [React Native Nitro Modules](https://github.com/mrousavy/nitro)
- [MAVLink Protocol](https://mavlink.io/)
- Inspired by [QGroundControl](https://github.com/mavlink/qgroundcontrol)

## Support

- 📧 Email: support@example.com
- 🐛 Issues: [GitHub Issues](https://github.com/yourname/react-native-mavlink/issues)
- 📖 Docs: [Full Documentation](https://yourname.github.io/react-native-mavlink)
