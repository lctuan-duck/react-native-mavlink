# Quick Start Guide - React Native MAVLink

Get up and running with React Native MAVLink in 5 minutes!

## Prerequisites

- Node.js 18+ and npm/yarn
- React Native development environment set up
- (Optional) ArduPilot SITL for testing without hardware

## Installation

```bash
# Create new React Native app (if you don't have one)
npx react-native init MyDroneApp
cd MyDroneApp

# Install react-native-mavlink
npm install react-native-mavlink
# or
yarn add react-native-mavlink

# iOS only
cd ios && pod install && cd ..
```

## Basic Usage

### 1. Import the Library

```typescript
import { mavlink, connectUDP, getTelemetry } from 'react-native-mavlink'
```

### 2. Connect to Your Drone

```typescript
// For SITL (simulator on localhost)
const connected = await connectUDP('127.0.0.1', 14550)

// For real drone via UDP telemetry radio
const connected = await connectUDP('192.168.1.100', 14550)

// For serial connection
import { connectSerial } from 'react-native-mavlink'
const connected = await connectSerial('/dev/ttyUSB0', 57600)
```

### 3. Get Telemetry

```typescript
// Get all telemetry at once
const telemetry = getTelemetry()

console.log(`Lat: ${telemetry.position.latitude}`)
console.log(`Lon: ${telemetry.position.longitude}`)
console.log(`Alt: ${telemetry.position.altitude}m`)
console.log(`Armed: ${telemetry.status.armed}`)
console.log(`Mode: ${telemetry.status.flightMode}`)
console.log(`Battery: ${telemetry.battery.remaining}%`)

// Or get individual values
const latitude = mavlink.getLatitude()
const armed = mavlink.isArmed()
```

### 4. Arm and Takeoff

```typescript
// First, switch to GUIDED mode
await mavlink.setFlightMode('GUIDED')

// Arm the vehicle
await mavlink.setArmed(true, false)

// Takeoff to 10 meters
await mavlink.guidedTakeoff(10)
```

### 5. Fly to Location

```typescript
// Fly to specific GPS coordinate
await mavlink.guidedGotoCoordinate({
  latitude: 47.3977,
  longitude: 8.5456,
  altitude: 50,
})
```

### 6. Land

```typescript
// Land at current position
await mavlink.guidedLand()

// Or return to launch
await mavlink.guidedRTL(false)
```

### 7. Disconnect

```typescript
await mavlink.disconnect()
```

## Complete Example

```typescript
import React, { useEffect, useState } from 'react'
import { View, Text, Button, Alert } from 'react-native'
import { mavlink, connectUDP, getTelemetry } from 'react-native-mavlink'

export default function DroneControl() {
  const [connected, setConnected] = useState(false)
  const [telemetry, setTelemetry] = useState(null)

  // Auto-update telemetry
  useEffect(() => {
    if (!connected) return

    const interval = setInterval(() => {
      if (mavlink.isConnected()) {
        setTelemetry(getTelemetry())
      }
    }, 100) // 10Hz

    return () => clearInterval(interval)
  }, [connected])

  const handleConnect = async () => {
    try {
      const success = await connectUDP('127.0.0.1', 14550)
      if (success) {
        setConnected(true)
        Alert.alert('Connected!')
      } else {
        Alert.alert('Connection failed')
      }
    } catch (error) {
      Alert.alert('Error', error.message)
    }
  }

  const handleTakeoff = async () => {
    try {
      // Switch to GUIDED
      await mavlink.setFlightMode('GUIDED')

      // Arm
      await mavlink.setArmed(true, false)

      // Takeoff
      const success = await mavlink.guidedTakeoff(10)
      Alert.alert(success ? 'Taking off!' : 'Takeoff failed')
    } catch (error) {
      Alert.alert('Error', error.message)
    }
  }

  const handleLand = async () => {
    try {
      const success = await mavlink.guidedLand()
      Alert.alert(success ? 'Landing!' : 'Land failed')
    } catch (error) {
      Alert.alert('Error', error.message)
    }
  }

  return (
    <View style={{ padding: 20 }}>
      <Text style={{ fontSize: 24, marginBottom: 20 }}>
        Drone Control
      </Text>

      {!connected ? (
        <Button title="Connect" onPress={handleConnect} />
      ) : (
        <>
          {telemetry && (
            <View style={{ marginBottom: 20 }}>
              <Text>Position: {telemetry.position.latitude.toFixed(6)}, {telemetry.position.longitude.toFixed(6)}</Text>
              <Text>Altitude: {telemetry.position.altitude.toFixed(2)}m</Text>
              <Text>Speed: {telemetry.velocity.groundSpeed.toFixed(1)}m/s</Text>
              <Text>Battery: {telemetry.battery.remaining.toFixed(0)}%</Text>
              <Text>Armed: {telemetry.status.armed ? 'YES' : 'NO'}</Text>
              <Text>Mode: {telemetry.status.flightMode}</Text>
            </View>
          )}

          <Button title="Takeoff" onPress={handleTakeoff} />
          <Button title="Land" onPress={handleLand} />
        </>
      )}
    </View>
  )
}
```

## Testing with SITL (No Drone Required!)

### Install ArduPilot SITL

```bash
# Clone ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Install prerequisites
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build copter SITL
./waf configure --board sitl
./waf copter
```

### Run SITL

```bash
cd ArduCopter
../Tools/autotest/sim_vehicle.py --console --map
```

This starts:

- MAVLink on UDP port 14550 (for your app)
- Console for monitoring
- Map for visualization

### Connect from Your App

```typescript
const connected = await connectUDP('127.0.0.1', 14550)
```

Now you can test all commands safely in simulation!

## Common Commands

### Check Connection

```typescript
const isConnected = mavlink.isConnected()
```

### Get System Info

```typescript
const systemId = mavlink.getSystemId()
const componentId = mavlink.getComponentId()
```

### Change Flight Mode

```typescript
await mavlink.setFlightMode('LOITER') // Hold position
await mavlink.setFlightMode('AUTO') // Follow mission
await mavlink.setFlightMode('RTL') // Return home
```

### Emergency Actions

```typescript
// Pause (hold position)
await mavlink.pauseVehicle()

// Emergency stop (force disarm)
await mavlink.emergencyStop()
```

### Camera Control

```typescript
// Take photo
await mavlink.triggerCamera()

// Start/stop video
await mavlink.startVideoRecording()
await mavlink.stopVideoRecording()
```

### Gimbal Control

```typescript
await mavlink.setGimbalAttitudeParams({
  pitch: -45, // Look down 45 degrees
  roll: 0,
  yaw: 0,
})
```

## Troubleshooting

### "Cannot connect"

- Check IP address and port
- Verify firewall settings
- Ensure drone is powered on and MAVLink is enabled

### "No telemetry"

- Wait 1-2 seconds after connection for HEARTBEAT
- Check if data streams are enabled on drone
- Try: `await mavlink.requestDataStreamParams({ streamId: 0, rateHz: 4 })`

### "Commands not working"

- Ensure drone is in GUIDED mode
- Check if GPS has lock (required for position commands)
- Verify drone is armed (for flight commands)

### Serial connection fails

- Check device permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate matches drone settings (usually 57600 or 115200)
- Check cable connection

## Next Steps

- [Full API Documentation](./README.md)
- [Implementation Details](./IMPLEMENTATION.md)
- [Example App](./example/App.tsx)
- [TODO List](./TODO.md)

## Need Help?

- 📧 Email: lctuan.dev@gmail.com
- 🐛 Issues: [GitHub Issues](https://github.com/lctuan-duck/react-native-mavlink/issues)
- 💬 Discord: [Join our community](#)

Happy flying! 🚁
