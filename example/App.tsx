/**
 * Example React Native App using MAVLink
 * Demonstrates basic usage of the library
 */

import React, { useEffect, useState } from 'react'
import { View, Text, Button, StyleSheet, ScrollView } from 'react-native'
import { mavlink, connectUDP, getTelemetry } from 'react-native-mavlink'

export default function App() {
  const [connected, setConnected] = useState(false)
  const [telemetry, setTelemetry] = useState<any>(null)
  const [status, setStatus] = useState('Disconnected')

  // Connect to vehicle
  const handleConnect = async () => {
    try {
      setStatus('Connecting...')
      // Android Emulator: Use 10.0.2.2 to access Windows host
      // iOS Simulator: Use 127.0.0.1 (direct localhost)
      // Real Device: Use Windows IP (e.g., 192.168.1.100)
      const success = await connectUDP('10.0.2.2', 14550)
      if (success) {
        setConnected(true)
        setStatus('Connected')
        startTelemetryUpdate()
      } else {
        setStatus('Connection failed')
      }
    } catch (error) {
      setStatus(`Error: ${error}`)
    }
  }

  // Start telemetry updates
  const startTelemetryUpdate = () => {
    const interval = setInterval(() => {
      if (mavlink.isConnected()) {
        const data = getTelemetry()
        setTelemetry(data)
      } else {
        clearInterval(interval)
        setConnected(false)
        setStatus('Disconnected')
      }
    }, 100) // 10Hz update
  }

  // Arm vehicle
  const handleArm = async () => {
    try {
      const success = await mavlink.setArmed(true, false)
      setStatus(success ? 'Armed' : 'Arm failed')
    } catch (error) {
      setStatus(`Arm error: ${error}`)
    }
  }

  // Disarm vehicle
  const handleDisarm = async () => {
    try {
      const success = await mavlink.setArmed(false, false)
      setStatus(success ? 'Disarmed' : 'Disarm failed')
    } catch (error) {
      setStatus(`Disarm error: ${error}`)
    }
  }

  // Takeoff
  const handleTakeoff = async () => {
    try {
      // First switch to GUIDED mode
      await mavlink.setFlightMode('GUIDED')
      // Then takeoff to 10m
      const success = await mavlink.guidedTakeoff(10)
      setStatus(success ? 'Taking off' : 'Takeoff failed')
    } catch (error) {
      setStatus(`Takeoff error: ${error}`)
    }
  }

  // Land
  const handleLand = async () => {
    try {
      const success = await mavlink.guidedLand()
      setStatus(success ? 'Landing' : 'Land failed')
    } catch (error) {
      setStatus(`Land error: ${error}`)
    }
  }

  // RTL
  const handleRTL = async () => {
    try {
      const success = await mavlink.guidedRTL(false)
      setStatus(success ? 'Returning home' : 'RTL failed')
    } catch (error) {
      setStatus(`RTL error: ${error}`)
    }
  }

  // Go to coordinate
  const handleGoto = async () => {
    try {
      const success = await mavlink.guidedGotoCoordinate({
        latitude: 47.3977,
        longitude: 8.5456,
        altitude: 50
      })
      setStatus(success ? 'Going to waypoint' : 'Goto failed')
    } catch (error) {
      setStatus(`Goto error: ${error}`)
    }
  }

  // Disconnect
  const handleDisconnect = async () => {
    await mavlink.disconnect()
    setConnected(false)
    setTelemetry(null)
    setStatus('Disconnected')
  }

  return (
    <View style={styles.container}>
      <Text style={styles.title}>React Native MAVLink Demo</Text>
      <Text style={styles.status}>Status: {status}</Text>

      {!connected ? (
        <Button title="Connect to Vehicle" onPress={handleConnect} />
      ) : (
        <ScrollView style={styles.content}>
          {/* Telemetry Display */}
          {telemetry && (
            <View style={styles.section}>
              <Text style={styles.sectionTitle}>Telemetry</Text>

              <Text>Position:</Text>
              <Text>  Lat: {telemetry.position.latitude.toFixed(6)}°</Text>
              <Text>  Lon: {telemetry.position.longitude.toFixed(6)}°</Text>
              <Text>  Alt: {telemetry.position.altitude.toFixed(2)}m</Text>
              <Text>  Hdg: {telemetry.position.heading.toFixed(1)}°</Text>

              <Text style={styles.spacer}>Velocity:</Text>
              <Text>  Ground: {telemetry.velocity.groundSpeed.toFixed(1)} m/s</Text>
              <Text>  Air: {telemetry.velocity.airSpeed.toFixed(1)} m/s</Text>
              <Text>  Climb: {telemetry.velocity.climbRate.toFixed(1)} m/s</Text>

              <Text style={styles.spacer}>Attitude:</Text>
              <Text>  Roll: {telemetry.attitude.roll.toFixed(1)}°</Text>
              <Text>  Pitch: {telemetry.attitude.pitch.toFixed(1)}°</Text>
              <Text>  Yaw: {telemetry.attitude.yaw.toFixed(1)}°</Text>

              <Text style={styles.spacer}>Battery:</Text>
              <Text>  Voltage: {telemetry.battery.voltage.toFixed(2)}V</Text>
              <Text>  Remaining: {telemetry.battery.remaining.toFixed(0)}%</Text>

              <Text style={styles.spacer}>GPS:</Text>
              <Text>  Fix: {telemetry.gps.fixType}</Text>
              <Text>  Sats: {telemetry.gps.satellites}</Text>

              <Text style={styles.spacer}>Status:</Text>
              <Text>  Armed: {telemetry.status.armed ? 'YES' : 'NO'}</Text>
              <Text>  Flying: {telemetry.status.flying ? 'YES' : 'NO'}</Text>
              <Text>  Mode: {telemetry.status.flightMode}</Text>
            </View>
          )}

          {/* Control Buttons */}
          <View style={styles.section}>
            <Text style={styles.sectionTitle}>Vehicle Control</Text>

            <View style={styles.buttonRow}>
              <Button title="Arm" onPress={handleArm} />
              <Button title="Disarm" onPress={handleDisarm} />
            </View>

            <View style={styles.buttonRow}>
              <Button title="Takeoff (10m)" onPress={handleTakeoff} />
              <Button title="Land" onPress={handleLand} />
            </View>

            <View style={styles.buttonRow}>
              <Button title="RTL" onPress={handleRTL} />
              <Button title="Goto Waypoint" onPress={handleGoto} />
            </View>

            <View style={styles.buttonRow}>
              <Button title="Disconnect" onPress={handleDisconnect} color="red" />
            </View>
          </View>
        </ScrollView>
      )}
    </View>
  )
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    padding: 20,
    backgroundColor: '#f5f5f5'
  },
  title: {
    fontSize: 24,
    fontWeight: 'bold',
    marginTop: 40,
    marginBottom: 10,
    textAlign: 'center'
  },
  status: {
    fontSize: 16,
    marginBottom: 20,
    textAlign: 'center',
    color: '#666'
  },
  content: {
    flex: 1
  },
  section: {
    backgroundColor: 'white',
    padding: 15,
    marginBottom: 15,
    borderRadius: 8,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 3
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: 'bold',
    marginBottom: 10,
    color: '#333'
  },
  spacer: {
    marginTop: 10
  },
  buttonRow: {
    flexDirection: 'row',
    justifyContent: 'space-around',
    marginVertical: 5
  }
})
