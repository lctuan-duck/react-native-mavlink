/**
 * React Native MAVLink Module
 * Main entry point for MAVLink integration
 */

import { NitroModules } from 'react-native-nitro-modules'
import type { MAVLink } from './specs/MAVLink.nitro'

// Export types
export * from './specs/MAVLink.nitro'

// Create and export MAVLink instance
export const mavlink = NitroModules.createHybridObject<MAVLink>('MAVLink')

// Export default
export default mavlink

// Helper functions for common operations
export const connectUDP = async (
  address: string,
  port?: number,
  autoReconnect: boolean = true,
  maxReconnectAttempts: number = 3,
  reconnectDelayMs: number = 5000
) => {
  return mavlink.connectWithConfig({
    type: 1, // UDP
    address,
    port: port ?? 14550,
    baudRate: 0,
    autoReconnect,
    maxReconnectAttempts,
    reconnectDelayMs,
  })
}

export const connectSerial = async (
  port: string,
  baudRate: number = 57600,
  autoReconnect: boolean = true,
  maxReconnectAttempts: number = 3,
  reconnectDelayMs: number = 5000
) => {
  return mavlink.connectWithConfig({
    type: 0, // SERIAL
    address: port,
    port: 0,
    baudRate,
    autoReconnect,
    maxReconnectAttempts,
    reconnectDelayMs,
  })
}

export const connectTCP = async (
  address: string,
  port?: number,
  autoReconnect: boolean = true,
  maxReconnectAttempts: number = 3,
  reconnectDelayMs: number = 5000
) => {
  return mavlink.connectWithConfig({
    type: 2, // TCP
    address,
    port: port ?? 5760,
    baudRate: 0,
    autoReconnect,
    maxReconnectAttempts,
    reconnectDelayMs,
  })
}

// Telemetry helper
export const getTelemetry = () => {
  return {
    position: {
      latitude: mavlink.getLatitude(),
      longitude: mavlink.getLongitude(),
      altitude: mavlink.getAltitude(),
      heading: mavlink.getHeading()
    },
    velocity: {
      groundSpeed: mavlink.getGroundSpeed(),
      airSpeed: mavlink.getAirSpeed(),
      climbRate: mavlink.getClimbRate()
    },
    attitude: {
      roll: mavlink.getRoll(),
      pitch: mavlink.getPitch(),
      yaw: mavlink.getYaw()
    },
    battery: {
      voltage: mavlink.getBatteryVoltage(0),
      remaining: mavlink.getBatteryRemaining(0)
    },
    gps: {
      fixType: mavlink.getGPSFixType(),
      satellites: mavlink.getGPSSatelliteCount()
    },
    status: {
      armed: mavlink.isArmed(),
      flying: mavlink.isFlying(),
      flightMode: mavlink.getFlightMode(),
      connected: mavlink.isConnected(),
      heartbeatTimeout: mavlink.isHeartbeatTimeout(),
      timeSinceLastHeartbeat: mavlink.getTimeSinceLastHeartbeat()
    }
  }
}

/**
 * Monitor connection health and detect heartbeat timeout
 * detects if no HEARTBEAT received for > 3.5s
 */
export function monitorConnectionHealth(
  callback: (status: {
    connected: boolean
    heartbeatTimeout: boolean
    timeSinceLastHeartbeat: number
  }) => void,
  checkIntervalMs: number = 1000
): () => void {
  const interval = setInterval(() => {
    const connected = mavlink.isConnected()
    const heartbeatTimeout = mavlink.isHeartbeatTimeout()
    const timeSinceLastHeartbeat = mavlink.getTimeSinceLastHeartbeat()

    callback({
      connected,
      heartbeatTimeout,
      timeSinceLastHeartbeat,
    })
  }, checkIntervalMs)

  return () => clearInterval(interval)
}