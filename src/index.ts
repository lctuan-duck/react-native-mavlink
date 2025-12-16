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
export const connectUDP = async (address: string, port: number) => {
  return mavlink.connectWithConfig({
    type: 1, // UDP
    address,
    port,
    baudRate: 0
  })
}

export const connectSerial = async (port: string, baudRate: number = 57600) => {
  return mavlink.connectWithConfig({
    type: 0, // SERIAL
    address: port,
    port: 0,
    baudRate
  })
}

export const connectTCP = async (address: string, port: number) => {
  return mavlink.connectWithConfig({
    type: 2, // TCP
    address,
    port,
    baudRate: 0
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
      connected: mavlink.isConnected()
    }
  }
}