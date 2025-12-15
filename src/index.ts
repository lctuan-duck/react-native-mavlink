import { NitroModules } from 'react-native-nitro-modules'
import type { MAVLink as IMAVLink } from './specs/MAVLink.nitro'

/**
 * MAVLink Hybrid Object Instance
 * 
 * This is the main entry point for using MAVLink in React Native.
 * All methods return Promises for async operations.
 * 
 * @example
 * ```typescript
 * import { MAVLink } from 'react-native-mavlink'
 * 
 * // Start UDP connection
 * // - host/port: Local bind address to RECEIVE MAVLink packets
 * // - remoteHost/remotePort: Drone address to SEND commands to
 * await MAVLink.startUdp({ 
 *   port: 14550,
 *   host: '0.0.0.0',  // Optional, default '0.0.0.0'
 *   remoteHost: '192.168.1.100',  // Optional, drone IP
 *   remotePort: 14551  // Optional, drone port
 * })
 * 
 * // Or TCP connection to drone/server
 * await MAVLink.startTcp({
 *   host: '192.168.1.100',  // Drone/server address
 *   port: 5760  // Drone/server port
 * })
 * 
 * // Listen for telemetry
 * const token = MAVLink.onTelemetry((data) => {
 *   console.log('Altitude:', data.altitude)
 * })
 * 
 * // Send command (COMMAND_LONG) - ARM the drone
 * await MAVLink.sendCommandLong({
 *   command: 400,      // MAV_CMD_COMPONENT_ARM_DISARM
 *   param1: 1,         // 1 = Arm, 0 = Disarm
 *   targetSystem: 1,
 *   targetComponent: 1
 * })
 * 
 * // Send position command (COMMAND_INT) - Go to location
 * await MAVLink.sendCommandInt({
 *   command: 192,                  // MAV_CMD_DO_REPOSITION
 *   x: 47.3977419 * 1e7,          // Latitude in degrees * 1e7
 *   y: 8.5455935 * 1e7,           // Longitude in degrees * 1e7
 *   z: 100,                       // Altitude in meters
 *   frame: 3,                     // MAV_FRAME_GLOBAL_RELATIVE_ALT (optional, default)
 *   param4: -1                    // Yaw (optional)
 * })
 * ```
 */
export const MAVLink = NitroModules.createHybridObject<IMAVLink>('MAVLink')

// Re-export types for convenience
export type {
  MAVLink as MAVLinkType,
  UdpOptions,
  TcpOptions,
  HeartbeatEvent,
  StatusEvent,
  TelemetryEvent,
  GpsEvent,
  AttitudeEvent,
  BatteryEvent,
  Attitude,
  VfrHud,
  GPSInfo,
  LocalNed,
  BatteryInfo,
  StatusText,
  SensorStatus,
  LoggingEvent,
  ModeEvent,
  ArmEvent,
  AckEvent,
  ParameterEvent,
  ByteArray,
  DecodedMessage,
  CommandLongArgs,
  CommandIntArgs,
  MissionCountEvent,
  MissionRequestEvent,
  MissionItemInt,
  MissionItemEvent,
  MissionAckEvent,
  CameraStatusEvent,
  CameraCaptureStatusEvent,
  GimbalDeviceAttitudeStatusEvent,
  LogEntry,
  DataTransmissionHandshake,
  LogDataChunk
} from './specs/MAVLink.nitro'