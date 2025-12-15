import type { HybridObject } from "react-native-nitro-modules"

export type UdpOptions = {
  host?: string // Local bind address (default '0.0.0.0')
  port: number  // Local bind port to receive MAVLink packets
  remoteHost?: string // Remote drone address to send commands to
  remotePort?: number // Remote drone port to send commands to
  heartbeatTimeoutMs?: number
}

export type TcpOptions = {
  host: string // Remote server address to connect to
  port: number // Remote server port to connect to
  heartbeatTimeoutMs?: number
}

export type HeartbeatEvent = {
  systemId: number
  componentId: number
  autopilot: number
  type: number
  baseMode: number
  customMode: number
  systemStatus: number
  timestampMs: number
}

export type BatteryInfo = { voltage: number; current?: number; remaining?: number }
export type StatusText = { severity: number; message: string }
export type SensorStatus = { name: string; ok: boolean }
export type StatusEvent = { battery?: BatteryInfo; cpuLoad?: number; sensors?: SensorStatus[]; text?: StatusText; timestampMs: number }

// Separate detailed events
export type GpsEvent = { latitude: number; longitude: number; altitude: number; fixType: number; satellitesVisible: number; timestampMs: number }
export type AttitudeEvent = { roll: number; pitch: number; yaw: number; rollspeed: number; pitchspeed: number; yawspeed: number; timestampMs: number }
export type BatteryEvent = { voltage: number; current: number; remaining: number; timestampMs: number }

// Aggregated telemetry
export type Attitude = { roll: number; pitch: number; yaw: number }
export type VfrHud = { groundspeed: number; throttle: number; alt: number }
export type GPSInfo = { latE7: number; lonE7: number; altMm?: number; satellites?: number; fixType?: number }
export type LocalNed = { x: number; y: number; z: number }
export type TelemetryEvent = { attitude?: Attitude; vfrHud?: VfrHud; gps?: GPSInfo; localNed?: LocalNed; timestampMs: number }

// Logging & Data Stream
export type LogEntry = { id: number; numLogs: number; lastLogNum: number; timeUtc: number; size: number }
export type DataTransmissionHandshake = { size: number; packets: number; payload: number; jpgQuality: number }
export type LogDataChunk = { id: number; ofs: number; data: ByteArray }

export type LoggingEvent = {
  entry?: LogEntry
  handshake?: DataTransmissionHandshake
  data?: LogDataChunk
  timestampMs: number
}

export type ModeEvent = { baseMode: number; customMode: number; flightMode?: string; timestampMs: number }
export type ArmEvent = { armed: boolean; timestampMs: number }

export type AckEvent = {
  command: number
  result: number
  timestampMs: number
}

export type ParameterEvent = {
  name: string
  value: number | string
  type: number
  index?: number
}

export type ByteArray = ArrayBuffer
export type DecodedMessage = { messageId: number; systemId: number; componentId: number; payload: ByteArray }

// Command args types
export type CommandLongArgs = {
  command: number
  param1?: number
  param2?: number
  param3?: number
  param4?: number
  param5?: number
  param6?: number
  param7?: number
  targetSystem?: number
  targetComponent?: number
}
export type CommandIntArgs = {
  command: number
  param1?: number
  param2?: number
  param3?: number
  param4?: number
  x: number          // Latitude (degrees * 1e7) or local x position
  y: number          // Longitude (degrees * 1e7) or local y position
  z: number          // Altitude (meters) or local z position
  frame?: number     // MAV_FRAME (default: MAV_FRAME_GLOBAL_RELATIVE_ALT = 3)
  targetSystem?: number
  targetComponent?: number
}

// Logging args types
export type LogDataRequestArgs = {
  id: number
  ofs: number
  count: number
}
export type DataTransmissionHandshakeArgs = {
  size: number
  packets: number
  payload: number
  jpgQuality: number
}

// Mission types
export type MissionCountEvent = { targetSystem: number; targetComponent: number; count: number; timestampMs: number }
export type MissionRequestEvent = { targetSystem: number; targetComponent: number; seq: number; timestampMs: number }
export type MissionItemInt = {
  seq: number
  frame: number
  command: number
  current: number
  autocontinue: number
  param1?: number
  param2?: number
  param3?: number
  param4?: number
  x: number
  y: number
  z: number
}
export type MissionItemEvent = { item: MissionItemInt; timestampMs: number }
export type MissionAckEvent = { type: number; timestampMs: number }

// Camera / Gimbal
export type CameraStatusEvent = { status: number; storageId?: number; recordingTimeMs?: number; imageInterval?: number; timestampMs: number }
export type CameraCaptureStatusEvent = { imageStatus: number; videoStatus: number; imageInterval?: number; recordingTimeMs?: number; timestampMs: number }
export type GimbalDeviceAttitudeStatusEvent = { q: [number, number, number, number]; targetSystem?: number; targetComponent?: number; timestampMs: number }

export interface MAVLink extends HybridObject<{ ios: 'c++', android: 'c++' }> {
  // Transport
  startUdp(options: UdpOptions): Promise<void>
  stopUdp(): Promise<void>
  startTcp(options: TcpOptions): Promise<void>
  stopTcp(): Promise<void>

  // Encode/Decode
  encode(messageId: number, payload: ByteArray): Promise<ByteArray>
  decode(raw: ByteArray): Promise<DecodedMessage>

  // Commands / Params
  sendCommandLong(args: CommandLongArgs): Promise<void>
  sendCommandInt(args: CommandIntArgs): Promise<void>
  requestParams(): Promise<void>
  setParam(name: string, value: number | string): Promise<void>
  // Mission commands
  requestMissionList(): Promise<void>
  sendMissionCount(count: number): Promise<void>
  sendMissionItemInt(item: MissionItemInt): Promise<void>
  clearAllMissions(): Promise<void>
  setCurrentMission(seq: number): Promise<void>
  // Mission auto-upload
  setMissionUpload(items: MissionItemInt[]): Promise<void>
  enableAutoMissionUpload(enable: boolean): Promise<void>

  // Logging helpers
  requestLogList(): Promise<void>
  requestLogData(args: LogDataRequestArgs): Promise<void>
  requestDataTransmissionHandshake(args: DataTransmissionHandshakeArgs): Promise<void>

  // Event subscriptions
  onRaw(listener: (raw: ByteArray) => void): number
  offRaw(token: number): void

  onHeartbeat(listener: (ev: HeartbeatEvent) => void): number
  offHeartbeat(token: number): void

  onStatus(listener: (ev: StatusEvent) => void): number
  offStatus(token: number): void

  onTelemetry(listener: (ev: TelemetryEvent) => void): number
  offTelemetry(token: number): void

  onGps(listener: (ev: GpsEvent) => void): number
  offGps(token: number): void

  onAttitude(listener: (ev: AttitudeEvent) => void): number
  offAttitude(token: number): void

  onBattery(listener: (ev: BatteryEvent) => void): number
  offBattery(token: number): void

  onAck(listener: (ev: AckEvent) => void): number
  offAck(token: number): void

  onParameter(listener: (ev: ParameterEvent) => void): number
  offParameter(token: number): void

  onMode(listener: (ev: ModeEvent) => void): number
  offMode(token: number): void

  onArm(listener: (ev: ArmEvent) => void): number
  offArm(token: number): void

  onConnect(listener: () => void): number
  offConnect(token: number): void
  onDisconnect(listener: (reason?: string) => void): number
  offDisconnect(token: number): void

  // Mission subscriptions
  onMissionCount(listener: (ev: MissionCountEvent) => void): number
  offMissionCount(token: number): void
  onMissionRequest(listener: (ev: MissionRequestEvent) => void): number
  offMissionRequest(token: number): void
  onMissionItem(listener: (ev: MissionItemEvent) => void): number
  offMissionItem(token: number): void
  onMissionAck(listener: (ev: MissionAckEvent) => void): number
  offMissionAck(token: number): void

  // Logging subscriptions
  onLogging(listener: (ev: LoggingEvent) => void): number
  offLogging(token: number): void

  // Camera/Gimbal subscriptions
  onCameraStatus(listener: (ev: CameraStatusEvent) => void): number
  offCameraStatus(token: number): void
  onCameraCaptureStatus(listener: (ev: CameraCaptureStatusEvent) => void): number
  offCameraCaptureStatus(token: number): void
  onGimbalDeviceAttitudeStatus(listener: (ev: GimbalDeviceAttitudeStatusEvent) => void): number
  offGimbalDeviceAttitudeStatus(token: number): void

  // Utilities
  getTelemetrySnapshot(): Promise<TelemetryEvent | null>
}
