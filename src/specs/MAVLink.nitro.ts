import type { HybridObject } from "react-native-nitro-modules"

// ============================================================================
// TYPE DEFINITIONS
// ============================================================================

/**
 * Geographic coordinate
 */
export interface Coordinate {
  latitude: number;
  longitude: number;
  altitude: number;
}

/**
 * Connection configuration
 */
export interface ConnectionConfig {
  type: number; // 0=SERIAL, 1=UDP, 2=TCP
  address: string;
  port: number;
  baudRate: number;
  autoReconnect?: boolean; // Enable auto-reconnect on connection loss (default: false)
  maxReconnectAttempts?: number; // Max reconnect attempts, 0 = infinite (default: 0)
  reconnectDelayMs?: number; // Initial reconnect delay in ms (default: 5000)
}

/**
 * Manual control input
 */
export interface ManualControlInput {
  roll: number; // -1000 to 1000
  pitch: number; // -1000 to 1000
  yaw: number; // -1000 to 1000
  throttle: number; // 0 to 1000
}

/**
 * Orbit command parameters
 */
export interface OrbitParams {
  latitude: number;
  longitude: number;
  radius: number;
  velocity: number; // negative for counter-clockwise
}

/**
 * MAVLink command parameters
 */
export interface CommandParams {
  command: number;
  param1: number;
  param2: number;
  param3: number;
  param4: number;
  param5: number;
  param6: number;
  param7: number;
}

/**
 * Gimbal attitude control
 */
export interface GimbalAttitude {
  pitch: number; // -90 to 90
  yaw: number; // -180 to 180
}

/**
 * Data stream request
 */
export interface DataStreamRequest {
  streamId: number;
  rate: number; // Hz
}

/**
 * Parameter setting
 */
export interface ParameterSet {
  name: string;
  value: number;
}

// ============================================================================
// MAIN INTERFACE
// ============================================================================

/**
 * Main MAVLink interface for React Native
 * Provides comprehensive drone control and monitoring capabilities
 */
export interface MAVLink extends HybridObject<{ ios: 'c++', android: 'c++' }> {
  // ============================================================================
  // CONNECTION MANAGEMENT
  // ============================================================================

  /**
   * Initialize MAVLink connection
   * @param config Connection configuration
   * @returns true if connection successful
   */
  connectWithConfig(config: ConnectionConfig): Promise<boolean>;

  /**
   * Disconnect from vehicle
   */
  disconnect(): Promise<void>;

  /**
   * Check if connected to vehicle
   */
  isConnected(): boolean;

  /**
   * Check if heartbeat timeout (>3.5s without HEARTBEAT)
   */
  isHeartbeatTimeout(): boolean;

  /**
   * Get time since last heartbeat in milliseconds
   */
  getTimeSinceLastHeartbeat(): number;

  // ============================================================================
  // VEHICLE STATE & TELEMETRY
  // ============================================================================

  /**
   * Get current latitude
   */
  getLatitude(): number;

  /**
   * Get current longitude
   */
  getLongitude(): number;

  /**
   * Get current altitude (relative)
   */
  getAltitude(): number;

  /**
   * Get current heading
   */
  getHeading(): number;

  /**
   * Get ground speed
   */
  getGroundSpeed(): number;

  /**
   * Get air speed
   */
  getAirSpeed(): number;

  /**
   * Get climb rate
   */
  getClimbRate(): number;

  /**
   * Get roll angle
   */
  getRoll(): number;

  /**
   * Get pitch angle
   */
  getPitch(): number;

  /**
   * Get yaw angle
   */
  getYaw(): number;

  /**
   * Get battery voltage
   * @param batteryId Battery ID (0-based)
   */
  getBatteryVoltage(batteryId: number): number;

  /**
   * Get battery remaining percentage
   * @param batteryId Battery ID (0-based)
   */
  getBatteryRemaining(batteryId: number): number;

  /**
   * Get GPS fix type
   */
  getGPSFixType(): number;

  /**
   * Get GPS satellite count
   */
  getGPSSatelliteCount(): number;

  /**
   * Check if vehicle is armed
   */
  isArmed(): boolean;

  /**
   * Check if vehicle is flying
   */
  isFlying(): boolean;

  /**
   * Get current flight mode
   */
  getFlightMode(): string;

  /**
   * Get system ID
   */
  getSystemId(): number;

  /**
   * Get component ID
   */
  getComponentId(): number;

  // ============================================================================
  // VEHICLE CONTROL - BASIC
  // ============================================================================

  /**
   * Arm/disarm vehicle
   * @param arm true to arm, false to disarm
   * @param force Force arming even with pre-arm checks failing
   */
  setArmed(arm: boolean, force: boolean): Promise<boolean>;

  /**
   * Change flight mode
   * @param mode Flight mode name (e.g., "GUIDED", "AUTO", "STABILIZE")
   */
  setFlightMode(mode: string): Promise<boolean>;

  // ============================================================================
  // GUIDED MODE COMMANDS
  // ============================================================================

  /**
   * Command vehicle to takeoff
   * @param altitude Target altitude in meters (relative)
   */
  guidedTakeoff(altitude: number): Promise<boolean>;

  /**
   * Command vehicle to land at current position
   */
  guidedLand(): Promise<boolean>;

  /**
   * Command vehicle to return to launch point
   * @param smartRTL Use smart RTL if available
   */
  guidedRTL(smartRTL: boolean): Promise<boolean>;

  /**
   * Command vehicle to go to location
   * @param coordinate Target coordinate
   */
  guidedGotoCoordinate(coordinate: Coordinate): Promise<boolean>;

  /**
   * Change vehicle altitude
   * @param altitudeChange Altitude change in meters (+ up, - down)
   */
  guidedChangeAltitude(altitudeChange: number): Promise<boolean>;

  /**
   * Change vehicle heading
   * @param heading Target heading in degrees (0-360)
   */
  guidedChangeHeading(heading: number): Promise<boolean>;

  /**
   * Command vehicle to orbit around a point
   * @param params Orbit parameters
   */
  guidedOrbitParams(params: OrbitParams): Promise<boolean>;

  /**
   * Set Region of Interest (camera/gimbal points to this location)
   * @param coordinate Target coordinate
   */
  guidedROICoordinate(coordinate: Coordinate): Promise<boolean>;

  /**
   * Clear Region of Interest
   */
  guidedClearROI(): Promise<boolean>;

  /**
   * Pause vehicle at current position
   */
  pauseVehicle(): Promise<boolean>;

  /**
   * Emergency stop - kills all motors
   */
  emergencyStop(): Promise<boolean>;

  // ============================================================================
  // MISSION MANAGEMENT
  // ============================================================================

  /**
   * Start mission execution
   */
  startMission(): Promise<boolean>;

  /**
   * Set current mission item
   * @param sequence Mission item sequence number
   */
  setCurrentMissionItem(sequence: number): Promise<boolean>;

  /**
   * Get current mission item sequence
   */
  getCurrentMissionItem(): number;

  /**
   * Clear all mission items
   */
  clearMission(): Promise<boolean>;

  // ============================================================================
  // PARAMETER MANAGEMENT
  // ============================================================================

  /**
   * Get parameter value
   * @param name Parameter name
   */
  getParameter(name: string): Promise<number>;

  /**
   * Set parameter value
   * @param name Parameter name
   * @param value Parameter value
   */
  setParameter(name: string, value: number): Promise<boolean>;

  /**
   * Set parameter value (object form)
   * @param param Parameter to set
   */
  setParameterValue(param: ParameterSet): Promise<boolean>;

  /**
   * Refresh all parameters from vehicle
   */
  refreshParameters(): Promise<boolean>;

  // ============================================================================
  // CAMERA & GIMBAL
  // ============================================================================

  /**
   * Trigger camera shutter
   */
  triggerCamera(): Promise<boolean>;

  /**
   * Start video recording
   */
  startVideoRecording(): Promise<boolean>;

  /**
   * Stop video recording
   */
  stopVideoRecording(): Promise<boolean>;

  /**
   * Control gimbal pitch and yaw
   * @param attitude Gimbal attitude
   */
  setGimbalAttitudeParams(attitude: GimbalAttitude): Promise<boolean>;

  // ============================================================================
  // MANUAL CONTROL
  // ============================================================================

  /**
   * Send manual control input (joystick)
   * @param input Manual control input
   */
  sendManualControlInput(input: ManualControlInput): void;

  // ============================================================================
  // ADVANCED COMMANDS
  // ============================================================================

  /**
   * Reboot autopilot
   */
  rebootAutopilot(): Promise<boolean>;

  /**
   * Request data stream
   * @param request Data stream request
   */
  requestDataStreamParams(request: DataStreamRequest): Promise<boolean>;

  /**
   * Send custom MAVLink command
   * @param params Command parameters
   */
  sendCommandParams(params: CommandParams): Promise<boolean>;
}
