export type Health = Record<string, boolean>
export type Dict = Record<string, unknown>

export interface Diagnostic {
  code: string; level: string; timestamp: number; reason: string; status: string
}

export interface StationState {
  sequence: number
  timestamp: number
  offline: boolean
  simulation: boolean
  connected: boolean
  operator_state: string
  health: Health
  fresh: Record<string, boolean>
  ages: Record<string, number>
  joints: Record<string, number>
  joint_velocities: Record<string, number>
  joint_velocity_source: string
  joint_limits: Record<string, [number, number]>
  wrench: Record<string, number>
  tcp: Record<string, number>
  force_z?: number | null
  filtered_force_z?: number | null
  force_setpoint?: number | null
  haptic_enabled?: boolean
  haptic_force_fresh?: boolean
  haptic_command?: {x: number; y: number; z: number}
  control_mode?: string
  stop_reason?: string
  tool_voltage?: number | null
  selected_tool?: string | null
  control_launch_mode?: string | null
  vision_status?: Dict
  task_status?: Dict
  five_target_status?: Dict
  marker_0?: Dict
  image_topic?: string
  subsystems: Record<string, {running: boolean; owned: boolean; expected_nodes: string[]}>
  control_session: {active: boolean; remaining_ms: number; client?: string; websocket_bound?: boolean}
  execution: Record<string, boolean>
  diagnostics: Diagnostic[]
  events: {timestamp: number; level: string; source: string; message: string}[]
  sampling_hardware: {
    authorized: boolean
    pump_channel: number
    sensors: {
      online: boolean; ec?: number|null; ec_temp?: number|null
      orp?: number|null; ph?: number|null; relay_state: number
      relay_online: boolean; pump_on: boolean; health: Record<string,string>
      status: string; updated?: number|null
    }
    motor: {
      online: boolean; busy: boolean; command: string; status: string
      updated?: number|null; position?: number; velocity?: number; torque?: number
    }
  }
}

export interface JointDescriptor {
  name: string; parent: string; child: string; axis: [number, number, number]
  matrix: number[]; lower: number; upper: number; velocity: number
}
export interface VisualDescriptor {
  id: number; link: string; mesh: string; scale: [number, number, number]
  matrix: number[]; source: string
}
export interface RobotDescriptor {
  model: string; base: string; tip: string; xacro: string; link7_variant: string
  joints: JointDescriptor[]; visuals: VisualDescriptor[]
}
