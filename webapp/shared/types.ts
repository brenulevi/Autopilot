export interface Waypoint { lat_deg: number; lon_deg: number; altitude_m: number; airspeed_m_s: number; type: 'fly_by' | 'fly_over' }
export interface Mission { version: 2 | 3; waypoints: Waypoint[] }
export interface Start { lat_deg: number; lon_deg: number; course_deg: number; ground_speed_m_s: number }
export interface PreviewRequest { mission: Mission; start: Start; bank_deg: number; l1_period_s: number }
export interface Preview { radius_m: number; length_m: number; selected_leg: number; paths: { kind: 'entry' | 'straight' | 'fly_by' | 'fly_over'; leg: number; points: [number,number][] }[] }
export interface Telemetry {
  version: 1; session_id: string; seq: number; time_s: number;
  lat_deg: number; lon_deg: number; altitude_m: number; heading_deg: number;
  airspeed_m_s: number; ground_speed_m_s?: number; mission_leg?: number;
  cross_track_m?: number; phase?: string; status: 'running' | 'completed' | 'failed';
}
export type ReceivedTelemetry = Telemetry & { received_at: number };
