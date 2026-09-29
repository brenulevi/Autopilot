import { finite, object } from './mission.js';
import type { Telemetry, ReceivedTelemetry } from '../shared/types.js';
export function validateTelemetry(input: unknown): Telemetry {
  const p=object(input,'Telemetry');
  if(p.version!==1) throw Error('Unsupported telemetry version.');
  if(typeof p.session_id!=='string' || !/^[\w.-]{1,80}$/.test(p.session_id)) throw Error('Invalid session ID.');
  const seq=finite(p.seq,'Sequence',0,Number.MAX_SAFE_INTEGER);
  if(!Number.isInteger(seq)) throw Error('Sequence must be an integer.');
  if(p.status!=='running' && p.status!=='completed' && p.status!=='failed') throw Error('Invalid flight status.');
  const packet: Telemetry={version:1,session_id:p.session_id,seq,status:p.status,
    time_s:finite(p.time_s,'Simulation time',0,1e9),lat_deg:finite(p.lat_deg,'Latitude',-90,90),
    lon_deg:finite(p.lon_deg,'Longitude',-180,180),altitude_m:finite(p.altitude_m,'Altitude',-1000,100000),
    heading_deg:finite(p.heading_deg,'Heading',0,360),airspeed_m_s:finite(p.airspeed_m_s,'Airspeed',0,2000)};
  if(p.ground_speed_m_s!==undefined) packet.ground_speed_m_s=finite(p.ground_speed_m_s,'Ground speed',0,2000);
  if(p.cross_track_m!==undefined) packet.cross_track_m=finite(p.cross_track_m,'Cross-track error',-1e6,1e6);
  if(p.mission_leg!==undefined) {
    packet.mission_leg=finite(p.mission_leg,'Mission leg',0,30);
    if(!Number.isInteger(packet.mission_leg)) throw Error('Mission leg must be an integer.');
  }
  if(p.phase!==undefined) {
    if(typeof p.phase!=='string' || p.phase.length>60) throw Error('Invalid phase.');
    packet.phase=p.phase;
  }
  return packet;
}
export class TelemetryStore {
  history: ReceivedTelemetry[]=[];
  private sequences=new Map<string,number>();
  accept(input: unknown): ReceivedTelemetry | undefined {
    const p=validateTelemetry(input);
    if(p.seq<=(this.sequences.get(p.session_id)??-1)) return;
    this.sequences.set(p.session_id,p.seq);
    if(this.sequences.size>32) this.sequences.delete(this.sequences.keys().next().value!);
    if(this.history.at(-1)?.session_id!==p.session_id) this.history=[];
    const received={...p,received_at:Date.now()};
    this.history.push(received);
    if(this.history.length>2000) this.history.splice(0,this.history.length-2000);
    return received;
  }
}
