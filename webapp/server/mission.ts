import { Buffer } from 'node:buffer';
import type { Mission, PreviewRequest, Waypoint } from '../shared/types.js';
export function object(value: unknown, name: string): Record<string, unknown> {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw Error(`${name} must be an object.`);
  return value as Record<string, unknown>;
}
export function finite(value: unknown, name: string, min: number, max: number): number {
  if (typeof value !== 'number' || !Number.isFinite(value) || value < min || value > max)
    throw Error(`${name} must be a number between ${min} and ${max}.`);
  return value;
}
// Match the original compiler's ties-to-even quantization, including negative values.
export function roundEven(value: number): number {
  const floor = Math.floor(value), fraction = value - floor;
  return fraction === 0.5 ? (floor % 2 === 0 ? floor : floor + 1) : Math.round(value);
}
export function validateMission(input: unknown): Mission {
  const source = object(input, 'Mission');
  if (source.version !== 2 && source.version !== 3) throw Error('Mission version must be 2 or 3.');
  if (!Array.isArray(source.waypoints) || source.waypoints.length < 2 || source.waypoints.length > 32)
    throw Error('Add between 2 and 32 waypoints.');
  const waypoints = source.waypoints.map((raw, i): Waypoint => {
    const w = object(raw, `Waypoint ${i+1}`);
    if (source.version === 2 && 'type' in w) throw Error('Waypoint types require mission version 3.');
    const type = w.type ?? 'fly_by';
    if (type !== 'fly_by' && type !== 'fly_over') throw Error(`Waypoint ${i+1}: invalid turn type.`);
    return {
      lat_deg: finite(w.lat_deg, `Waypoint ${i+1} latitude`, -90, 90),
      lon_deg: finite(w.lon_deg, `Waypoint ${i+1} longitude`, -180, 180),
      altitude_m: finite(w.altitude_m, `Waypoint ${i+1} altitude (MSL)`, -500, 10000),
      airspeed_m_s: finite(w.airspeed_m_s, `Waypoint ${i+1} airspeed (TAS)`, 5, 100),
      type,
    };
  });
  const lat0 = roundEven(waypoints[0].lat_deg*1e7), lon0 = roundEven(waypoints[0].lon_deg*1e7);
  const scale = 6371000 * Math.PI/180 * 1e-7;
  let previous: [number,number] | undefined;
  waypoints.forEach((w,i) => {
    let delta = roundEven(w.lon_deg*1e7)-lon0;
    if (delta>1800000000) delta-=3600000000;
    if (delta< -1800000000) delta+=3600000000;
    const n=(roundEven(w.lat_deg*1e7)-lat0)*scale, e=delta*scale*Math.cos(lat0*1e-7*Math.PI/180);
    if (Math.abs(n)>20000 || Math.abs(e)>20000) throw Error(`Waypoint ${i+1} exceeds the ±20 km local frame.`);
    if (previous && Math.hypot(n-previous[0],e-previous[1])<1) throw Error(`Waypoint ${i+1} must be at least 1 m from its predecessor.`);
    previous=[n,e];
  });
  return { version: source.version, waypoints };
}
export function crc32(bytes: Uint8Array): number {
  let crc=0xffffffff;
  for (const byte of bytes) {
    crc^=byte;
    for (let i=0;i<8;i++) crc=(crc>>>1)^((crc&1)?0xedb88320:0);
  }
  return (crc^0xffffffff)>>>0;
}
export function compileMission(input: unknown): Buffer {
  const mission=validateMission(input), bytes=Buffer.alloc(16+16*mission.waypoints.length);
  bytes.write(`APM${mission.version}`); bytes.writeUInt16LE(mission.version,4); bytes.writeUInt16LE(mission.waypoints.length,6);
  mission.waypoints.forEach((w,i)=>{
    const offset=12+i*16;
    bytes.writeInt32LE(roundEven(w.lat_deg*1e7),offset);
    bytes.writeInt32LE(roundEven(w.lon_deg*1e7),offset+4);
    bytes.writeInt32LE(roundEven(w.altitude_m*100),offset+8);
    bytes.writeUInt16LE(roundEven(w.airspeed_m_s*100),offset+12);
    bytes.writeUInt16LE(w.type==='fly_over'?1:0,offset+14);
  });
  bytes.writeUInt32LE(crc32(bytes.subarray(0,-4)),bytes.length-4);
  return bytes;
}
export function validatePreview(input: unknown): PreviewRequest {
  const request=object(input,'Preview'), start=object(request.start,'Start');
  const mission=validateMission(request.mission);
  // Normalize legacy input for consumers without feeding explicit types back to APM2 validation.
  mission.version=3;
  return {mission,start:{
    lat_deg:finite(start.lat_deg,'Start latitude',-90,90),lon_deg:finite(start.lon_deg,'Start longitude',-180,180),
    course_deg:finite(start.course_deg,'Start ground course',0,359.999999),
    ground_speed_m_s:finite(start.ground_speed_m_s,'Start ground speed',5,150),
  },bank_deg:finite(request.bank_deg,'Maximum bank',1,60),l1_period_s:finite(request.l1_period_s,'L1 period',1,30)};
}
