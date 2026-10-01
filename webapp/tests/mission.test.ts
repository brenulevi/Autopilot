import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import { compileMission, crc32, roundEven } from '../server/mission.js';
import { native, preview } from '../server/planner.js';
import { repoRoot } from '../server/paths.js';
import { TelemetryStore, validateTelemetry } from '../server/telemetry.js';
const turns=JSON.parse(await readFile(join(repoRoot,'missions/c172x_turns.json'),'utf8'));
const line=JSON.parse(await readFile(join(repoRoot,'missions/c172x_line.json'),'utf8'));
const options={mission:turns,start:{lat_deg:30,lon_deg:0,course_deg:0,ground_speed_m_s:56.2},bank_deg:20,l1_period_s:4};
test('CRC32 uses the standard check vector and ties-to-even matches mission quantization',()=>{
  assert.equal(crc32(Buffer.from('123456789')),0xcbf43926);
  assert.deepEqual([0.5,1.5,2.5,-0.5,-1.5,-2.5].map(roundEven),[0,2,2,0,-2,-2]);
});
test('APM2 and APM3 bytes are accepted by the real C decoder; altered CRC is rejected',async()=>{
  for(const mission of [line,turns]){
    const bytes=compileMission(mission);
    assert.equal(bytes.length,16+16*mission.waypoints.length);
    assert.equal(bytes.readInt32LE(12),300000000);
    assert.equal(bytes.readInt32LE(20),91440);
    assert.equal(bytes.readUInt16LE(24),5620);
    assert.equal(JSON.parse(await native(['--validate'],bytes)).valid,true);
    bytes[12]^=1;await assert.rejects(native(['--validate'],bytes),/CRC/);
  }
});
test('invalid coordinates, duplicate waypoints, count, speed and legacy types are rejected',()=>{
  for(const change of [
    (m:any)=>m.waypoints[1].lat_deg=91,(m:any)=>m.waypoints[1]={...m.waypoints[0]},
    (m:any)=>m.waypoints[1].airspeed_m_s=0,(m:any)=>m.waypoints.length=1,
    (m:any)=>m.waypoints[1].lon_deg=30,(m:any)=>m.version=2,
  ]){const mission=structuredClone(turns);change(mission);assert.throws(()=>compileMission(mission));}
});
test('preview includes fly-by and fly-over with continuous transitions and expected radius',async()=>{
  const p=await preview(options);
  assert.ok(Math.abs(p.radius_m-1.6*56.2**2/(9.80665*Math.tan(20*Math.PI/180)))<.1);
  assert.ok(p.paths.some(s=>s.kind==='fly_by'));assert.ok(p.paths.some(s=>s.kind==='fly_over'));
  const paths=p.paths.filter(p=>p.points.length);
  for(let i=1;i<paths.length;i++){
    const a=paths[i-1].points.at(-1)!,b=paths[i].points[0];
    assert.ok(Math.hypot(a[0]-b[0],a[1]-b[1])<.00001,'path pieces must join');
  }
  assert.ok(p.length_m>10000);
});
test('offset start produces Dubins capture; nearest-leg entry can skip earlier legs',async()=>{
  const p=await preview({...options,start:{...options.start,lon_deg:-.01}});
  assert.equal(p.paths[0].kind,'entry');
  const later=await preview({...options,start:{...options.start,lat_deg:30.07,lon_deg:.0519207}});
  assert.equal(later.selected_leg,2);
});
test('short fly-by legs are infeasible, not silently tightened',async()=>{
  const mission=structuredClone(turns);mission.waypoints=mission.waypoints.slice(0,3);
  mission.waypoints[1].lat_deg=30.001;mission.waypoints[2].lat_deg=30.001;mission.waypoints[2].lon_deg=.001;
  await assert.rejects(preview({...options,mission}),/cannot be planned/);
});
const packet={version:1,session_id:'test-1',seq:0,time_s:0,lat_deg:30,lon_deg:0,altitude_m:914.4,heading_deg:0,airspeed_m_s:56.2,status:'running'};
test('telemetry rejects malformed values, drops out-of-order packets and bounds history',()=>{
  assert.throws(()=>validateTelemetry({...packet,lat_deg:'30'}));
  assert.throws(()=>validateTelemetry({...packet,seq:1.5}));
  const store=new TelemetryStore();assert.ok(store.accept(packet));assert.equal(store.accept(packet),undefined);
  for(let seq=1;seq<2100;seq++)store.accept({...packet,seq});
  assert.equal(store.history.length,2000);
  store.accept({...packet,session_id:'test-2'});assert.equal(store.history.length,1);
});
