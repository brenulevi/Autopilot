import test from 'node:test';
import assert from 'node:assert/strict';
import { createServer } from 'node:net';
import { createSocket } from 'node:dgram';
import { spawn } from 'node:child_process';
import { once } from 'node:events';
import { existsSync } from 'node:fs';
import { readFile, mkdtemp, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { setTimeout as delay } from 'node:timers/promises';
import { WebSocket } from 'ws';
import { appRoot, repoRoot } from '../server/paths.js';
async function until(predicate:()=>boolean,description:string){
  const start=Date.now();while(!predicate()){if(Date.now()-start>10000)throw Error(`Timed out: ${description}`);await delay(20);}
}
test('HTTP APM export, local-origin checks, UDP/WebSocket stream, and real simulator bridge',async t=>{
  const reservation=createServer();reservation.listen(0,'127.0.0.1');await once(reservation,'listening');
  const port=(reservation.address() as {port:number}).port;await new Promise<void>(resolve=>reservation.close(()=>resolve()));
  const udpReservation=createSocket('udp4');udpReservation.bind(0,'127.0.0.1');await once(udpReservation,'listening');
  const udpPort=udpReservation.address().port;udpReservation.close();
  const child=spawn(process.execPath,['--import','tsx','server/index.ts','--production'],{cwd:appRoot,env:{...process.env,PORT:String(port),TELEMETRY_PORT:String(udpPort)},stdio:['ignore','pipe','pipe']});
  let logs='';child.stdout.on('data',data=>logs+=data);child.stderr.on('data',data=>logs+=data);
  t.after(()=>{child.kill();});
  await until(()=>logs.includes('Mission planner:'),'server startup');
  const url=`http://127.0.0.1:${port}`;
  const mission=JSON.parse(await readFile(join(repoRoot,'missions/c172x_turns.json'),'utf8'));
  const compiled=await fetch(url+'/api/compile',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(mission)});
  assert.equal(compiled.status,200);assert.equal(compiled.headers.get('content-type'),'application/octet-stream');
  const binary=Buffer.from(await compiled.arrayBuffer());assert.equal(binary.subarray(0,4).toString(),'APM3');
  const blocked=await fetch(url+'/api/compile',{method:'POST',headers:{Origin:'https://unrelated.example'},body:JSON.stringify(mission)});assert.equal(blocked.status,403);
  const bad=await fetch(url+'/api/compile',{method:'POST',body:'{}'});assert.equal(bad.status,400);
  const ws=new WebSocket(url.replace('http','ws')+'/ws/telemetry');
  const messages:any[]=[];ws.on('message',bytes=>messages.push(JSON.parse(bytes.toString())));
  t.after(()=>ws.close());await once(ws,'open');await until(()=>messages.some(m=>m.type==='history'),'initial history');
  const sender=createSocket('udp4');t.after(()=>sender.close());
  sender.send(Buffer.from('{invalid'),udpPort,'127.0.0.1');
  const sample={version:1,session_id:'integration',seq:1,time_s:.1,lat_deg:30,lon_deg:0,altitude_m:914.4,heading_deg:0,airspeed_m_s:56.2,status:'running'};
  sender.send(Buffer.from(JSON.stringify(sample)),udpPort,'127.0.0.1');
  await until(()=>messages.some(m=>m.sample?.session_id==='integration'),'UDP forwarding');
  const received=messages.find(m=>m.sample?.session_id==='integration').sample;
  assert.equal(received.lat_deg,30);assert.ok(received.received_at>Date.now()-5000);
  const sim=join(repoRoot,'build/host/bin/Debug',process.platform==='win32'?'autopilot_sim.exe':'autopilot_sim');
  if(!existsSync(sim)){t.diagnostic('Simulator bridge check skipped: build autopilot_sim to enable.');return;}
  const temp=await mkdtemp(join(tmpdir(),'webapp-telemetry-'));t.after(()=>rm(temp,{recursive:true,force:true}));
  const bridge=spawn(process.execPath,['--import','tsx','scripts/stream-sim.ts','--mode','manual','--duration','0.3','--output',join(temp,'flight.csv')],{
    cwd:appRoot,env:{...process.env,TELEMETRY_PORT:String(udpPort)},stdio:['ignore','pipe','pipe'],
  });
  let bridgeLogs='';bridge.stdout.on('data',data=>bridgeLogs+=data);bridge.stderr.on('data',data=>bridgeLogs+=data);
  t.after(()=>bridge.kill());const [code]=await once(bridge,'close');assert.equal(code,0,bridgeLogs);
  await until(()=>messages.some(m=>m.sample?.session_id.startsWith('sim-')&&m.sample.status==='completed'),'simulator terminal telemetry');
  const frames=messages.filter(m=>m.sample?.session_id.startsWith('sim-'));
  assert.ok(frames.length>=4);assert.equal(frames[0].sample.time_s,0);
  assert.ok(Math.abs(frames[0].sample.lat_deg-30)<.001);
});
