import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import { setTimeout as delay } from 'node:timers/promises';
import { preview } from '../server/planner.js';
import { repoRoot } from '../server/paths.js';
import { sender } from './sender.js';
const mission=JSON.parse(await readFile(join(repoRoot,'missions/c172x_turns.json'),'utf8'));
const route=await preview({mission,start:{lat_deg:30,lon_deg:0,course_deg:0,ground_speed_m_s:56.2},bank_deg:20,l1_period_s:4});
const points=route.paths.flatMap(p=>p.points),stream=sender('geometry-demo');
console.log('Sending synthetic geometry telemetry. This is not a simulated flight.');
try{
  for(let i=0;i<points.length;i++){
    const a=points[i],b=points[Math.min(i+1,points.length-1)];
    const heading=(Math.atan2((b[1]-a[1])*Math.cos(a[0]*Math.PI/180),b[0]-a[0])*180/Math.PI+360)%360;
    await stream.send({time_s:i*.1,lat_deg:a[0],lon_deg:a[1],altitude_m:914.4,heading_deg:heading,airspeed_m_s:56.2,phase:'Synthetic geometry demo',status:i===points.length-1?'completed':'running'});
    await delay(100);
  }
}finally{stream.close();}
