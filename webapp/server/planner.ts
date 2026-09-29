import { spawn } from 'node:child_process';
import { nativePath } from './paths.js';
import { compileMission, validatePreview } from './mission.js';
import type { Preview } from '../shared/types.js';
let active=0;
export function native(args: string[], bytes: Buffer): Promise<string> {
  if(active>=4) return Promise.reject(Error('Planner busy. Try again shortly.'));
  active++;
  return new Promise((resolve,reject)=>{
    const child=spawn(nativePath,args,{stdio:['pipe','pipe','pipe']});
    let stdout='', stderr='', settled=false;
    const timer=setTimeout(()=>{child.kill(); finish(Error('Preview timed out.'));},5000);
    function finish(error?: Error) {
      if(settled) return; settled=true; clearTimeout(timer); active--;
      if(error) reject(error); else resolve(stdout);
    }
    child.stdout.on('data',chunk=>{stdout+=chunk; if(stdout.length>8_000_000) {child.kill();finish(Error('Preview is too large.'));}});
    child.stderr.on('data',chunk=>{if(stderr.length<4096) stderr+=chunk;});
    child.stdin.on('error',()=>{});
    child.on('error',()=>finish(Error('Native planner unavailable. Run npm run native in webapp.')));
    child.on('close',code=>finish(code===0?undefined:Error(stderr.trim()||'Native planner failed.')));
    child.stdin.end(bytes);
  });
}
export async function preview(input: unknown): Promise<Preview> {
  const p=validatePreview(input);
  return JSON.parse(await native([
    p.start.lat_deg,p.start.lon_deg,p.start.course_deg,p.start.ground_speed_m_s,p.bank_deg,p.l1_period_s,
  ].map(String),compileMission(p.mission)));
}
