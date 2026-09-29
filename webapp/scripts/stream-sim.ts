import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';
import { join, resolve } from 'node:path';
import { repoRoot } from '../server/paths.js';
import { sender } from './sender.js';
import type { Telemetry } from '../shared/types.js';
const args=process.argv.slice(2);
if(!args.length){console.error('Usage: npm run telemetry:sim -- --mode mission --mission logs/mission.apm --duration 600');process.exit(1);}
const sim=process.env.AUTOPILOT_SIM?resolve(process.env.AUTOPILOT_SIM):join(repoRoot,'build/host/bin/Debug',process.platform==='win32'?'autopilot_sim.exe':'autopilot_sim');
const stream=sender('sim');
const child=spawn(sim,[...args,'--realtime','--telemetry-stdout'],{cwd:repoRoot,stdio:['ignore','pipe','inherit']});
let last: Omit<Telemetry,'version'|'seq'|'session_id'>|undefined, chain=Promise.resolve();
const phases=['entry capture','tracking leg','complete','fly-by turn','fly-over turn'];
createInterface({input:child.stdout}).on('line',line=>{
  if(!line.startsWith('{')){console.log(line);return;}
  try{
    const data=JSON.parse(line);
    const sample={...data,phase:phases[Number(data.phase)]??data.phase,status:'running' as const};
    last=sample;
    chain=chain.then(()=>stream.send(sample)).catch(error=>console.error('Telemetry:',error.message));
  }catch{console.log(line);}
});
child.on('error',error=>{console.error(`Cannot start simulator: ${error.message}. Build autopilot_sim first.`);process.exitCode=1;});
child.on('close',code=>{
  chain.then(async()=>{if(last)await stream.send({...last,status:code===0?'completed':'failed'});}).catch(console.error).finally(()=>{stream.close();process.exitCode=code??1;});
});
process.on('SIGINT',()=>child.kill('SIGINT'));
process.on('SIGTERM',()=>child.kill('SIGTERM'));
