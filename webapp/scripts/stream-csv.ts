import { createReadStream } from 'node:fs';
import { createInterface } from 'node:readline';
import { setTimeout as delay } from 'node:timers/promises';
import { sender } from './sender.js';
import type { Telemetry } from '../shared/types.js';
const [file,rate='1']=process.argv.slice(2), speed=Number(rate);
if(!file||!Number.isFinite(speed)||speed<=0||speed>100){console.error('Usage: npm run telemetry:csv -- /path/to/mission.csv [replay-speed 1..100]');process.exit(1);}
const stream=sender('replay'), source=createReadStream(file,{encoding:'utf8'});
let last:Omit<Telemetry,'version'|'seq'|'session_id'>|undefined;
try{
  const lines=createInterface({input:source,crlfDelay:Infinity});
  source.on('error',error=>{console.error(error.message);lines.close();process.exitCode=1;});
  let headers:string[]=[],nextTime=0,baseTime:number|undefined,wallStart=0;
  for await(const line of lines){
    if(!headers.length){headers=line.trim().split(',');for(const field of ['time_s','lat_deg','lon_deg','altitude_m','yaw_rad','airspeed_m_s'])if(!headers.includes(field))throw Error(`CSV is missing ${field}.`);continue;}
    const values=line.split(','), row=Object.fromEntries(headers.map((key,i)=>[key,values[i]]));
    if(row.lat_deg===''||row.lon_deg==='')continue;
    const time=Number(row.time_s);if(!Number.isFinite(time)||time<nextTime)continue;nextTime=time+.099;
    if(baseTime===undefined){baseTime=time;wallStart=performance.now();}
    const wait=(time-baseTime)*1000/speed-(performance.now()-wallStart);if(wait>0)await delay(wait);
    last={time_s:time,lat_deg:Number(row.lat_deg),lon_deg:Number(row.lon_deg),altitude_m:Number(row.altitude_m),
      heading_deg:((Number(row.yaw_rad)*180/Math.PI)%360+360)%360,airspeed_m_s:Number(row.airspeed_m_s),status:'running',phase:'CSV replay',
      ...(row.mission_leg?{mission_leg:Number(row.mission_leg)}:{}),...(row.cross_track_m?{cross_track_m:Number(row.cross_track_m)}:{})};
    await stream.send(last);
  }
  if(last)await stream.send({...last,status:'completed',phase:'Replay finished (mission outcome unknown)'});
  else if(!process.exitCode)throw Error('CSV contains no mission position samples.');
}catch(error){console.error((error as Error).message);process.exitCode=1;}
finally{source.destroy();stream.close();}
