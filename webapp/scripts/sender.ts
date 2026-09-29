import { createSocket } from 'node:dgram';
import { randomUUID } from 'node:crypto';
import { validateTelemetry } from '../server/telemetry.js';
import type { Telemetry } from '../shared/types.js';
export function sender(prefix: string) {
  const socket=createSocket('udp4'), session_id=`${prefix}-${randomUUID().slice(0,8)}`;
  socket.on('error',error=>{console.error(error.message);process.exitCode=1;});
  let seq=0;
  return {
    send(sample: Omit<Telemetry,'version'|'seq'|'session_id'>): Promise<void> {
      const packet=validateTelemetry({...sample,version:1,session_id,seq:seq++});
      return new Promise((resolve,reject)=>socket.send(Buffer.from(JSON.stringify(packet)),Number(process.env.TELEMETRY_PORT??5601),'127.0.0.1',error=>error?reject(error):resolve()));
    },
    close:()=>{try{socket.close();}catch{/* No datagrams were sent. */}},
  };
}
