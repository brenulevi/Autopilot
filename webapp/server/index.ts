import { createServer, type IncomingMessage, type ServerResponse } from 'node:http';
import { createSocket } from 'node:dgram';
import { readFile, stat } from 'node:fs/promises';
import { resolve, join, extname, sep } from 'node:path';
import { WebSocketServer, WebSocket } from 'ws';
import { appRoot, repoRoot } from './paths.js';
import { compileMission } from './mission.js';
import { preview, native } from './planner.js';
import { TelemetryStore } from './telemetry.js';
const port=Number(process.env.PORT??3000), udpPort=Number(process.env.TELEMETRY_PORT??5601);
for(const value of [port,udpPort]) if(!Number.isInteger(value)||value<1||value>65535) throw Error('Invalid port.');
const production=process.argv.includes('--production');
const vite=production?undefined:await (await import('vite')).createServer({
  configFile:join(appRoot,'vite.config.ts'),root:join(appRoot,'client'),server:{middlewareMode:true},appType:'spa',
});
const origins=new Set([`http://127.0.0.1:${port}`,`http://localhost:${port}`]);
const store=new TelemetryStore();
function json(res: ServerResponse, value: unknown, status=200) {
  res.writeHead(status,{'Content-Type':'application/json','Cache-Control':'no-store'}); res.end(JSON.stringify(value));
}
async function body(req: IncomingMessage) {
  let size=0; const chunks: Buffer[]=[];
  for await(const chunk of req) { size+=chunk.length; if(size>131072) throw Error('Request exceeds 128 KB.'); chunks.push(chunk); }
  return JSON.parse(Buffer.concat(chunks).toString('utf8'));
}
const server=createServer(async(req,res)=>{
  try {
    if(!origins.has(`http://${req.headers.host}`) || (req.headers.origin && !origins.has(req.headers.origin))) {
      json(res,{error:'Use the local planner address.'},403); return;
    }
    const path=new URL(req.url??'/',`http://127.0.0.1:${port}`).pathname;
    if(path==='/api/health') {json(res,{ok:true,udp_port:udpPort});return;}
    if(path==='/api/example' && req.method==='GET') {
      json(res,JSON.parse(await readFile(join(repoRoot,'missions/c172x_turns.json'),'utf8')));return;
    }
    if(path==='/api/compile' && req.method==='POST') {
      const binary=compileMission(await body(req));
      await native(['--validate'],binary);
      res.writeHead(200,{'Content-Type':'application/octet-stream','Content-Disposition':'attachment; filename="mission.apm"','Cache-Control':'no-store'});
      res.end(binary);return;
    }
    if(path==='/api/preview' && req.method==='POST') {json(res,await preview(await body(req)));return;}
    if(path.startsWith('/api/')) {json(res,{error:'Unknown endpoint.'},404);return;}
    if(vite) {vite.middlewares(req,res);return;}
    if(req.method!=='GET' && req.method!=='HEAD') {json(res,{error:'Method not allowed.'},405);return;}
    const root=join(appRoot,'dist/client');
    const file=resolve(root,'.'+decodeURIComponent(path==='/'?'/index.html':path));
    if(!file.startsWith(root+sep)) {json(res,{error:'Not found.'},404);return;}
    try {
      if(!(await stat(file)).isFile()) throw Error('Not a file');
      const mime: Record<string,string>={'.html':'text/html','.js':'text/javascript','.css':'text/css','.png':'image/png','.svg':'image/svg+xml','.map':'application/json'};
      res.writeHead(200,{'Content-Type':mime[extname(file)]??'application/octet-stream','X-Content-Type-Options':'nosniff'});
      res.end(req.method==='HEAD'?undefined:await readFile(file));
    } catch {json(res,{error:'Not found. Run npm run build first.'},404);}
  } catch(error) {if(!res.headersSent) json(res,{error:error instanceof Error?error.message:'Request failed.'},400);else res.end();}
});
const wss=new WebSocketServer({noServer:true,maxPayload:1024});
server.on('upgrade',(req,socket,head)=>{
  if(req.url!=='/ws/telemetry'||!origins.has(`http://${req.headers.host}`)||(req.headers.origin&&!origins.has(req.headers.origin))) {socket.destroy();return;}
  wss.handleUpgrade(req,socket,head,ws=>{wss.emit('connection',ws,req);});
});
wss.on('connection',ws=>{ws.on('error',()=>{});ws.send(JSON.stringify({type:'history',samples:store.history}));});
const udp=createSocket('udp4');
udp.on('message',data=>{
  if(data.length>4096) return;
  try {
    const packet=store.accept(JSON.parse(data.toString('utf8'))); if(!packet)return;
    const message=JSON.stringify({type:'telemetry',sample:packet});
    for(const client of wss.clients) {
      if(client.bufferedAmount>1_000_000) {client.close(1013,'Slow connection; reconnect for history.');continue;}
      if(client.readyState===WebSocket.OPEN)client.send(message);
    }
  } catch { /* Malformed packets do not interrupt the receiver. */ }
});
udp.on('error',error=>{console.error('Telemetry receiver:',error.message);process.exitCode=1;shutdown();});
server.on('error',error=>{console.error('Web server:',error.message);process.exitCode=1;shutdown();});
udp.bind(udpPort,'127.0.0.1');
server.listen(port,'127.0.0.1',()=>console.log(`Mission planner: http://127.0.0.1:${port}\nTelemetry: UDP 127.0.0.1:${udpPort}`));
let closing=false;
function shutdown() {
  if(closing)return;closing=true;
  for(const client of wss.clients)client.terminate();
  wss.close();server.close();try{udp.close();}catch{}void vite?.close();
  setTimeout(()=>process.exit(process.exitCode??0),500).unref();
}
process.on('SIGINT',shutdown);process.on('SIGTERM',shutdown);
