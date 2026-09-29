import L from 'leaflet';
import './style.css';
import type { Mission, Preview, PreviewRequest, ReceivedTelemetry, Waypoint } from '../shared/types.js';
const $=<T extends HTMLElement=HTMLElement>(id:string)=>document.getElementById(id) as T;
const input=(id:string)=>$<HTMLInputElement>(id);
let mission: Mission={version:3,waypoints:[]};
let start={lat_deg:30,lon_deg:0,course_deg:0,ground_speed_m_s:56.2};
let selected=0, mode:'add'|'pan'|'start'='add', timer:ReturnType<typeof setTimeout>, generation=0;
let controller: AbortController|undefined;
let settings={bank_deg:20,l1_period_s:4};
const map=L.map('map',{zoomControl:false,worldCopyJump:true}).setView([30.04,0.025],12);
L.control.zoom({position:'bottomright'}).addTo(map);
L.control.scale({position:'bottomright',imperial:false}).addTo(map);
const tiles=L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',{
  attribution:'&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',maxZoom:19,
}).addTo(map);
let tileNotice=false;
tiles.on('tileerror',()=>{if(!tileNotice){tileNotice=true;notice('Map tiles are unavailable. The route editor still works; reconnect to load the basemap.');}});
const route=L.layerGroup().addTo(map), markers=L.layerGroup().addTo(map);
const skeleton=L.polyline([],{color:'#84977b',weight:2,opacity:.7,dashArray:'5 7'}).addTo(map);
const trail=L.polyline([],{color:'#438ac0',weight:3,opacity:.95}).addTo(map);
const startMarker=L.marker([30,0],{draggable:true,zIndexOffset:500,icon:L.divIcon({className:'start-icon',html:'S',iconSize:[27,27],iconAnchor:[13,13]})}).addTo(map).bindTooltip('Aircraft start · drag to move');
startMarker.on('dragend',()=>{const p=startMarker.getLatLng().wrap();start.lat_deg=p.lat;start.lon_deg=p.lng;schedule();});
let aircraft:L.Marker|undefined;
let history:ReceivedTelemetry[]=[],latest:ReceivedTelemetry|undefined,connected=false;
function notice(message:string,error=false){const n=$('notice');n.textContent=message;n.className=error?'error':'';n.hidden=false;setTimeout(()=>{if(n.textContent===message)n.hidden=true;},7000);}
async function request(path:string,body?:unknown){
  const res=await fetch(path,body===undefined?undefined:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  if(!res.ok){const e=await res.json();throw Error(e.error??'Request failed.');}return res;
}
function download(data:Blob,name:string){const url=URL.createObjectURL(data),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
function persist(){try{localStorage.setItem('autopilot-plan-v1',JSON.stringify({mission,start,...settings}));}catch{}}
function fit(){const points: L.LatLngExpression[]=mission.waypoints.map(w=>[w.lat_deg,w.lon_deg]);points.push([start.lat_deg,start.lon_deg]);if(points.length)map.fitBounds(L.latLngBounds(points),{padding:[65,90],maxZoom:14});}
function syncSettings(){input('bank').value=String(settings.bank_deg);input('l1').value=String(settings.l1_period_s);input('course').value=String(start.course_deg);input('speed').value=String(start.ground_speed_m_s);startMarker.setLatLng([start.lat_deg,start.lon_deg]);}
function render(){
  markers.clearLayers();$('count').textContent=`${mission.waypoints.length} / 32`;
  skeleton.setLatLngs(mission.waypoints.map(w=>[w.lat_deg,w.lon_deg]));
  const list=$('waypoints');list.replaceChildren();
  if(!mission.waypoints.length){const e=document.createElement('div');e.className='empty';e.textContent='Your route starts with a click on the map.';list.append(e);}
  mission.waypoints.forEach((w,i)=>{
    const button=document.createElement('button');button.className=`waypoint-row ${selected===i?'selected':''}`;
    button.innerHTML=`<span class="waypoint-number">${i+1}</span><span class="waypoint-copy">Waypoint ${String(i+1).padStart(2,'0')}<small>${w.altitude_m} m MSL · ${w.airspeed_m_s} m/s</small></span><span class="turn-label">${i===0?'ORIGIN':w.type==='fly_over'?'FLY-OVER':'FLY-BY'}</span>`;
    button.onclick=()=>{selected=i;render();};list.append(button);
    const marker=L.marker([w.lat_deg,w.lon_deg],{draggable:true,icon:L.divIcon({className:`waypoint-icon ${selected===i?'chosen':''}`,html:String(i+1),iconSize:[29,29],iconAnchor:[14,14]})}).addTo(markers);
    marker.bindTooltip(`Waypoint ${i+1} · ${w.type==='fly_over'?'fly-over':'fly-by'}`);
    marker.on('click',()=>{selected=i;render();});
    marker.on('dragend',()=>{const p=marker.getLatLng().wrap();w.lat_deg=Number(p.lat.toFixed(7));w.lon_deg=Number(p.lng.toFixed(7));selected=i;render();schedule();});
  });
  const editor=$('waypoint-editor'), w=mission.waypoints[selected];
  if(!w){editor.replaceChildren();return;}
  editor.innerHTML=`<div class="editor-heading"><h2>Waypoint ${String(selected+1).padStart(2,'0')}</h2><div class="editor-actions"><button id="move-up" aria-label="Move waypoint earlier">↑</button><button id="move-down" aria-label="Move waypoint later">↓</button><button id="delete" aria-label="Delete waypoint">Delete</button></div></div>
    <div class="field-grid"><label>Latitude · °<input id="wp-lat" type="number" step="0.0000001" min="-90" max="90" value="${w.lat_deg}"/></label><label>Longitude · °<input id="wp-lon" type="number" step="0.0000001" min="-180" max="180" value="${w.lon_deg}"/></label><label>Altitude MSL · m<input id="wp-alt" type="number" step="0.1" min="-500" max="10000" value="${w.altitude_m}"/></label><label>Airspeed TAS · m/s<input id="wp-speed" type="number" step="0.1" min="5" max="100" value="${w.airspeed_m_s}"/></label></div>
    <label class="full-field">Waypoint behavior<select id="wp-type"><option value="fly_by">Fly-by · turn before waypoint</option><option value="fly_over">Fly-over · pass waypoint first</option></select></label>${selected===0?'<p class="hint">Waypoint 1 anchors the coordinate frame. Entry selects the nearest route leg.</p>':''}`;
  const keys=[['wp-lat','lat_deg'],['wp-lon','lon_deg'],['wp-alt','altitude_m'],['wp-speed','airspeed_m_s']] as const;
  keys.forEach(([id,key])=>{input(id).onchange=()=>{const field=input(id);if(!field.checkValidity()||field.value===''){field.reportValidity();field.value=String(w[key]);return;}w[key]=field.valueAsNumber;render();schedule();};});
  const select=$<HTMLSelectElement>('wp-type');select.value=w.type;select.disabled=selected===0;
  select.onchange=()=>{w.type=select.value as Waypoint['type'];render();schedule();};
  $('delete').onclick=()=>{mission.waypoints.splice(selected,1);selected=Math.max(0,selected-1);render();schedule();};
  function move(delta:number){const next=selected+delta;[mission.waypoints[selected],mission.waypoints[next]]=[mission.waypoints[next],mission.waypoints[selected]];selected=next;render();schedule();}
  $<HTMLButtonElement>('move-up').disabled=selected===0;$('move-up').onclick=()=>move(-1);
  $<HTMLButtonElement>('move-down').disabled=selected===mission.waypoints.length-1;$('move-down').onclick=()=>move(1);
}
function previewState(label:string,message:string,error=false){$('preview-label').textContent=label;$('preview-message').textContent=message;$('preview-dot').className=`dot ${error?'red':'green'}`;}
function schedule(){
  persist();route.clearLayers();generation++;controller?.abort();clearTimeout(timer);
  for(const id of ['length','radius','entry-leg'])$(id).textContent='—';
  if(mission.waypoints.length<2){previewState('Add your waypoints','Place at least two waypoints to preview a route.');return;}
  previewState('Updating preview…','Checking turn geometry and entry capture.');
  timer=setTimeout(()=>void updatePreview(generation),220);
}
async function updatePreview(revision:number){
  controller=new AbortController();
  try{
    const p:PreviewRequest={mission,start,...settings};
    const res=await fetch('/api/preview',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p),signal:controller.signal});
    const data=await res.json();if(revision!==generation)return;if(!res.ok)throw Error(data.error);
    const preview=data as Preview;
    route.clearLayers();
    for(const path of preview.paths){
      if(!path.points.length)continue;
      L.polyline(path.points,{color:path.kind==='entry'?'#cc903e':'#267452',weight:path.kind==='entry'?3:4,opacity:.9,dashArray:path.kind==='entry'?'7 5':undefined}).bindTooltip(`${path.kind.replaceAll('_',' ')} · leg ${path.leg+1}`).addTo(route);
    }
    $('length').textContent=`${(preview.length_m/1000).toFixed(2)} km`;$('radius').textContent=`${Math.round(preview.radius_m)} m`;$('entry-leg').textContent=String(preview.selected_leg+1).padStart(2,'0');
    previewState('Route preview ready',preview.selected_leg>0?`Entry joins leg ${preview.selected_leg+1}; earlier waypoints are skipped.`:'Turn geometry fits the selected speed and bank limit.');
  }catch(error){if(revision!==generation || (error as Error).name==='AbortError')return;previewState('Preview unavailable',(error as Error).message,true);}
}
map.on('click',(e:L.LeafletMouseEvent)=>{
  const p=e.latlng.wrap();
  if(mode==='pan')return;
  if(mode==='start'){start.lat_deg=p.lat;start.lon_deg=p.lng;syncSettings();schedule();return;}
  if(mission.waypoints.length>=32){notice('A mission supports up to 32 waypoints.',true);return;}
  const last=mission.waypoints.at(-1);
  mission.waypoints.push({lat_deg:Number(p.lat.toFixed(7)),lon_deg:Number(p.lng.toFixed(7)),altitude_m:last?.altitude_m??914.4,airspeed_m_s:last?.airspeed_m_s??56.2,type:'fly_by'});
  if(mission.waypoints.length===1){start.lat_deg=p.lat;start.lon_deg=p.lng;syncSettings();}
  selected=mission.waypoints.length-1;render();schedule();
});
for(const m of ['add','pan','start'] as const)$(`mode-${m}`).onclick=()=>{mode=m;for(const key of ['add','pan','start'])$(`mode-${key}`).classList.toggle('active',m===key);};
for(const id of ['bank','l1','course','speed'])input(id).onchange=()=>{
  const field=input(id);if(!field.checkValidity()||field.value===''){field.reportValidity();syncSettings();return;}
  settings={bank_deg:input('bank').valueAsNumber,l1_period_s:input('l1').valueAsNumber};start.course_deg=input('course').valueAsNumber;start.ground_speed_m_s=input('speed').valueAsNumber;schedule();
};
$('fit').onclick=fit;
$('reset-start').onclick=()=>{const w=mission.waypoints[0];if(!w)return;start.lat_deg=w.lat_deg;start.lon_deg=w.lon_deg;syncSettings();schedule();};
$('save').onclick=()=>download(new Blob([JSON.stringify(mission,null,2)+'\n'],{type:'application/json'}),'mission.json');
$('export').onclick=async()=>{const button=$<HTMLButtonElement>('export');button.disabled=true;try{const res=await request('/api/compile',mission);download(await res.blob(),'mission.apm');notice('APM compiled and accepted by the autopilot decoder.');}catch(error){notice((error as Error).message,true);}finally{button.disabled=false;}};
$('import').onclick=()=>input('file').click();
async function loadMission(value:unknown){
  // The server validates all coordinates and values before they enter the editor.
  await request('/api/compile',value);
  const raw=value as Mission;mission={version:3,waypoints:raw.waypoints.map(w=>({...w,type:w.type??'fly_by'}))};
  selected=0;start.lat_deg=mission.waypoints[0].lat_deg;start.lon_deg=mission.waypoints[0].lon_deg;
  syncSettings();render();fit();schedule();
}
input('file').onchange=async()=>{const file=input('file').files?.[0];if(!file)return;try{if(file.size>131072)throw Error('Mission JSON exceeds 128 KB.');await loadMission(JSON.parse(await file.text()));notice('Mission imported.');}catch(error){notice((error as Error).message,true);}input('file').value='';};
$('example').onclick=async()=>{try{await loadMission(await (await request('/api/example')).json());}catch(error){notice((error as Error).message,true);}};
$('new').onclick=()=>{mission={version:3,waypoints:[]};selected=0;render();schedule();};
$('clear-track').onclick=()=>{history=[];trail.setLatLngs([]);};
function receive(sample:ReceivedTelemetry,draw=true){
  if(latest?.session_id!==sample.session_id)history=[];
  if(latest?.session_id===sample.session_id && sample.seq<=latest.seq)return;
  latest=sample;history.push(sample);if(history.length>2000)history.shift();
  if(draw)drawTelemetry();
}
function drawTelemetry(){
  if(!latest)return;
  trail.setLatLngs(history.map(p=>[p.lat_deg,p.lon_deg]));
  const icon=L.divIcon({className:'aircraft-icon',html:`<span style="transform:rotate(${latest.heading_deg}deg)" aria-label="Aircraft">↑</span>`,iconSize:[34,34],iconAnchor:[17,17]});
  if(!aircraft)aircraft=L.marker([latest.lat_deg,latest.lon_deg],{icon,zIndexOffset:1000}).addTo(map);else aircraft.setLatLng([latest.lat_deg,latest.lon_deg]).setIcon(icon);
  if(input('follow').checked)map.panTo([latest.lat_deg,latest.lon_deg],{animate:false});
  $('t-alt').textContent=`${latest.altitude_m.toFixed(1)} m`;$('t-speed').textContent=`${latest.airspeed_m_s.toFixed(1)} m/s`;
  $('t-cross').textContent=latest.cross_track_m===undefined?'—':`${latest.cross_track_m.toFixed(1)} m`;$('t-time').textContent=`${latest.time_s.toFixed(1)} s`;
  $('session').textContent=`${latest.session_id}${latest.mission_leg===undefined?'':` · leg ${latest.mission_leg+1}`}${latest.phase?` · ${latest.phase}`:''}`;
}
function connect(){
  const ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws/telemetry`);
  ws.onopen=()=>{connected=true;};
  ws.onmessage=event=>{try{const data=JSON.parse(event.data);if(data.type==='history'){history=[];latest=undefined;for(const sample of data.samples)receive(sample,false);drawTelemetry();}else if(data.type==='telemetry')receive(data.sample);}catch{}};
  ws.onclose=()=>{connected=false;setTimeout(connect,1500);};ws.onerror=()=>ws.close();
}
setInterval(()=>{
  const age=latest?(Date.now()-latest.received_at)/1000:Infinity;
  const stale=age>3;
  $('connection').textContent=!connected?'Reconnecting…':!latest?'Receiver ready':latest.status==='completed'?'Flight complete':latest.status==='failed'?'Flight failed':stale?`Telemetry stale · ${Math.floor(age)}s`:'Receiving telemetry';
  $('connection-dot').className=`dot ${!connected||stale?'amber':latest?.status==='failed'?'red':'green'}`;
  aircraft?.setOpacity(stale?.45:1);
},500);
async function init(){
  try{
    const saved=localStorage.getItem('autopilot-plan-v1');
    if(saved){const p=JSON.parse(saved) as PreviewRequest;await loadMission(p.mission);start=p.start;settings={bank_deg:p.bank_deg,l1_period_s:p.l1_period_s};syncSettings();fit();schedule();}
    else await loadMission(await (await request('/api/example')).json());
  }catch(error){notice(`Could not restore the plan: ${(error as Error).message}`,true);render();schedule();}
  connect();
}
void init();
