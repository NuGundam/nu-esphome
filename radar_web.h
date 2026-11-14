#pragma once
// ──────────────────────────────────────────────────────────────────
//  NU-Presence-24GHz  radar tuning page  v1.10  (radar_web.h)  2026-11-11
//  v1.10: the browser tab title carries the device name (document.title, set every render).
//  v1.9: edit lock - the page opens locked (no writes, bars not draggable, chart scrolls);
//        'Locked - tap to edit' unlocks for 2 min after the last edit. Used by NU-Presence-24GHz-1 + NU-Garage-1.
//
//  Same idea as NU-AQI-3's custom_web.h: an IDF httpd on port 8080 serving one
//  embedded page plus small JSON endpoints, reading the ESPHome entities
//  directly in C++ (no web_server component, no HA round trip).
//
//    /            tuning page (room/scope view with gate rings and a room box,
//                 zones on a distance axis, 9 gate columns with live
//                 move/still energy, draggable thresholds, peak hold)
//    /data.json   live radar state + every tunable value, polled by the page
//    /set?k=..&v= write a number (zone edges, gate thresholds, max gates,
//                 clear delay) - goes through the ESPHome number, so HA sees
//                 the change and ld2410 numbers are written to the module
//    /sw?k=eng&v= engineering mode on/off; /sw?k=query&v=1 re-reads the module's settings
//    /healthz     "OK"
//
//  Entities are handed over from the yaml (radar_web_cfg() filled in a lambda,
//  where id() works) - a header included before main.cpp cannot use id().
//
//  Threading (v1.2): the httpd runs in its own FreeRTOS task. It only READS
//  plain floats/bools that main-loop callbacks keep up to date, and WRITES go
//  into a small queue that radar_web_pump() (a 100 ms yaml interval, main
//  loop) performs. Calling number/switch/button actions from the httpd task
//  crashed the ESP (task wdt) - the ld2410 driver shares the UART with the loop.
// ──────────────────────────────────────────────────────────────────
#include "esphome.h"
#include "esp_http_server.h"
#include <string>
#include <cstring>
#include <cstdlib>
#include <cmath>

struct RadarWebCfg {
  esphome::sensor::Sensor *dist = nullptr, *mdist = nullptr, *sdist = nullptr, *men = nullptr, *sen = nullptr;
  esphome::sensor::Sensor *gm[9] = {}, *gs[9] = {};
  esphome::binary_sensor::BinarySensor *occ = nullptr, *mv = nullptr, *st = nullptr, *za = nullptr, *zb = nullptr;
  esphome::number::Number *a_near = nullptr, *a_far = nullptr, *b_near = nullptr, *b_far = nullptr, *edge = nullptr, *hold = nullptr;
  esphome::number::Number *maxm = nullptr, *maxs = nullptr, *timeout = nullptr, *gmt[9] = {}, *gst[9] = {};
  esphome::switch_::Switch *eng = nullptr;
  esphome::select::Select *res = nullptr;
  esphome::text_sensor::TextSensor *fw = nullptr;
  esphome::button::Button *query = nullptr;
  esphome::sensor::Sensor *uptime = nullptr, *rssi = nullptr;
};
static RadarWebCfg &radar_web_cfg() { static RadarWebCfg c; return c; }

// raw (unfiltered) copies, filled by callbacks so the page sees the 10 Hz stream, not the recorder-filtered values
static float rw_dist = NAN, rw_mdist = NAN, rw_sdist = NAN, rw_men = NAN, rw_sen = NAN, rw_gm[9], rw_gs[9];
static char rw_fw[32] = "", rw_res[16] = "";   // cached on the main loop by callbacks

// write queue: httpd task -> main loop
struct RwCmd { char k[12]; float v; };
static RwCmd rw_q[8];
static volatile uint8_t rw_qh = 0, rw_qt = 0;
static portMUX_TYPE rw_mux = portMUX_INITIALIZER_UNLOCKED;
static bool rw_push(const char *k, float v) {
  bool ok = false;
  taskENTER_CRITICAL(&rw_mux);
  uint8_t nt = (rw_qt + 1) & 7;
  if (nt != rw_qh) { strlcpy(rw_q[rw_qt].k, k, sizeof rw_q[rw_qt].k); rw_q[rw_qt].v = v; rw_qt = nt; ok = true; }
  taskEXIT_CRITICAL(&rw_mux);
  return ok;
}

static const char RADAR_WEB_HTML[] PROGMEM = R"ESPH(<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Radar tuning</title>
<style>
:root{--bg:#10151c;--card:#182029;--line:#243040;--tx:#e6edf3;--mut:#8b9bb0;--mv:#4aa3ff;--st:#ff9f43;--za:#34d399;--zb:#60a5fa;--bad:#f87171}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif;padding:12px 16px 32px}
.wrap{max-width:1100px;margin:0 auto}
h1{font-size:18px;margin:0 0 4px}h2{font-size:13px;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;margin:18px 0 8px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
.row{display:flex;flex-wrap:wrap;gap:10px 18px;align-items:center}
.pill{display:inline-block;padding:2px 9px;border-radius:999px;background:#2a3645;color:var(--mut);font-weight:600;font-size:12px}
.pill.on{background:#1e5a3f;color:#9ff0c9}.pill.za.on{background:#145c43;color:#b2f5dc}.pill.zb.on{background:#1b3f73;color:#bcd8ff}
.big{font-size:34px;font-weight:800;line-height:1}.sub{color:var(--mut);font-size:12px}
label{color:var(--mut);font-size:12px;display:flex;flex-direction:column;gap:3px}
input[type=number],select{background:#0d1218;color:var(--tx);border:1px solid var(--line);border-radius:6px;padding:5px 7px;font-size:14px;width:84px}
select{width:auto;min-width:132px}
input.g{width:46px;padding:3px 4px;font-size:12px;text-align:center}
button{background:#263445;color:var(--tx);border:1px solid #35465b;border-radius:7px;padding:6px 11px;font-size:13px;cursor:pointer}button:hover{background:#31435a}
svg{display:block;width:100%;touch-action:none}#axis{height:96px}#gates{height:236px}#fan{height:420px}
html,body{overflow-x:hidden}.wrap{min-width:0}
.warn{color:var(--bad);font-size:12px}.ok{color:var(--za);font-size:12px}
.gates{display:grid;grid-template-columns:repeat(9,minmax(0,1fr));gap:4px;margin-top:6px;font-size:11px;text-align:center;color:var(--mut)}
.gates>div{min-width:0}.gates .rs{display:none}.gates .rg{display:block}
.gates b{display:block;color:var(--tx);font-size:12px}.gates div input{margin:2px 1px 0}
.leg{display:flex;flex-wrap:wrap;gap:6px 14px;font-size:12px;color:var(--mut);margin-top:6px}.leg i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px;vertical-align:-1px}
.help{color:var(--mut);font-size:12px;margin-top:8px;line-height:1.5}
#lock{position:sticky;top:6px;z-index:5;margin:8px 0 2px;font-weight:700}#lock.un{background:#5a3a12;border-color:#a8721f;color:#ffd79a}
input:disabled,select:disabled,button:disabled{opacity:.45;cursor:not-allowed}
@media (max-width:640px){
 body{padding:8px 8px 24px;font-size:13px}h1{font-size:16px}h2{margin:14px 2px 6px}
 .card{padding:10px;border-radius:12px}.big{font-size:28px}.row{gap:8px 10px}
 .row>label:not(.ck){flex:1 1 calc(33% - 10px);min-width:90px}.row{align-items:flex-end}.row>label.sel{flex:1 1 calc(50% - 10px);min-width:140px}
 .row>label input[type=number],.row>label select{width:100%;min-width:0;padding:7px 8px}
 button{min-height:38px}#roomTxt{flex-basis:100%}#applyMax{flex:1 1 100%}
 .gates{gap:2px;font-size:10px}.gates b{font-size:11px}.gates .rg{display:none}.gates .rs{display:block}
 input.g{width:100%;margin:3px 0 0!important;padding:6px 0;font-size:13px}
}
</style></head><body><div class="wrap">
<h1 id="ttl">Radar tuning</h1><div class="sub" id="meta">connecting&hellip;</div>
<button id="lock">&#128274; Locked &ndash; tap to edit</button>

<h2>Live</h2><div class="card"><div class="row">
<div><div class="big" id="dist">&mdash;</div><div class="sub">target distance</div></div>
<div><span class="pill" id="pOcc">Occupied</span> <span class="pill" id="pMv">Moving</span> <span class="pill" id="pSt">Still</span></div>
<div><span class="pill za" id="pZa">Zone A</span> <span class="pill zb" id="pZb">Zone B</span></div>
<div class="sub">moving <b id="men">&mdash;</b> @ <span id="mdist">&mdash;</span> &middot; still <b id="sen">&mdash;</b> @ <span id="sdist">&mdash;</span></div>
</div></div>

<h2>Room view</h2><div class="card">
<svg id="fan" viewBox="0 0 900 520" preserveAspectRatio="xMidYMid meet"></svg>
<div class="row" style="margin-top:8px">
<label>Room width m<input type="number" id="room_w" min="1" max="20" step="0.1"></label>
<label>Room depth m<input type="number" id="room_d" min="1" max="20" step="0.1"></label>
<label class="sel">Sensor mounted<select id="mount"><option value="wall">middle of the near wall</option><option value="corner">in a corner, pointing across</option></select></label>
<span id="roomTxt" class="sub"></span>
<button id="applyMax">Apply suggested max gates</button>
</div>
<div class="help">Top-down view, sensor at the bottom looking up, about 120&deg; wide. The radar only knows distance, so the target is drawn as an arc - it is somewhere on that arc. Rings are the gates, tinted orange by their live still energy with a blue band as wide as their move energy (engineering mode on) - that is where the reflections are along the range; A/B are the zone bands; grey = past the max gates. The room box is only for your eyes (saved in this browser): it shows which rings fall inside the room and where the far wall lands, so you know which gates matter and where to cut off.</div>
</div>

<h2>Zones &amp; range</h2><div class="card">
<svg id="axis" viewBox="0 0 900 96" preserveAspectRatio="none"></svg>
<div class="row" style="margin-top:10px">
<label>Zone A near m<input type="number" id="a_near" min="0" max="6.75" step="0.05"></label>
<label>Zone A far m<input type="number" id="a_far" min="0" max="6.75" step="0.05"></label>
<label>Zone B near m<input type="number" id="b_near" min="0" max="6.75" step="0.05"></label>
<label>Zone B far m<input type="number" id="b_far" min="0" max="6.75" step="0.05"></label>
<label>Edge (hysteresis) m<input type="number" id="edge" min="0" max="1" step="0.05"></label>
<label>Hold s<input type="number" id="hold" min="0" max="600" step="5"></label>
</div>
<div class="row" style="margin-top:10px">
<label class="sel">Max moving gate<select id="maxm"></select></label>
<label class="sel">Max still gate<select id="maxs"></select></label>
<label>Clear delay s<input type="number" id="timeout" min="0" max="65535" step="1"></label>
<label>Resolution<span id="res" style="color:var(--tx);padding:5px 0">&mdash;</span></label>
<button id="rq" title="ask the module for its settings again (blank boxes after a power cycle)">Re-read radar settings</button>
</div>
<div class="help">A zone is on while the radar's target is inside its band (plus Edge on either side once on) and stays on for Hold after it leaves. Anything past the max gates is never scanned - set them to the gate just beyond the farthest spot you care about.</div>
</div>

<h2>Gates - energy vs threshold</h2><div class="card">
<div class="row" style="margin-bottom:6px">
<label class="ck" style="flex-direction:row;align-items:center;gap:6px"><input type="checkbox" id="eng"> Engineering mode (needed for the bars)</label>
<button id="rst">Reset peaks</button>
<span class="row" style="gap:6px"><button id="apply">Thresholds = peaks +</button><input type="number" id="margin" value="10" min="0" max="60" step="5" style="width:64px"></span>
<span id="engWarn" class="warn"></span>
</div>
<svg id="gates" viewBox="0 0 900 230" preserveAspectRatio="xMidYMid meet"></svg>
<div class="gates" id="gtab"></div>
<div class="leg"><span><i style="background:var(--mv)"></i>move energy</span><span><i style="background:var(--st)"></i>still energy</span><span><i style="background:#fff;height:2px"></i>threshold (drag it)</span><span><i style="background:#ffffff66;height:2px"></i>peak since reset</span><span>boxes: move / still threshold</span></div>
<div class="help">A gate only counts when its energy is above its threshold; 100 = that ring off. Recipe: leave the room (or sit still where you do NOT want detection), Reset peaks, wait a few minutes, then "Thresholds = peaks + margin" lifts every gate just above what it saw. Then lower the gates where you DO want to be seen if they ended up too high. Stand at least 1.5 m away while reading the bars - a target closer than ~0.5 m floods every gate.</div>
</div>

<script>
const $=id=>document.getElementById(id), N=9; let D=null, gl=75, peakM=Array(N).fill(0), peakS=Array(N).fill(0), drag=null, busy=0;
const numKeys=['a_near','a_far','b_near','b_far','edge','hold','timeout'], cmKeys=['a_near','a_far','b_near','b_far','edge']; /* v1.7: metres on the ESP and in the boxes; data.json sends these in cm */
let optGl=0; function buildMax(){ if(optGl===gl) return; optGl=gl; for (const k of ['maxm','maxs']) { const cur=$(k).value; $(k).innerHTML=''; for (let i=0;i<=8;i++) { const o=document.createElement('option'); o.value=i; o.textContent='gate '+i+' \u2192 '+((i+1)*gl/100).toFixed(2)+' m'; $(k).appendChild(o);} if(cur!=='') $(k).value=cur; } }
buildMax();
const room={w:400,d:400,mount:'wall'}; try{ Object.assign(room, JSON.parse(localStorage.getItem('radarRoom')||'{}')); }catch(e){}
$('room_w').value=room.w/100; $('room_d').value=room.d/100; $('mount').value=room.mount;
const saveRoom=()=>{ room.w=Math.round(+$('room_w').value*100)||400; room.d=Math.round(+$('room_d').value*100)||400; room.mount=$('mount').value; try{ localStorage.setItem('radarRoom',JSON.stringify(room)); }catch(e){} if(D) fan(); };
for (const k of ['room_w','room_d','mount']) $(k).addEventListener('change',saveRoom);
let suggMax=8; $('applyMax').onclick=async()=>{ await setv('maxm',suggMax); await new Promise(r=>setTimeout(r,200)); await setv('maxs',suggMax); };
/* v1.9 edit lock: opens locked; unlocked for 2 min after the last edit */
let unlocked=false, relockT=0;
const lockIds=()=>numKeys.concat(['maxm','maxs','eng','apply','applyMax']).concat([...Array(N).keys()].flatMap(i=>['gm'+i,'gs'+i]));
function applyLock(){ for(const id of lockIds()){ const el=$(id); if(el) el.disabled=!unlocked; }
  $('gates').style.touchAction=unlocked?'none':'auto'; $('gates').style.cursor=unlocked?'':'default';
  const b=$('lock'); b.classList.toggle('un',unlocked); b.innerHTML=unlocked?'&#128275; Editing &ndash; locks again 2 min after the last change (tap to lock)':'&#128274; Locked &ndash; tap to edit'; }
function setLock(u){ unlocked=u; clearTimeout(relockT); if(u) relockT=setTimeout(()=>{ drag=null; setLock(false); },120000); applyLock(); }
$('lock').onclick=()=>setLock(!unlocked);
const setv=async(k,v)=>{ if(!unlocked) return; setLock(true); busy++; try{ await fetch('/set?k='+k+'&v='+encodeURIComponent(v),{signal:AbortSignal.timeout(4000)});}catch(e){} busy--; };
for (const k of numKeys.concat(['maxm','maxs'])) $(k).addEventListener('change',e=>setv(k,e.target.value));
$('eng').addEventListener('change',e=>{ if(!unlocked){ e.target.checked=!e.target.checked; return; } setLock(true); fetch('/sw?k=eng&v='+(e.target.checked?1:0)); });
$('rst').onclick=()=>{peakM.fill(0);peakS.fill(0);};
$('apply').onclick=async()=>{ const m=+$('margin').value||0; for(let i=0;i<N;i++){ await setv('gm'+i,Math.min(100,Math.round(peakM[i]+m))); await new Promise(r=>setTimeout(r,150)); await setv('gs'+i,Math.min(100,Math.round(peakS[i]+m))); await new Promise(r=>setTimeout(r,150)); } };
const fmt=v=>v==null||isNaN(v)?'—':(v/100).toFixed(2)+' m';
function fan(){ const el=$('fan'), W=Math.max(280,Math.round(el.clientWidth||900)), R=Math.min(470,(W-44)/1.74), thin=R/N<30, H=Math.round(R+50), cx=W/2, cy=H-22, ax=N*gl, sc=R/ax, n=D.n, HALF=60*Math.PI/180; let s='';
  el.setAttribute('viewBox',`0 0 ${W} ${H}`); el.style.height=H+'px';
  const P=(r,a)=>[cx+r*Math.sin(a),cy-r*Math.cos(a)];           // a: 0 = straight ahead, +right
  const arc=(r,a0,a1)=>{ const [x0,y0]=P(r,a0),[x1,y1]=P(r,a1); return `${x0.toFixed(1)} ${y0.toFixed(1)} A ${r.toFixed(1)} ${r.toFixed(1)} 0 ${a1-a0>Math.PI?1:0} 1 ${x1.toFixed(1)} ${y1.toFixed(1)}`; };
  const wedge=(r0,r1)=>{ const [xa,ya]=P(r0,HALF),[xb,yb]=P(r0,-HALF); return `M ${arc(r1,-HALF,HALF)} L ${xa.toFixed(1)} ${ya.toFixed(1)} A ${r0.toFixed(1)} ${r0.toFixed(1)} 0 0 0 ${xb.toFixed(1)} ${yb.toFixed(1)} Z`; };
  s+=`<path d="${wedge(0.01,ax*sc)}" fill="#ffffff07" stroke="#ffffff22"/>`;
  // v1.4: live energy along the range - ring tint = still energy, blue band width = move energy
  for(let i=0;i<N;i++){ const gs=D.gs[i],gm=D.gm[i],r0=i*gl*sc,r1=(i+1)*gl*sc;
    if(gs!=null&&gs>2) s+=`<path d="${wedge(Math.max(0.01,r0),r1)}" fill="var(--st)" opacity="${(gs/100*0.5).toFixed(2)}"/>`;
    if(gm!=null&&gm>2) s+=`<path d="M ${arc((r0+r1)/2,-HALF,HALF)}" fill="none" stroke="var(--mv)" stroke-width="${(1+gm/100*(r1-r0)*0.8).toFixed(1)}" opacity=".7"/>`; }
  const mg=Math.max(+n.maxm,+n.maxs); if(mg<8) s+=`<path d="${wedge((mg+1)*gl*sc,ax*sc)}" fill="#000" opacity=".35"/>`;
  s+=`<path d="${wedge(Math.max(0.01,n.a_near*sc),Math.max(n.a_near+1,n.a_far)*sc)}" fill="var(--za)" opacity=".28"/>`;
  s+=`<path d="${wedge(Math.max(0.01,n.b_near*sc),Math.max(n.b_near+1,n.b_far)*sc)}" fill="var(--zb)" opacity=".28"/>`;
  for(let i=1;i<=N;i++){ const r=i*gl*sc; s+=`<path d="M ${arc(r,-HALF,HALF)}" fill="none" stroke="${i-1>mg?'#ffffff18':'#ffffff33'}"/>`; const [tx,ty]=P(r-gl*sc/2,-HALF-0.06); s+=`<text x="${tx}" y="${ty}" font-size="11" fill="#8b9bb0" text-anchor="end">g${i-1}</text>`; if(!thin||i%2==0){ const [lx,ly]=P(r,HALF+0.05); s+=`<text x="${lx}" y="${ly}" font-size="10" fill="#5e6e84">${(i*gl/100).toFixed(2)} m</text>`; } }
  // room box in sensor coordinates (x right, y forward, cm)
  let pts; if(room.mount==='corner'){ const u=[-0.7071,0.7071],v=[0.7071,0.7071]; pts=[[0,0],[u[0]*room.d,u[1]*room.d],[u[0]*room.d+v[0]*room.w,u[1]*room.d+v[1]*room.w],[v[0]*room.w,v[1]*room.w]]; }
  else pts=[[-room.w/2,0],[room.w/2,0],[room.w/2,room.d],[-room.w/2,room.d]];
  const far=Math.max(...pts.map(p=>Math.hypot(p[0],p[1]))); suggMax=Math.min(8,Math.max(0,Math.ceil(far/gl)-1));
  s+=`<polygon points="${pts.map(p=>(cx+p[0]*sc).toFixed(1)+','+(cy-p[1]*sc).toFixed(1)).join(' ')}" fill="none" stroke="#f0c060" stroke-width="2" stroke-dasharray="6 4"/>`;
  const drawArc=(d,col,w)=>{ const r=d*sc; if(r>2&&r<ax*sc) s+=`<path d="M ${arc(r,-HALF,HALF)}" fill="none" stroke="${col}" stroke-width="${w}" opacity=".9"/>`; };
  if(D.mv&&D.mdist>0) drawArc(D.mdist,'var(--mv)',3); if(D.st&&D.sdist>0) drawArc(D.sdist,'var(--st)',3);
  if(D.occ&&D.dist>0){ drawArc(D.dist,'#fff',4); const [tx,ty]=P(D.dist*sc+14,0); s+=`<text x="${tx}" y="${ty}" font-size="13" font-weight="700" fill="#fff" text-anchor="middle">${(D.dist/100).toFixed(2)} m</text>`; }
  s+=`<circle cx="${cx}" cy="${cy}" r="7" fill="#fff"/><text x="${cx}" y="${cy+16}" font-size="11" fill="#8b9bb0" text-anchor="middle">sensor</text>`;
  $('fan').innerHTML=s;
  const inRoom=Math.min(8,Math.ceil(far/gl)-1);
  $('roomTxt').innerHTML=`far corner <b>${(far/100).toFixed(2)} m</b> &rarr; room covers gates 0&ndash;${inRoom}; suggested max gate <b>${suggMax}</b> (${((suggMax+1)*gl/100).toFixed(2)} m)${mg>suggMax?` &mdash; currently ${mg}, scanning past the room`:''}`; }
function axis(){ const el=$('axis'), W=Math.max(280,Math.round(el.clientWidth||900)), H=96, L=14, R=W-14, ax=N*gl, X=c=>L+(R-L)*Math.min(c,ax)/ax, n=D.n; let s='';
  const mg=Math.max(+n.maxm,+n.maxs); s+=`<rect x="${X((mg+1)*gl)}" y="18" width="${R-X((mg+1)*gl)}" height="40" fill="#ffffff10"/>`;
  s+=`<rect x="${X(n.a_near)}" y="20" width="${Math.max(0,X(n.a_far)-X(n.a_near))}" height="16" rx="3" fill="var(--za)" opacity=".55"/><text x="${X(n.a_near)+4}" y="32" font-size="11" fill="#fff">A</text>`;
  s+=`<rect x="${X(n.b_near)}" y="40" width="${Math.max(0,X(n.b_far)-X(n.b_near))}" height="16" rx="3" fill="var(--zb)" opacity=".55"/><text x="${X(n.b_near)+4}" y="52" font-size="11" fill="#fff">B</text>`;
  for(let i=0;i<=N;i++){ const x=X(i*gl); s+=`<line x1="${x}" y1="14" x2="${x}" y2="64" stroke="#ffffff22"/><text x="${x}" y="78" font-size="10" fill="#8b9bb0" text-anchor="middle">${(i*gl/100).toFixed(gl==75?2:1)}</text>`; if(i<N) s+=`<text x="${X(i*gl+gl/2)}" y="92" font-size="10" fill="#5e6e84" text-anchor="middle">g${i}</text>`; }
  if(D.occ&&D.dist>0) s+=`<polygon points="${X(D.dist)-7},2 ${X(D.dist)+7},2 ${X(D.dist)},14" fill="#fff"/>`;
  if(D.mv&&D.mdist>0) s+=`<line x1="${X(D.mdist)}" y1="58" x2="${X(D.mdist)}" y2="66" stroke="var(--mv)" stroke-width="3"/>`;
  if(D.st&&D.sdist>0) s+=`<line x1="${X(D.sdist)}" y1="58" x2="${X(D.sdist)}" y2="66" stroke="var(--st)" stroke-width="3"/>`;
  el.setAttribute('viewBox',`0 0 ${W} ${H}`); $('axis').innerHTML=s; }
function gates(){ const el=$('gates'), W=Math.max(280,Math.round(el.clientWidth||900)), T=12, B=196, cw=W/N, narrow=cw<64, p=narrow?2:6, iw=cw-2*p, bw=iw*(narrow?.4:.38), Y=e=>B-(B-T)*Math.max(0,Math.min(100,e))/100; let s='';
  el.setAttribute('viewBox',`0 0 ${W} 236`);
  for(let i=0;i<N;i++){ const x0=i*cw+p, gm=D.gm[i], gs=D.gs[i], tm=+D.n['gm'+i], ts=+D.n['gs'+i], xm=x0+2, xs=x0+iw-2-bw;
    s+=`<rect x="${x0}" y="${T}" width="${iw}" height="${B-T}" fill="#ffffff06" stroke="#ffffff14"/>`;
    if(gm!=null) s+=`<rect x="${xm}" y="${Y(gm)}" width="${bw}" height="${B-Y(gm)}" fill="var(--mv)" opacity=".85"/>`;
    if(gs!=null) s+=`<rect x="${xs}" y="${Y(gs)}" width="${bw}" height="${B-Y(gs)}" fill="var(--st)" opacity=".85"/>`;
    s+=`<line x1="${xm-1}" y1="${Y(peakM[i])}" x2="${xm+bw+1}" y2="${Y(peakM[i])}" stroke="#ffffff66"/><line x1="${xs-1}" y1="${Y(peakS[i])}" x2="${xs+bw+1}" y2="${Y(peakS[i])}" stroke="#ffffff66"/>`;
    const dm=drag&&drag.i==i&&drag.k=='gm'?drag.v:tm, ds=drag&&drag.i==i&&drag.k=='gs'?drag.v:ts;
    s+=`<line x1="${xm-2}" y1="${Y(dm)}" x2="${xm+bw+2}" y2="${Y(dm)}" stroke="#fff" stroke-width="2"/><line x1="${xs-2}" y1="${Y(ds)}" x2="${xs+bw+2}" y2="${Y(ds)}" stroke="#fff" stroke-width="2"/>`;
    const vm=gm==null?'':Math.round(gm), vs=gs==null?'':Math.round(gs);
    if(narrow) s+=`<text x="${x0+iw/2}" y="${B+14}" font-size="11" fill="#7fbfff" text-anchor="middle">${vm}</text><text x="${x0+iw/2}" y="${B+28}" font-size="11" fill="#ffb877" text-anchor="middle">${vs}</text>`;
    else s+=`<text x="${x0+iw/2}" y="${B+16}" font-size="11" fill="#8b9bb0" text-anchor="middle">${vm} / ${vs}</text>`;
    s+=`<rect data-i="${i}" data-k="gm" x="${x0}" y="${T}" width="${iw/2}" height="${B-T}" fill="transparent" style="cursor:ns-resize"/><rect data-i="${i}" data-k="gs" x="${x0+iw/2}" y="${T}" width="${iw/2}" height="${B-T}" fill="transparent" style="cursor:ns-resize"/>`; }
  $('gates').innerHTML=s; }
let tabGl=0; function gtab(){ if($('gtab').children.length&&tabGl===gl) return; tabGl=gl; let s=''; for(let i=0;i<N;i++) s+=`<div><b>g${i}</b><span class="rg">${(i*gl/100).toFixed(2)}&ndash;${((i+1)*gl/100).toFixed(2)} m</span><span class="rs">&le;${((i+1)*gl/100).toFixed(2)}</span><input class="g" type="number" id="gm${i}" min="0" max="100" title="move threshold"><input class="g" type="number" id="gs${i}" min="0" max="100" title="still threshold"></div>`; $('gtab').innerHTML=s;
  for(let i=0;i<N;i++) for(const k of ['gm','gs']) $(k+i).addEventListener('change',e=>setv(k+i,e.target.value)); applyLock(); }
const sv=$('gates'); const yv=e=>{ const p=sv.createSVGPoint(); p.x=e.clientX; p.y=e.clientY; const y=p.matrixTransform(sv.getScreenCTM().inverse()).y; return Math.round(Math.max(0,Math.min(100,(196-y)/(196-12)*100))); };
sv.addEventListener('pointerdown',e=>{ const t=e.target; if(!unlocked||!t.dataset.k) return; drag={i:+t.dataset.i,k:t.dataset.k,v:yv(e)}; sv.setPointerCapture(e.pointerId); gates(); });
sv.addEventListener('pointermove',e=>{ if(!drag) return; drag.v=yv(e); gates(); });
sv.addEventListener('pointercancel',()=>{ drag=null; if(D) gates(); });
sv.addEventListener('pointerup',async e=>{ if(!drag) return; const d=drag; drag=null; $(d.k+d.i).value=d.v; await setv(d.k+d.i,d.v); });
function fill(id,v){ const el=$(id); if(document.activeElement===el||busy) return; if(el.type==='checkbox') el.checked=!!v; else el.value=v==null?'':cmKeys.includes(id)?+(v/100).toFixed(2):Math.round(v); }
function render(){ gl=(D.res||'').startsWith('0.2')?20:75; gtab(); buildMax();
  $('ttl').textContent=D.name+' - radar tuning'; document.title=D.name+' - radar tuning'; $('meta').textContent=`fw ${D.fw||'?'} · up ${Math.round((D.up||0)/60)} min · rssi ${D.rssi==null?'?':Math.round(D.rssi)} dBm`;
  $('dist').textContent=D.occ?fmt(D.dist):'no target'; $('men').textContent=D.men==null?'—':Math.round(D.men); $('sen').textContent=D.sen==null?'—':Math.round(D.sen);
  $('mdist').textContent=fmt(D.mdist); $('sdist').textContent=fmt(D.sdist);
  for(const [p,v] of [['pOcc',D.occ],['pMv',D.mv],['pSt',D.st],['pZa',D.za],['pZb',D.zb]]) $(p).classList.toggle('on',!!v);
  $('res').textContent=D.res||'?'; fill('eng',D.eng); $('engWarn').textContent=D.eng?'':'engineering mode is off - no gate energies';
  for(const k of numKeys.concat(['maxm','maxs'])) fill(k,D.n[k]); for(let i=0;i<N;i++){ fill('gm'+i,D.n['gm'+i]); fill('gs'+i,D.n['gs'+i]); if(D.gm[i]!=null) peakM[i]=Math.max(peakM[i],D.gm[i]); if(D.gs[i]!=null) peakS[i]=Math.max(peakS[i],D.gs[i]); }
  fan(); axis(); gates(); }
async function poll(){ try{ const r=await fetch('/data.json',{cache:'no-store',signal:AbortSignal.timeout(3000)}); D=await r.json(); render(); }catch(e){ $('meta').textContent='no answer from the device'; } }
$('rq').onclick=()=>fetch('/sw?k=query&v=1');
let rzT=0; addEventListener('resize',()=>{ clearTimeout(rzT); rzT=setTimeout(()=>{ if(D){ fan(); axis(); gates(); } },120); });
applyLock(); poll(); setInterval(poll,1000);
</script></div></body></html>
)ESPH";

static void rw_num(std::string &o, const char *k, float v) {
  char b[40]; if (std::isnan(v)) snprintf(b, sizeof b, "\"%s\":null,", k); else snprintf(b, sizeof b, "\"%s\":%.1f,", k, v); o += b;
}
static void rw_nument(std::string &o, const char *k, esphome::number::Number *n) { rw_num(o, k, (n && n->has_state()) ? n->state : NAN); }
// v1.7: zone numbers are metres on the ESP; data.json keeps cm because rw_num prints one decimal and the page draws in cm
static void rw_numcm(std::string &o, const char *k, esphome::number::Number *n) { rw_num(o, k, (n && n->has_state()) ? n->state * 100.0f : NAN); }
static void rw_bool(std::string &o, const char *k, esphome::binary_sensor::BinarySensor *b) { o += "\""; o += k; o += (b && b->state) ? "\":1," : "\":0,"; }

static esp_err_t rw_html(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  const char *p = RADAR_WEB_HTML; size_t rem = sizeof(RADAR_WEB_HTML) - 1;
  while (rem > 0) { size_t n = rem < 4096 ? rem : 4096; if (httpd_resp_send_chunk(req, p, (ssize_t) n) != ESP_OK) { httpd_resp_send_chunk(req, nullptr, 0); return ESP_FAIL; } p += n; rem -= n; }
  return httpd_resp_send_chunk(req, nullptr, 0);
}
static esp_err_t rw_health(httpd_req_t *req) { httpd_resp_set_type(req, "text/plain"); return httpd_resp_sendstr(req, "OK"); }

static esp_err_t rw_data(httpd_req_t *req) {
  auto &c = radar_web_cfg(); std::string o; o.reserve(1200); o += "{";
  o += "\"name\":\""; o += esphome::App.get_friendly_name(); o += "\",";
  rw_bool(o, "occ", c.occ); rw_bool(o, "mv", c.mv); rw_bool(o, "st", c.st); rw_bool(o, "za", c.za); rw_bool(o, "zb", c.zb);
  rw_num(o, "dist", rw_dist); rw_num(o, "mdist", rw_mdist); rw_num(o, "sdist", rw_sdist); rw_num(o, "men", rw_men); rw_num(o, "sen", rw_sen);
  const bool eng = c.eng && c.eng->state;
  o += "\"gm\":["; for (int i = 0; i < 9; i++) { char b[16]; if (!eng || std::isnan(rw_gm[i])) snprintf(b, sizeof b, "null,"); else snprintf(b, sizeof b, "%.0f,", rw_gm[i]); o += b; } o.pop_back(); o += "],";
  o += "\"gs\":["; for (int i = 0; i < 9; i++) { char b[16]; if (!eng || std::isnan(rw_gs[i])) snprintf(b, sizeof b, "null,"); else snprintf(b, sizeof b, "%.0f,", rw_gs[i]); o += b; } o.pop_back(); o += "],";
  o += "\"n\":{"; rw_numcm(o, "a_near", c.a_near); rw_numcm(o, "a_far", c.a_far); rw_numcm(o, "b_near", c.b_near); rw_numcm(o, "b_far", c.b_far);
  rw_numcm(o, "edge", c.edge); rw_nument(o, "hold", c.hold); rw_nument(o, "maxm", c.maxm); rw_nument(o, "maxs", c.maxs); rw_nument(o, "timeout", c.timeout);
  for (int i = 0; i < 9; i++) { char k[8]; snprintf(k, sizeof k, "gm%d", i); rw_nument(o, k, c.gmt[i]); snprintf(k, sizeof k, "gs%d", i); rw_nument(o, k, c.gst[i]); }
  o.pop_back(); o += "},";
  o += eng ? "\"eng\":1," : "\"eng\":0,";
  o += "\"res\":\""; o += rw_res; o += "\",";
  o += "\"fw\":\""; o += rw_fw; o += "\",";
  rw_num(o, "up", (c.uptime && c.uptime->has_state()) ? c.uptime->state : NAN); rw_num(o, "rssi", (c.rssi && c.rssi->has_state()) ? c.rssi->state : NAN);
  o.pop_back(); o += "}";
  httpd_resp_set_type(req, "application/json"); httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, o.c_str(), o.size());
}

static bool rw_query(httpd_req_t *req, char *k, size_t kl, char *v, size_t vl) {
  char q[96]; if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK) return false;
  return httpd_query_key_value(q, "k", k, kl) == ESP_OK && httpd_query_key_value(q, "v", v, vl) == ESP_OK;
}
static esphome::number::Number *rw_find(const char *k) {
  auto &c = radar_web_cfg();
  struct { const char *k; esphome::number::Number *n; } t[] = {{"a_near", c.a_near}, {"a_far", c.a_far}, {"b_near", c.b_near}, {"b_far", c.b_far}, {"edge", c.edge}, {"hold", c.hold}, {"maxm", c.maxm}, {"maxs", c.maxs}, {"timeout", c.timeout}};
  for (auto &e : t) if (strcmp(k, e.k) == 0) return e.n;
  if ((k[0] == 'g') && (k[1] == 'm' || k[1] == 's') && k[2] >= '0' && k[2] <= '8' && k[3] == 0) return k[1] == 'm' ? c.gmt[k[2] - '0'] : c.gst[k[2] - '0'];
  return nullptr;
}
static esp_err_t rw_set(httpd_req_t *req) {
  char k[16], v[16]; httpd_resp_set_type(req, "text/plain");
  if (!rw_query(req, k, sizeof k, v, sizeof v)) return httpd_resp_sendstr(req, "ERR query");
  auto *n = rw_find(k); if (!n) return httpd_resp_sendstr(req, "ERR key");
  float f = strtof(v, nullptr); if (std::isnan(f)) return httpd_resp_sendstr(req, "ERR value");
  return httpd_resp_sendstr(req, rw_push(k, f) ? "queued" : "ERR busy");
}
static esp_err_t rw_sw(httpd_req_t *req) {
  char k[16], v[16]; httpd_resp_set_type(req, "text/plain"); auto &c = radar_web_cfg();
  if (!rw_query(req, k, sizeof k, v, sizeof v)) return httpd_resp_sendstr(req, "ERR query");
  if (strcmp(k, "query") == 0) { if (!c.query) return httpd_resp_sendstr(req, "ERR key"); return httpd_resp_sendstr(req, rw_push("!query", 1) ? "queued" : "ERR busy"); }
  if (strcmp(k, "eng") != 0 || !c.eng) return httpd_resp_sendstr(req, "ERR key");
  return httpd_resp_sendstr(req, rw_push("!eng", v[0] == '1' ? 1 : 0) ? "queued" : "ERR busy");
}

// main loop (100 ms yaml interval): perform what the httpd task queued
static void radar_web_pump() {
  auto &c = radar_web_cfg();
  while (rw_qh != rw_qt) {
    RwCmd cmd;
    taskENTER_CRITICAL(&rw_mux); cmd = rw_q[rw_qh]; rw_qh = (rw_qh + 1) & 7; taskEXIT_CRITICAL(&rw_mux);
    if (strcmp(cmd.k, "!query") == 0) { if (c.query) c.query->press(); continue; }
    if (strcmp(cmd.k, "!eng") == 0) { if (c.eng) { if (cmd.v > 0.5f) c.eng->turn_on(); else c.eng->turn_off(); } continue; }
    auto *n = rw_find(cmd.k); if (!n) continue;
    ESP_LOGI("radar_web", "set %s = %.1f", cmd.k, cmd.v);
    n->make_call().set_value(cmd.v).perform();
  }
}

// call from a yaml lambda after filling radar_web_cfg(); safe to call every few seconds (returns once running)
static void radar_web_register() {
  static bool registered = false, hooked = false; if (registered) return;
  auto &c = radar_web_cfg();
  if (!hooked) {
    hooked = true;
    for (int i = 0; i < 9; i++) { rw_gm[i] = NAN; rw_gs[i] = NAN; }
    if (c.dist) c.dist->add_on_raw_state_callback([](float x) { rw_dist = x; });
    if (c.mdist) c.mdist->add_on_raw_state_callback([](float x) { rw_mdist = x; });
    if (c.sdist) c.sdist->add_on_raw_state_callback([](float x) { rw_sdist = x; });
    if (c.men) c.men->add_on_raw_state_callback([](float x) { rw_men = x; });
    if (c.sen) c.sen->add_on_raw_state_callback([](float x) { rw_sen = x; });
    for (int i = 0; i < 9; i++) {
      if (c.gm[i]) c.gm[i]->add_on_raw_state_callback([i](float x) { rw_gm[i] = x; });
      if (c.gs[i]) c.gs[i]->add_on_raw_state_callback([i](float x) { rw_gs[i] = x; });
    }
    if (c.fw) { c.fw->add_on_state_callback([](const std::string &x) { strlcpy(rw_fw, x.c_str(), sizeof rw_fw); }); if (c.fw->has_state()) strlcpy(rw_fw, c.fw->state.c_str(), sizeof rw_fw); }
    if (c.res) { c.res->add_on_state_callback([](size_t) { auto &r = radar_web_cfg(); if (r.res) strlcpy(rw_res, r.res->current_option().c_str(), sizeof rw_res); }); if (c.res->has_state()) strlcpy(rw_res, c.res->current_option().c_str(), sizeof rw_res); }
  }
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  // ctrl_port must be a real port: 0 binds an ephemeral one but httpd still sends its work queue to port 0, so the LRU
  // close never runs. With all 5 slots taken it then refuses every client and spins on the listen socket.
  cfg.server_port = 8080; cfg.ctrl_port = 32769; cfg.stack_size = 8192; cfg.max_uri_handlers = 6; cfg.max_open_sockets = 5; cfg.lru_purge_enable = true;
  cfg.keep_alive_enable = true; cfg.keep_alive_idle = 10; cfg.keep_alive_interval = 5; cfg.keep_alive_count = 3;   // a vanished browser frees its slot
  httpd_handle_t srv = nullptr; esp_err_t err = httpd_start(&srv, &cfg);
  if (err != ESP_OK) { ESP_LOGE("radar_web", "httpd_start failed: %s - will retry", esp_err_to_name(err)); return; }
  registered = true;
  static const httpd_uri_t uris[] = {
    {"/", HTTP_GET, rw_html, nullptr}, {"/data.json", HTTP_GET, rw_data, nullptr}, {"/set", HTTP_GET, rw_set, nullptr},
    {"/sw", HTTP_GET, rw_sw, nullptr}, {"/healthz", HTTP_GET, rw_health, nullptr},
  };
  for (const auto &u : uris) httpd_register_uri_handler(srv, &u);
  ESP_LOGI("radar_web", "tuning page at http://<device-ip>:8080/");
}
