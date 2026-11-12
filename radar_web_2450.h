#pragma once
// ─────────────────────────────────────────────────
//  NU-Presence-24GHz-2  LD2450 tuning page  v1.7  (radar_web_2450.h)  2026-11-08
//
//  Same structure as radar_web.h (24GHz-1): an IDF httpd on port 8080, one embedded page, small JSON endpoints.
//
//    /            live top-down map: up to 3 targets with trails, room box, zones you drag out with the mouse
//    /data.json   live state + settings, polled twice a second
//    /set?k=..&v= write a number (z1x1..z3y2 in mm, timeout, hold, mvs = Moving speed cm/s, mvr = Moving range m, mvh = Moving hold s, sth = Still hold s, igf / igt = Ignore from / to m)
//    /sw?k=multi|zt&v=  multi target on/off, zone type 0/1/2
//    /healthz     "OK"
//
//  Threading: the httpd runs in its own task. It only READS sensor raw values and flags; every WRITE goes into a
//  queue that radar_web_pump() performs on the main loop. Calling entity actions from the httpd task crashed
//  24GHz-1 (the driver shares the UART with the loop).
//  Entities are handed over from a yaml lambda, because a header included before main.cpp cannot use id().
//
//  v1.1 (after a Task WDT crash that rolled a build back): the ESP32-C3 has one core, ESPHome's loop runs at
//  priority 1 and feeds a 5 s task watchdog, and IDF's httpd task defaults to priority 5, so any busy spin in
//  httpd_server() (select() failing, a stale socket) starves the loop and panics. The httpd now runs at priority 1
//  (time-sliced with the loop) and is not started until 70 s after boot, past safe_mode's 60 s "boot is good"
//  mark, so a crash in it can reboot but can never roll the firmware back.
//
//  v1.2 (yaml v1.6): "Moving target" card. Moving target is a software detector since yaml v1.5 (the LD2450 has no
//  sensitivity setting): speed / range / hold numbers, now editable here with presets and a live "fastest target"
//  readout so the numbers can be chosen by watching. The map draws the Moving range arc and fills a target only when
//  it counts as moving. data.json also carries the radar's raw moving flag (mvraw) for comparison. Numbers are
//  printed with %g (the range has a decimal). Less page work per poll: the map is redrawn only when something
//  on it changed.
//
//  v1.3 (yaml v1.8): "Still presence" card with the Still hold box, and the presence tracks in data.json (trk) and on
//  the map. A target the radar reports but no track owns is drawn grey: it never moved, so it does not count.
//
//  v1.5 (yaml v1.10): current preset on both cards (data.json ps / pm) and in HA, and radar_web_state() for the "Web
//  page" entity. The names live here only; the page matches them against its button text, so keep the two the same.
//  The values are also in the page script. Moving preset ignores range: the page sets that from its room box.
// ─────────────────────────────────────────────────
#include "esphome.h"
#include "presence_2450.h"
#include "esp_http_server.h"
#include <string>
#include <cstring>
#include <cstdlib>
#include <cmath>

struct RadarWebCfg {
  esphome::binary_sensor::BinarySensor *occ = nullptr, *mv = nullptr, *st = nullptr, *zn[3] = {};
  esphome::sensor::Sensor *tx[3] = {}, *ty[3] = {}, *ts[3] = {}, *count = nullptr, *uptime = nullptr, *rssi = nullptr;
  esphome::number::Number *zc[3][4] = {}, *timeout = nullptr, *hold = nullptr, *mvs = nullptr, *mvr = nullptr, *mvh = nullptr, *sth = nullptr, *igf = nullptr, *igt = nullptr;
  esphome::binary_sensor::BinarySensor *mvraw = nullptr, *occraw = nullptr;
  esphome::switch_::Switch *multi = nullptr;
  esphome::select::Select *zt = nullptr;
  esphome::text_sensor::TextSensor *fw = nullptr;
};
static RadarWebCfg &radar_web_cfg() { static RadarWebCfg c; return c; }

static char rw_fw[32] = "";
static int rw_zt = -1;

// write queue: httpd task -> main loop
struct RwCmd { char k[12]; float v; };
static RwCmd rw_q[16];
static volatile uint8_t rw_qh = 0, rw_qt = 0;
static portMUX_TYPE rw_mux = portMUX_INITIALIZER_UNLOCKED;
static bool rw_push(const char *k, float v) {
  bool ok = false;
  taskENTER_CRITICAL(&rw_mux);
  uint8_t nt = (rw_qt + 1) & 15;
  if (nt != rw_qh) { strlcpy(rw_q[rw_qt].k, k, sizeof rw_q[rw_qt].k); rw_q[rw_qt].v = v; rw_qt = nt; ok = true; }
  taskEXIT_CRITICAL(&rw_mux);
  return ok;
}

// which preset the numbers match
static const char *rw_still_preset(float v) {
  if (std::isnan(v)) return "Custom";
  switch ((int) lroundf(v)) {
    case 60: return "Hallway 60 s";
    case 300: return "Desk 300 s";
    case 900: return "Couch or TV 900 s";
    case 1800: return "Bed 1800 s";
    default: return "Custom";
  }
}
static const char *rw_move_preset(float speed, float hold) {
  if (std::isnan(speed) || std::isnan(hold)) return "Custom";
  int s = (int) lroundf(speed), h = (int) lroundf(hold);
  if (s == 5 && h == 10) return "Small movements";
  if (s == 10 && h == 5) return "Default";
  if (s == 25 && h == 5) return "Walking only";
  if (s == 10 && h == 30) return "Hold for lights";
  return "Custom";
}

// 0 not started yet (70 s after boot), 1 serving, 2 httpd failed
static volatile int rw_page = 0;
static const char *radar_web_state() { return rw_page == 1 ? "Ready" : rw_page == 2 ? "Failed" : "Starting"; }

static const char RADAR_WEB_HTML[] PROGMEM = R"ESPH(<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Radar map</title>
<style>
:root{--bg:#10151c;--card:#182029;--line:#243040;--tx:#e6edf3;--mut:#8b9bb0;--t0:#4aa3ff;--t1:#ff9f43;--t2:#34d399;--z0:#34d399;--z1:#60a5fa;--z2:#f0a0e0;--bad:#f87171}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif;padding:12px 16px 32px}
html,body{overflow-x:hidden}.wrap{max-width:1100px;margin:0 auto;min-width:0}
h1{font-size:18px;margin:0 0 4px}h2{font-size:13px;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;margin:18px 0 8px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
.row{display:flex;flex-wrap:wrap;gap:10px 18px;align-items:center}
.pill{display:inline-block;padding:2px 9px;border-radius:999px;background:#2a3645;color:var(--mut);font-weight:600;font-size:12px}
.pill.on{background:#1e5a3f;color:#9ff0c9}
.big{font-size:34px;font-weight:800;line-height:1}.sub{color:var(--mut);font-size:12px}
label{color:var(--mut);font-size:12px;display:flex;flex-direction:column;gap:3px}label.ck{flex-direction:row;align-items:center;gap:6px;font-size:13px}
input[type=number],select{background:#0d1218;color:var(--tx);border:1px solid var(--line);border-radius:6px;padding:5px 7px;font-size:14px;width:84px}select{width:auto;min-width:132px}
button{background:#263445;color:var(--tx);border:1px solid #35465b;border-radius:7px;padding:6px 11px;font-size:13px;cursor:pointer}button:hover{background:#31435a}button.on{background:#1e5a3f;border-color:#2f7d5a}
svg{display:block;width:100%;height:520px;touch-action:none;user-select:none}
table{border-collapse:collapse;width:100%;font-size:13px}th{color:var(--mut);font-weight:600;text-align:left;font-size:12px}td,th{padding:3px 8px 3px 0}td input{width:72px}
.help{color:var(--mut);font-size:12px;margin-top:8px;line-height:1.5}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:5px}
@media (max-width:640px){body{padding:8px 8px 24px;font-size:13px}h1{font-size:16px}.card{padding:10px}.big{font-size:28px}svg{height:360px}.row{gap:8px 10px}
 .row>label:not(.ck){flex:1 1 calc(33% - 10px);min-width:90px}.row>label input,.row>label select{width:100%;min-width:0}td input{width:58px;padding:5px 4px}button{min-height:38px}}
</style></head><body><div class="wrap">
<h1 id="ttl">Radar map</h1><div class="sub" id="meta">connecting&hellip;</div>

<h2>Live</h2><div class="card"><div class="row">
<div><div class="big" id="cnt">&mdash;</div><div class="sub">targets</div></div>
<div><span class="pill" id="pOcc">Occupied</span> <span class="pill" id="pMv">Moving</span> <span class="pill" id="pSt">Still</span> <span class="pill" id="pRaw" title="the radar module's own moving flag, held for the whole Clear delay">radar raw moving</span> <span class="pill" id="pRawO" title="the radar reports a target right now, person or ghost">radar raw target</span></div>
<div><span class="pill" id="pZ0">Zone 1</span> <span class="pill" id="pZ1">Zone 2</span> <span class="pill" id="pZ2">Zone 3</span></div>
</div>
<table style="margin-top:8px"><thead><tr><th>Target</th><th>Left / right</th><th>Forward</th><th>Distance</th><th>Angle</th><th>Speed</th></tr></thead><tbody id="tt"></tbody></table>
</div>

<h2>Room map</h2><div class="card">
<svg id="map"></svg>
<div class="row" style="margin-top:8px">
<span class="sub">Draw:</span><button id="d0">Zone 1</button><button id="d1">Zone 2</button><button id="d2">Zone 3</button>
<label class="ck"><input type="checkbox" id="trail" checked>trails</label>
<label class="ck"><input type="checkbox" id="flip">mirror left / right</label>
</div>
<div class="row" style="margin-top:8px">
<label>Room width m<input type="number" id="room_w" min="1" max="20" step="0.1"></label>
<label>Room depth m<input type="number" id="room_d" min="1" max="20" step="0.1"></label>
<label>Sensor mounted<select id="mount"><option value="wall">middle of the near wall</option><option value="corner">in a corner, pointing across</option></select></label>
</div>
<div class="help">Top-down, sensor at the bottom, about 120&deg; wide, rings every metre. Pick a zone, then drag a rectangle on the map. If someone walking to their right moves left on the map, tick mirror. The room box is only for your eyes and is saved in this browser.</div>
</div>

<h2>Still presence</h2><div class="card">
<div class="row">
<div><div class="big" id="prN">&mdash;</div><div class="sub">people tracked</div></div>
<div><div class="big" id="prS" style="font-size:22px;line-height:34px">&mdash;</div><div class="sub" id="prL">radar status</div></div>
<label>Still hold s<input type="number" id="sth" min="30" max="3600" step="10"></label>
<label>Ignore from m<input type="number" id="igf" min="0" max="7.5" step="0.05"></label>
<label>Ignore to m<input type="number" id="igt" min="0" max="7.5" step="0.05"></label>
<div><div class="big" id="phC" style="font-size:22px;line-height:34px">&mdash;</div><div class="sub">current preset</div></div>
</div>
<div class="row" style="margin-top:10px"><span class="sub">Presets:</span>
<button id="ph0">Hallway 60 s</button><button id="ph1">Desk 300 s</button><button id="ph2">Couch or TV 900 s</button><button id="ph3">Bed 1800 s</button>
</div>
<div class="help">This radar cannot see breathing, so someone sitting still fades out of it for a minute or more. To leave a room a person has to move,
so a person who disappears <b>without moving first</b> is taken to be still there, for <b>Still hold</b> seconds. A person who was moving right up to
the moment they disappeared has walked out, and clears after <b>Clear delay</b> instead. The wait restarts each time the radar finds them again at the same spot.<br>
<b>Still hold</b>: set it longer than the longest gap you see while sitting. 60 s suits a hallway; 300 s (default) a desk; 900 s a couch or TV; 1800 s a bed.
Longer never affects someone who walks out normally, it only sets how long Occupied can stay on if they slip out unseen.<br>
A target only counts once it has moved, so a grey dot on the map is a return that never moved: the radar reports it, Occupied ignores it. That also means
that after a power cut a seated person is not counted until they move; a restart or a flash keeps them. <b>Moving speed</b> and <b>Moving range</b> below decide what counts as moving, so they
affect Occupied too: a high Moving speed makes it slower to notice someone.<br>
<b>Ignore from / to</b>: targets this far from the sensor (the red band on the map) are not counted at all, so a reflection that sits there cannot
hold Occupied on. The office has one at about 4.4 m, so the default is 3.75 to 5.05 m. Anyone in the band is picked up again as soon as they step out of it.
Set both to 0 to turn it off.</div>
</div>

<h2>Moving target</h2><div class="card">
<div class="row">
<div><div class="big" id="spdNow">&mdash;</div><div class="sub">fastest target now, cm/s</div></div>
<div><div class="big" id="spdPk">&mdash;</div><div class="sub">peak, last 30 s</div></div>
<div><div class="big" id="pmC" style="font-size:22px;line-height:34px">&mdash;</div><div class="sub">current preset</div></div>
<div><span class="pill" id="pMv2">Moving target</span></div>
</div>
<div class="row" style="margin-top:10px">
<label>Moving speed cm/s<input type="number" id="mvs" min="1" max="100" step="1"></label>
<label>Moving range m<input type="number" id="mvr" min="0.5" max="7.5" step="0.1"></label>
<label>Moving hold s<input type="number" id="mvh" min="1" max="120" step="1"></label>
</div>
<div class="row" style="margin-top:10px"><span class="sub">Presets:</span>
<button id="pr0" title="speed 5, range = room, hold 10">Small movements</button>
<button id="pr1" title="speed 10, range = room, hold 5">Default</button>
<button id="pr2" title="speed 25, range = room, hold 5">Walking only</button>
<button id="pr3" title="speed 10, range = room, hold 30">Hold for lights</button>
</div>
<div class="help">The radar has no sensitivity setting, so this is it. Moving target = any target closer than <b>range</b>
moving at <b>speed</b> or faster, kept on for <b>hold</b> seconds after the last such movement. The LED is blue exactly while it is on.
The speed is measured towards or away from the sensor only: walking straight across its view reads low, so sit, stand up and walk
about while watching the numbers above and set <b>speed</b> a little under what you want it to catch.<br>
<b>Speed</b>: 3&ndash;5 catches a hand reaching for a cup or typing; 10 (default) catches shifting in a seat and standing up;
20&ndash;30 reacts to walking only; a slow walk towards the sensor is about 50.<br>
<b>Range</b>: set it to the far wall (the presets use the room box above). Anything past it is ignored, which drops the
ghost returns from the hallway or the next room that used to keep the radar's own moving flag on. Current preset goes by speed and hold only, because a preset's range comes from the room box in your browser.<br>
<b>Hold</b>: 2&ndash;3 s for a snappy LED, 5 s (default) for a calm one, 15&ndash;30 s if an automation keeps the lights on with Moving target.</div>
</div>

<h2>Zones</h2><div class="card">
<table><thead><tr><th>Zone</th><th>X1 m</th><th>Y1 m</th><th>X2 m</th><th>Y2 m</th><th></th></tr></thead><tbody id="zt"></tbody></table>
<div class="row" style="margin-top:10px">
<label>Zone type<select id="ztype"><option value="0">Disabled</option><option value="1">Detection</option><option value="2">Filter</option></select></label>
<label>Clear delay s<input type="number" id="timeout" min="0" max="3600" step="1"></label>
<label>Zone hold s<input type="number" id="hold" min="0" max="600" step="5"></label>
<label class="ck"><input type="checkbox" id="multi">multi target</label>
</div>
<div class="help">Disabled: zones just report who is inside them. Detection: the radar only sees targets inside the zones. Filter: it ignores targets inside them, for a fan or a curtain. Multi target off tracks one person, which is steadier.</div>
</div>
</div>

<script>
const $=id=>document.getElementById(id), R=6000, HALF=Math.PI/3, TC=['var(--t0)','var(--t1)','var(--t2)'], ZC=['var(--z0)','var(--z1)','var(--z2)'];
let D=null, busy=0, draw=-1, rub=null, geo=null, trails=[[],[],[]];
const store={w:4,d:4,mount:'wall',flip:false,trail:true}; try{Object.assign(store,JSON.parse(localStorage.getItem('radar2')||'{}'));}catch(e){}
const save=()=>{try{localStorage.setItem('radar2',JSON.stringify(store));}catch(e){}};
$('room_w').value=store.w;$('room_d').value=store.d;$('mount').value=store.mount;$('flip').checked=store.flip;$('trail').checked=store.trail;
for(const k of ['room_w','room_d','mount','flip','trail']) $(k).addEventListener('change',()=>{store.w=+$('room_w').value||4;store.d=+$('room_d').value||4;store.mount=$('mount').value;store.flip=$('flip').checked;store.trail=$('trail').checked;save();if(D)map();});
const setv=async(k,v)=>{busy++;try{await fetch('/set?k='+k+'&v='+encodeURIComponent(v),{signal:AbortSignal.timeout(4000)});}catch(e){}busy--;};
const setw=async(k,v)=>{busy++;try{await fetch('/sw?k='+k+'&v='+v,{signal:AbortSignal.timeout(4000)});}catch(e){}busy--;};
const m2=v=>v==null?'—':(v/1000).toFixed(2)+' m';
function fill(id,v,f){const el=$(id);if(document.activeElement===el||busy)return;if(el.type==='checkbox')el.checked=!!v;else el.value=v==null?'':(f?f(v):v);}
function setZone(i,z){z.forEach((v,j)=>setv('z'+(i+1)+'xyxy'[j]+(j<2?1:2),Math.round(v)));}
for(let i=0;i<3;i++){$('d'+i).addEventListener('click',()=>{draw=draw===i?-1:i;for(let j=0;j<3;j++)$('d'+j).classList.toggle('on',j===draw);});}
let tb='';for(let i=0;i<3;i++){tb+=`<tr><td><span class="dot" style="background:${ZC[i]}"></span>Zone ${i+1}</td>`;for(let j=0;j<4;j++)tb+=`<td><input type="number" id="z${i}${j}" step="0.05" min="-6" max="8"></td>`;tb+=`<td><button id="zc${i}">Clear</button></td></tr>`;}$('zt').innerHTML=tb;
for(let i=0;i<3;i++){for(let j=0;j<4;j++)$('z'+i+j).addEventListener('change',e=>setv('z'+(i+1)+'xyxy'[j]+(j<2?1:2),Math.round(e.target.value*1000)));$('zc'+i).addEventListener('click',()=>setZone(i,[0,0,0,0]));}
$('ztype').addEventListener('change',e=>setw('zt',e.target.value));$('multi').addEventListener('change',e=>setw('multi',e.target.checked?1:0));
for(const k of ['timeout','hold','mvs','mvr','mvh','sth','igf','igt'])$(k).addEventListener('change',e=>setv(k,e.target.value));
[60,300,900,1800].forEach((v,i)=>$('ph'+i).addEventListener('click',()=>setv('sth',v)));
const owned=t=>D&&(D.trk||[]).some(k=>Math.hypot(k[0]-t[0],k[1]-t[1])<1000);const mmss=s=>Math.floor(s/60)+':'+String(Math.round(s%60)).padStart(2,'0');
const roomRange=()=>Math.min(7.5,Math.max(0.5,Math.round((store.mount==='corner'?Math.hypot(store.w,store.d):store.d)*10)/10));
[[5,10],[10,5],[25,5],[10,30]].forEach((p,i)=>$('pr'+i).addEventListener('click',async()=>{await setv('mvs',p[0]);await setv('mvr',roomRange());await setv('mvh',p[1]);}));
let spk=[];const mvQual=t=>t&&t[1]!=null&&D&&D.n.mvr!=null&&D.n.mvs!=null&&t[1]<=D.n.mvr*1000&&Math.abs(t[2]||0)>=D.n.mvs;
function roomPts(){const rw=store.w*1000,rd=store.d*1000;if(store.mount==='corner'){const u=[-0.7071,0.7071],v=[0.7071,0.7071];return[[0,0],[u[0]*rd,u[1]*rd],[u[0]*rd+v[0]*rw,u[1]*rd+v[1]*rw],[v[0]*rw,v[1]*rw]];}return[[-rw/2,0],[rw/2,0],[rw/2,rd],[-rw/2,rd]];}
function map(){const el=$('map'),pts=roomPts(),W=Math.max(280,Math.round(el.clientWidth||900)),H=Math.round(el.clientHeight||520),pad=26,hw=Math.max(R*Math.sin(HALF),...pts.map(p=>Math.abs(p[0]))),hy=Math.max(R,...pts.map(p=>p[1])),sc=Math.min((W-2*pad)/(2*hw),(H-2*pad)/hy),cx=W/2,cy=H-pad,fl=store.flip?-1:1;
 geo={sc,cx,cy,fl};const X=x=>cx+fl*x*sc,Y=y=>cy-y*sc;let s='';
 const pt=(r,a)=>[cx+r*sc*Math.sin(a),cy-r*sc*Math.cos(a)];
 const [ax,ay]=pt(R,-HALF),[bx,by]=pt(R,HALF);s+=`<path d="M ${cx} ${cy} L ${ax} ${ay} A ${R*sc} ${R*sc} 0 0 1 ${bx} ${by} Z" fill="#ffffff07" stroke="#ffffff22"/>`;
 for(let m=1;m<=6;m++){const r=m*1000,[p,q]=pt(r,-HALF),[u,v]=pt(r,HALF);s+=`<path d="M ${p} ${q} A ${r*sc} ${r*sc} 0 0 1 ${u} ${v}" fill="none" stroke="#ffffff18"/><text x="${u+4}" y="${v}" font-size="10" fill="#6d7d92">${m} m</text>`;}
 if(D&&D.n.igf!=null&&D.n.igt!=null&&D.n.igt>D.n.igf){const r0=D.n.igf*1000,r1=Math.min(D.n.igt*1000,R),[a0,b0]=pt(r0,-HALF),[a1,b1]=pt(r0,HALF),[c0,d0]=pt(r1,HALF),[c1,d1]=pt(r1,-HALF);if(r1>r0)s+=`<path d="M ${a0} ${b0} A ${r0*sc} ${r0*sc} 0 0 1 ${a1} ${b1} L ${c0} ${d0} A ${r1*sc} ${r1*sc} 0 0 0 ${c1} ${d1} Z" fill="#f8717126" stroke="#f8717166"/><text x="${c0+4}" y="${d0+12}" font-size="10" fill="#f87171">ignored ${D.n.igf}-${D.n.igt} m</text>`;}
 if(D&&D.n.mvr!=null){const yr=Y(D.n.mvr*1000),x0=X(-R*Math.sin(HALF)),x1=X(R*Math.sin(HALF));s+=`<line x1="${Math.min(x0,x1)}" y1="${yr}" x2="${Math.max(x0,x1)}" y2="${yr}" stroke="#4aa3ff" stroke-dasharray="5 4" opacity=".7"/><text x="${Math.min(x0,x1)+4}" y="${yr-4}" font-size="10" fill="#4aa3ff">moving range ${D.n.mvr} m</text>`;}
 D&&D.zc.forEach((z,i)=>{if(!z||(!z[0]&&!z[1]&&!z[2]&&!z[3]))return;const x0=Math.min(X(z[0]),X(z[2])),x1=Math.max(X(z[0]),X(z[2])),y0=Math.min(Y(z[1]),Y(z[3])),y1=Math.max(Y(z[1]),Y(z[3]));s+=`<rect x="${x0}" y="${y0}" width="${x1-x0}" height="${y1-y0}" fill="${ZC[i]}" opacity="${D.z[i]?.35:.16}" stroke="${ZC[i]}"/><text x="${x0+4}" y="${y0+13}" font-size="11" fill="#fff">Zone ${i+1}</text>`;});
 s+=`<polygon points="${pts.map(p=>X(p[0]).toFixed(1)+','+Y(p[1]).toFixed(1)).join(' ')}" fill="none" stroke="#f0c060" stroke-width="2" stroke-dasharray="6 4"/>`;
 if(store.trail)trails.forEach((t,i)=>{if(t.length>1)s+=`<polyline points="${t.map(p=>X(p[0]).toFixed(1)+','+Y(p[1]).toFixed(1)).join(' ')}" fill="none" stroke="${TC[i]}" stroke-width="2" opacity=".45"/>`;});
 D&&(D.trk||[]).forEach(k=>{if(k[2]==0)return;const x=X(k[0]),y=Y(k[1]);s+=`<circle cx="${x}" cy="${y}" r="13" fill="none" stroke="#f0c060" stroke-width="2" stroke-dasharray="4 3"/><text x="${x+17}" y="${y+4}" font-size="11" fill="#f0c060">${k[2]==1?'held':'leaving'} ${mmss(k[3])}</text>`;});
 D&&D.t.forEach((t,i)=>{if(!t||t[1]==null)return;const x=X(t[0]),y=Y(t[1]),mv=mvQual(t),c=owned(t)?TC[i]:'#6d7d92';s+=`<circle cx="${x}" cy="${y}" r="9" fill="${mv?c:'none'}" stroke="${c}" stroke-width="3"/><text x="${x+13}" y="${y+4}" font-size="12" font-weight="700" fill="#fff">${i+1} &middot; ${(Math.hypot(t[0],t[1])/1000).toFixed(2)} m</text>`;});
 if(rub)s+=`<rect x="${Math.min(rub[0],rub[2])}" y="${Math.min(rub[1],rub[3])}" width="${Math.abs(rub[2]-rub[0])}" height="${Math.abs(rub[3]-rub[1])}" fill="${ZC[draw]}" opacity=".3" stroke="#fff" stroke-dasharray="4 3"/>`;
 s+=`<circle cx="${cx}" cy="${cy}" r="7" fill="#fff"/><text x="${cx}" y="${cy+18}" font-size="11" fill="#8b9bb0" text-anchor="middle">sensor</text>`;
 if(s===map.last&&W===map.W&&H===map.H)return;map.last=s;map.W=W;map.H=H;el.setAttribute('viewBox',`0 0 ${W} ${H}`);el.innerHTML=s;}
const sv=$('map');const pos=e=>{const r=sv.getBoundingClientRect();return[e.clientX-r.left,e.clientY-r.top];};
sv.addEventListener('pointerdown',e=>{if(draw<0)return;const[p,q]=pos(e);rub=[p,q,p,q];sv.setPointerCapture(e.pointerId);map();});
sv.addEventListener('pointermove',e=>{if(!rub)return;const[p,q]=pos(e);rub[2]=p;rub[3]=q;map();});
sv.addEventListener('pointerup',()=>{if(!rub||!geo)return;const{sc,cx,cy,fl}=geo,mm=(p,q)=>[Math.round(fl*(p-cx)/sc/10)*10,Math.round((cy-q)/sc/10)*10],a=mm(rub[0],rub[1]),b=mm(rub[2],rub[3]);rub=null;
 if(Math.abs(a[0]-b[0])>=200&&Math.abs(a[1]-b[1])>=200){const y0=Math.max(0,Math.min(a[1],b[1])),y1=Math.max(a[1],b[1]);setZone(draw,[Math.min(a[0],b[0]),y0,Math.max(a[0],b[0]),y1]);}map();});
function render(){$('ttl').textContent=D.name+' - radar map';$('meta').textContent=`fw ${D.fw||'?'} · up ${Math.round((D.up||0)/60)} min · rssi ${D.rssi==null?'?':Math.round(D.rssi)} dBm`;
 $('cnt').textContent=D.count==null?'—':D.count;for(const[p,v]of[['pOcc',D.occ],['pMv',D.mv],['pMv2',D.mv],['pRaw',D.mvraw],['pRawO',D.occraw],['pSt',D.st],['pZ0',D.z[0]],['pZ1',D.z[1]],['pZ2',D.z[2]]])$(p).classList.toggle('on',!!v);
 {const k=D.trk||[],h=k.filter(q=>q[2]==1),l=k.filter(q=>q[2]==2);$('prN').textContent=k.length;$('prS').textContent=!k.length?'empty':h.length?mmss(Math.max(...h.map(q=>q[3]))):l.length?mmss(Math.max(...l.map(q=>q[3]))):'in view';$('prL').textContent=!k.length?'nobody tracked':h.length?'lost while still, holding':l.length?'walked out, clearing':'radar can see them';}
 for(const[p,v]of[['ph',D.ps],['pr',D.pm]])for(let i=0;i<4;i++)$(p+i).classList.toggle('on',$(p+i).textContent===v);$('phC').textContent=D.ps||'—';$('pmC').textContent=D.pm||'—';
 {const now=Date.now();let f=null;D.t.forEach(t=>{if(t&&t[1]!=null&&t[2]!=null&&(D.n.mvr==null||t[1]<=D.n.mvr*1000)){const a=Math.abs(t[2]);if(f==null||a>f)f=a;}});if(f!=null)spk.push([now,f]);spk=spk.filter(x=>now-x[0]<30000);
  $('spdNow').textContent=f==null?'—':Math.round(f);$('spdPk').textContent=spk.length?Math.round(Math.max(...spk.map(x=>x[1]))):'—';
  const th=D.n.mvs;for(const id of ['spdNow','spdPk']){const v=+$(id).textContent;$(id).style.color=(th!=null&&v>=th)?'#4aa3ff':'';}}
 let tb='';D.t.forEach((t,i)=>{const on=t&&t[1]!=null;if(on){trails[i].push([t[0],t[1]]);if(trails[i].length>40)trails[i].shift();}else trails[i]=[];
  tb+=`<tr><td><span class="dot" style="background:${TC[i]}"></span>${i+1}</td><td>${on?m2(t[0]):'—'}</td><td>${on?m2(t[1]):'—'}</td><td>${on?m2(Math.hypot(t[0],t[1])):'—'}</td><td>${on?Math.round(Math.atan2(t[0],t[1])*180/Math.PI)+'&deg;':'—'}</td><td style="${mvQual(t)?'color:#4aa3ff;font-weight:700':''}">${on&&t[2]!=null?Math.abs(t[2])+' cm/s':'—'}</td></tr>`;});$('tt').innerHTML=tb;
 for(let i=0;i<3;i++)for(let j=0;j<4;j++)fill('z'+i+j,D.zc[i]?D.zc[i][j]:null,v=>+(v/1000).toFixed(2));
 fill('timeout',D.n.timeout);fill('hold',D.n.hold);fill('mvs',D.n.mvs);fill('mvr',D.n.mvr);fill('mvh',D.n.mvh);fill('sth',D.n.sth);fill('igf',D.n.igf);fill('igt',D.n.igt);fill('multi',D.multi);if(D.zt>=0)fill('ztype',D.zt);if(!rub)map();}
async function poll(){try{const r=await fetch('/data.json',{cache:'no-store',signal:AbortSignal.timeout(3000)});D=await r.json();render();}catch(e){$('meta').textContent='no answer from the device';}}
window.addEventListener('resize',()=>{if(D)map();});poll();setInterval(poll,500);
</script></body></html>)ESPH";

static void rw_num(std::string &o, float v) {
  char b[24]; if (std::isnan(v)) snprintf(b, sizeof b, "null"); else snprintf(b, sizeof b, "%g", v); o += b;
}
static void rw_bool(std::string &o, const char *k, esphome::binary_sensor::BinarySensor *b) { o += "\""; o += k; o += (b && b->state) ? "\":1," : "\":0,"; }

static esp_err_t rw_html(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  const char *p = RADAR_WEB_HTML; size_t rem = sizeof(RADAR_WEB_HTML) - 1;
  while (rem > 0) { size_t n = rem < 4096 ? rem : 4096; if (httpd_resp_send_chunk(req, p, (ssize_t) n) != ESP_OK) { httpd_resp_send_chunk(req, nullptr, 0); return ESP_FAIL; } p += n; rem -= n; }
  return httpd_resp_send_chunk(req, nullptr, 0);
}
static esp_err_t rw_health(httpd_req_t *req) { httpd_resp_set_type(req, "text/plain"); return httpd_resp_sendstr(req, "OK"); }

static esp_err_t rw_data(httpd_req_t *req) {
  auto &c = radar_web_cfg(); std::string o; o.reserve(900); o += "{";
  o += "\"name\":\""; o += esphome::App.get_friendly_name(); o += "\",";
  rw_bool(o, "occ", c.occ); rw_bool(o, "mv", c.mv); rw_bool(o, "st", c.st); rw_bool(o, "mvraw", c.mvraw); rw_bool(o, "occraw", c.occraw);
  o += "\"z\":["; for (int i = 0; i < 3; i++) { o += (c.zn[i] && c.zn[i]->state) ? "1" : "0"; if (i < 2) o += ","; } o += "],";
  o += "\"t\":["; for (int i = 0; i < 3; i++) {
    o += "["; rw_num(o, c.tx[i] ? c.tx[i]->get_raw_state() : NAN); o += ","; rw_num(o, c.ty[i] ? c.ty[i]->get_raw_state() : NAN); o += ","; rw_num(o, c.ts[i] ? c.ts[i]->get_raw_state() : NAN); o += "]";
    if (i < 2) o += ","; } o += "],";
  o += "\"zc\":["; for (int i = 0; i < 3; i++) { o += "["; for (int j = 0; j < 4; j++) { rw_num(o, (c.zc[i][j] && c.zc[i][j]->has_state()) ? c.zc[i][j]->state : NAN); if (j < 3) o += ","; } o += "]"; if (i < 2) o += ","; } o += "],";
  o += "\"count\":"; rw_num(o, c.count ? c.count->get_raw_state() : NAN); o += ",";
  o += "\"n\":{\"timeout\":"; rw_num(o, (c.timeout && c.timeout->has_state()) ? c.timeout->state : NAN); o += ",\"hold\":"; rw_num(o, (c.hold && c.hold->has_state()) ? c.hold->state : NAN);
  o += ",\"mvs\":"; rw_num(o, (c.mvs && c.mvs->has_state()) ? c.mvs->state : NAN);
  o += ",\"mvr\":"; rw_num(o, (c.mvr && c.mvr->has_state()) ? c.mvr->state : NAN);
  o += ",\"mvh\":"; rw_num(o, (c.mvh && c.mvh->has_state()) ? c.mvh->state : NAN);
  o += ",\"sth\":"; rw_num(o, (c.sth && c.sth->has_state()) ? c.sth->state : NAN);
  o += ",\"igf\":"; rw_num(o, (c.igf && c.igf->has_state()) ? c.igf->state : NAN);
  o += ",\"igt\":"; rw_num(o, (c.igt && c.igt->has_state()) ? c.igt->state : NAN); o += "},";
  // presence tracks: x mm, y mm, state 0 in view / 1 held still / 2 walked out, seconds left
  { uint32_t now = millis(); o += "\"trk\":["; bool first = true;
    for (const auto &t : pr_t) { PrTrack k = t; if (!k.used) continue; uint32_t age = now - k.seen;
      int st = age < 1500 ? 0 : (k.limit > 0 && k.seen - k.moved < PR_EXIT_MS ? 2 : 1);
      if (!first) o += ","; first = false; o += "["; rw_num(o, roundf(k.x)); o += ","; rw_num(o, roundf(k.y)); o += ","; o += std::to_string(st); o += ",";
      rw_num(o, age < k.limit ? (float) ((k.limit - age) / 1000) : 0.0f); o += "]"; }
    o += "],"; }
  o += "\"rst\":"; o += std::to_string(pr_restored); o += ",";   // tracks put back at boot, -1 = no snapshot
  o += "\"ps\":\""; o += rw_still_preset((c.sth && c.sth->has_state()) ? c.sth->state : NAN); o += "\",";
  o += "\"pm\":\""; o += rw_move_preset((c.mvs && c.mvs->has_state()) ? c.mvs->state : NAN, (c.mvh && c.mvh->has_state()) ? c.mvh->state : NAN); o += "\",";
  o += "\"multi\":"; o += (c.multi && c.multi->state) ? "1," : "0,";
  o += "\"zt\":"; o += std::to_string(rw_zt); o += ",";
  o += "\"fw\":\""; o += rw_fw; o += "\",";
  o += "\"up\":"; rw_num(o, (c.uptime && c.uptime->has_state()) ? c.uptime->state : NAN); o += ",";
  o += "\"rssi\":"; rw_num(o, (c.rssi && c.rssi->has_state()) ? c.rssi->state : NAN); o += "}";
  httpd_resp_set_type(req, "application/json"); httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, o.c_str(), o.size());
}

static bool rw_query(httpd_req_t *req, char *k, size_t kl, char *v, size_t vl) {
  char q[96]; if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK) return false;
  return httpd_query_key_value(q, "k", k, kl) == ESP_OK && httpd_query_key_value(q, "v", v, vl) == ESP_OK;
}
// z1x1 .. z3y2, timeout, hold, mvs, mvr, mvh, sth
static esphome::number::Number *rw_find(const char *k) {
  auto &c = radar_web_cfg();
  if (strcmp(k, "timeout") == 0) return c.timeout;
  if (strcmp(k, "hold") == 0) return c.hold;
  if (strcmp(k, "mvs") == 0) return c.mvs;
  if (strcmp(k, "mvr") == 0) return c.mvr;
  if (strcmp(k, "mvh") == 0) return c.mvh;
  if (strcmp(k, "sth") == 0) return c.sth;
  if (strcmp(k, "igf") == 0) return c.igf;
  if (strcmp(k, "igt") == 0) return c.igt;
  if (k[0] == 'z' && k[1] >= '1' && k[1] <= '3' && (k[2] == 'x' || k[2] == 'y') && (k[3] == '1' || k[3] == '2') && k[4] == 0)
    return c.zc[k[1] - '1'][(k[3] - '1') * 2 + (k[2] == 'y' ? 1 : 0)];
  return nullptr;
}
static esp_err_t rw_set(httpd_req_t *req) {
  char k[16], v[16]; httpd_resp_set_type(req, "text/plain");
  if (!rw_query(req, k, sizeof k, v, sizeof v)) return httpd_resp_sendstr(req, "ERR query");
  if (!rw_find(k)) return httpd_resp_sendstr(req, "ERR key");
  float f = strtof(v, nullptr); if (std::isnan(f)) return httpd_resp_sendstr(req, "ERR value");
  return httpd_resp_sendstr(req, rw_push(k, f) ? "queued" : "ERR busy");
}
static esp_err_t rw_sw(httpd_req_t *req) {
  char k[16], v[16]; httpd_resp_set_type(req, "text/plain");
  if (!rw_query(req, k, sizeof k, v, sizeof v)) return httpd_resp_sendstr(req, "ERR query");
  if (strcmp(k, "multi") == 0) return httpd_resp_sendstr(req, rw_push("!multi", v[0] == '1' ? 1 : 0) ? "queued" : "ERR busy");
  if (strcmp(k, "zt") == 0 && v[0] >= '0' && v[0] <= '2') return httpd_resp_sendstr(req, rw_push("!zt", v[0] - '0') ? "queued" : "ERR busy");
  return httpd_resp_sendstr(req, "ERR key");
}

// main loop (100 ms yaml interval): perform what the httpd task queued
static void radar_web_pump() {
  auto &c = radar_web_cfg();
  while (rw_qh != rw_qt) {
    RwCmd cmd;
    taskENTER_CRITICAL(&rw_mux); cmd = rw_q[rw_qh]; rw_qh = (rw_qh + 1) & 15; taskEXIT_CRITICAL(&rw_mux);
    if (strcmp(cmd.k, "!multi") == 0) { if (c.multi) { if (cmd.v > 0.5f) c.multi->turn_on(); else c.multi->turn_off(); } continue; }
    if (strcmp(cmd.k, "!zt") == 0) { if (c.zt) c.zt->make_call().set_index((size_t) cmd.v).perform(); continue; }
    auto *n = rw_find(cmd.k); if (!n) continue;
    ESP_LOGI("radar_web", "set %s = %g", cmd.k, cmd.v);
    n->make_call().set_value(cmd.v).perform();
  }
}

// call from a yaml lambda after filling radar_web_cfg(); safe to call every few seconds (returns once running)
static void radar_web_register() {
  static bool registered = false, hooked = false; if (registered) return;
  if (millis() < 70000) return;   // v1.1: only after the app is marked valid
  auto &c = radar_web_cfg();
  if (!hooked) {
    hooked = true;
    if (c.fw) { c.fw->add_on_state_callback([](const std::string &x) { strlcpy(rw_fw, x.c_str(), sizeof rw_fw); }); if (c.fw->has_state()) strlcpy(rw_fw, c.fw->state.c_str(), sizeof rw_fw); }
    if (c.zt) { c.zt->add_on_state_callback([](size_t i) { rw_zt = (int) i; }); auto a = c.zt->active_index(); if (a.has_value()) rw_zt = (int) *a; }
  }
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  // ctrl_port must be a real port: 0 binds an ephemeral one but httpd still sends its work queue to port 0, so the LRU
  // close never runs. With all 5 slots taken it then refuses every client and spins on the listen socket.
  cfg.server_port = 8080; cfg.ctrl_port = 32769; cfg.stack_size = 8192; cfg.max_uri_handlers = 6; cfg.max_open_sockets = 5; cfg.lru_purge_enable = true;
  cfg.keep_alive_enable = true; cfg.keep_alive_idle = 10; cfg.keep_alive_interval = 5; cfg.keep_alive_count = 3;   // a vanished browser frees its slot
  cfg.task_priority = 1;   // v1.1: same as ESPHome's loop task, so a spin here cannot starve it
  httpd_handle_t srv = nullptr; esp_err_t err = httpd_start(&srv, &cfg);
  if (err != ESP_OK) { rw_page = 2; ESP_LOGE("radar_web", "httpd_start failed: %s - will retry", esp_err_to_name(err)); return; }
  registered = true;
  static const httpd_uri_t uris[] = {
    {"/", HTTP_GET, rw_html, nullptr}, {"/data.json", HTTP_GET, rw_data, nullptr}, {"/set", HTTP_GET, rw_set, nullptr},
    {"/sw", HTTP_GET, rw_sw, nullptr}, {"/healthz", HTTP_GET, rw_health, nullptr},
  };
  int ok = 0; for (const auto &u : uris) if (httpd_register_uri_handler(srv, &u) == ESP_OK) ok++;
  rw_page = ok == 5 ? 1 : 2;
  ESP_LOGI("radar_web", "radar map at http://<device-ip>:8080/");
}
