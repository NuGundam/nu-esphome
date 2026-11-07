#pragma once
// ─────────────────────────────────────────────────
//  NU-Presence MR60BHA2 page  v1.0  (mr60_web.h)  2026-11-06
//
//  An IDF httpd on port 8080, one embedded page, small JSON endpoints. Same shape as radar_web_2450.h.
//
//    /            live distance strip, vitals, trends, settings with presets
//    /data.json   live state + settings, polled twice a second
//    /set?k=..&v= write a setting: cd Clear delay s, sh Still hold s, mr Max range m, lb LED blue hold s,
//                 dt Dark threshold lx, led Presence LED 0/1
//    /healthz     "OK"
//
//  Threading: this device drives real automations, so the httpd task touches nothing of ESPHome's. The main loop
//  writes a finished JSON snapshot here four times a second (mw_publish) and the httpd task only copies it out.
//  Writes go the other way through a queue the main loop empties (mw_pop). Calling entity actions from the httpd task
//  crashed 24GHz-1.
//
//  The ESP32-C6 has one core, so the httpd runs at the loop's priority (1): a spin at IDF's default 5 would starve the
//  loop and trip the task watchdog. It starts 70 s after boot, past safe_mode's 60 s "boot is good" mark, so a crash
//  in it can reboot but can never roll the firmware back (24GHz-2 v1.1).
// ─────────────────────────────────────────────────
#include "esphome.h"
#include "esp_http_server.h"
#include <string>
#include <cstring>
#include <cstdlib>
#include <cmath>

struct MwState {
  bool occ, mv, raw, brd, hung, stat, off, dark, led, ledmv, wm;
  float d, t, br, hr, lux, seen, since, fps, pps, rs, up, rssi, cd, sh, mr, lb, dt;
  const char *name, *st, *vit, *ps;
};

// which Still hold preset the number matches; the page matches this against its button text
static const char *mw_still_preset(float v) {
  if (std::isnan(v)) return "Custom";
  switch ((int) lroundf(v)) {
    case 30: return "Hallway 30 s";
    case 60: return "Room 60 s";
    case 300: return "Couch or TV 300 s";
    case 900: return "Bed 900 s";
    default: return "Custom";
  }
}

// 0 not started yet (70 s after boot), 1 serving, 2 httpd failed
static volatile int mw_page = 0;
static const char *mw_web_state() { return mw_page == 1 ? "Ready" : mw_page == 2 ? "Failed" : "Starting"; }

// snapshot: main loop -> httpd task
static char mw_snap[1024] = "{}";
static size_t mw_snap_len = 2;
static portMUX_TYPE mw_mux = portMUX_INITIALIZER_UNLOCKED;

static void mw_num(std::string &o, const char *k, float v) {
  char b[40];
  if (std::isnan(v)) snprintf(b, sizeof b, "\"%s\":null,", k); else snprintf(b, sizeof b, "\"%s\":%.6g,", k, v);
  o += b;
}
static void mw_flag(std::string &o, const char *k, bool v) { o += "\""; o += k; o += v ? "\":1," : "\":0,"; }
static void mw_str(std::string &o, const char *k, const char *v) { o += "\""; o += k; o += "\":\""; o += v; o += "\","; }

static void mw_publish(const MwState &s) {
  std::string o; o.reserve(640); o += "{";
  mw_str(o, "name", s.name); mw_str(o, "ver", ESPHOME_VERSION); mw_str(o, "st", s.st); mw_str(o, "vit", s.vit); mw_str(o, "ps", s.ps);
  mw_flag(o, "occ", s.occ); mw_flag(o, "mv", s.mv); mw_flag(o, "raw", s.raw); mw_flag(o, "brd", s.brd); mw_flag(o, "hung", s.hung);
  mw_flag(o, "stat", s.stat); mw_flag(o, "off", s.off); mw_flag(o, "dark", s.dark); mw_flag(o, "led", s.led); mw_flag(o, "ledmv", s.ledmv); mw_flag(o, "wm", s.wm);
  mw_num(o, "d", s.d); mw_num(o, "t", s.t); mw_num(o, "br", s.br); mw_num(o, "hr", s.hr); mw_num(o, "lux", s.lux); mw_num(o, "seen", s.seen); mw_num(o, "since", s.since);
  mw_num(o, "fps", s.fps); mw_num(o, "pps", s.pps); mw_num(o, "rs", s.rs); mw_num(o, "up", s.up); mw_num(o, "rssi", s.rssi);
  mw_num(o, "cd", s.cd); mw_num(o, "sh", s.sh); mw_num(o, "mr", s.mr); mw_num(o, "lb", s.lb); mw_num(o, "dt", s.dt);
  o.back() = '}';
  if (o.size() >= sizeof mw_snap) return;
  taskENTER_CRITICAL(&mw_mux);
  memcpy(mw_snap, o.data(), o.size()); mw_snap_len = o.size();
  taskEXIT_CRITICAL(&mw_mux);
}

// write queue: httpd task -> main loop
struct MwCmd { char k[4]; float v; };
static MwCmd mw_q[16];
static volatile uint8_t mw_qh = 0, mw_qt = 0;
static bool mw_push(const char *k, float v) {
  bool ok = false;
  taskENTER_CRITICAL(&mw_mux);
  uint8_t nt = (mw_qt + 1) & 15;
  if (nt != mw_qh) { strlcpy(mw_q[mw_qt].k, k, sizeof mw_q[mw_qt].k); mw_q[mw_qt].v = v; mw_qt = nt; ok = true; }
  taskEXIT_CRITICAL(&mw_mux);
  return ok;
}
static bool mw_pop(MwCmd &c) {
  bool ok = false;
  taskENTER_CRITICAL(&mw_mux);
  if (mw_qh != mw_qt) { c = mw_q[mw_qh]; mw_qh = (mw_qh + 1) & 15; ok = true; }
  taskEXIT_CRITICAL(&mw_mux);
  return ok;
}

static const char MW_HTML[] PROGMEM = R"ESPH(<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Radar page</title>
<style>
:root{--bg:#10151c;--card:#182029;--line:#243040;--tx:#e6edf3;--mut:#8b9bb0;--blue:#4aa3ff;--green:#34d399;--amber:#f0c060;--bad:#f87171}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif;padding:12px 16px 32px}
html,body{overflow-x:hidden}.wrap{max-width:1100px;margin:0 auto;min-width:0}
h1{font-size:18px;margin:0 0 4px}h2{font-size:13px;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;margin:18px 0 8px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
.row{display:flex;flex-wrap:wrap;gap:10px 18px;align-items:center}
.pill{display:inline-block;padding:2px 9px;border-radius:999px;background:#2a3645;color:var(--mut);font-weight:600;font-size:12px}
.pill.on{background:#1e5a3f;color:#9ff0c9}.pill.bad{background:#5a1e1e;color:#ffc2c2}
.big{font-size:34px;font-weight:800;line-height:1}.mid{font-size:22px;font-weight:800;line-height:34px}.sub{color:var(--mut);font-size:12px}
label{color:var(--mut);font-size:12px;display:flex;flex-direction:column;gap:3px}label.ck{flex-direction:row;align-items:center;gap:6px;font-size:13px}
input[type=number]{background:#0d1218;color:var(--tx);border:1px solid var(--line);border-radius:6px;padding:5px 7px;font-size:14px;width:84px}
button{background:#263445;color:var(--tx);border:1px solid #35465b;border-radius:7px;padding:6px 11px;font-size:13px;cursor:pointer}button:hover{background:#31435a}button.on{background:#1e5a3f;border-color:#2f7d5a}
svg{display:block;width:100%}
.help{color:var(--mut);font-size:12px;margin-top:8px;line-height:1.5}
.tr{display:grid;grid-template-columns:96px 1fr;gap:8px;align-items:center;margin-top:6px}.tr b{font-size:18px;display:block}
@media (max-width:640px){body{padding:8px 8px 24px;font-size:13px}h1{font-size:16px}.card{padding:10px}.big{font-size:28px}.row{gap:8px 10px}
 .row>label:not(.ck){flex:1 1 calc(33% - 10px);min-width:90px}.row>label input{width:100%;min-width:0}button{min-height:38px}.tr{grid-template-columns:76px 1fr}}
</style></head><body><div class="wrap">
<h1 id="ttl">Radar page</h1><div class="sub" id="meta">connecting&hellip;</div>

<h2>Live</h2><div class="card"><div class="row">
<div><div class="big" id="dist">&mdash;</div><div class="sub">distance, m</div></div>
<div><div class="big" id="cnt">&mdash;</div><div class="sub">targets</div></div>
<div><span class="pill" id="pOcc">Occupied</span> <span class="pill" id="pMv">Moving</span> <span class="pill" id="pSt">Still</span> <span class="pill" id="pBr">Breathing</span> <span class="pill" id="pRaw" title="the radar reports a target right now, held 2 s because the raw output blinks">radar raw target</span></div>
</div>
<div class="sub" id="stat" style="margin-top:8px">&nbsp;</div>
<svg id="strip" height="78" style="margin-top:8px"></svg>
<div class="help">This radar gives one distance and no direction, so this is a straight line out from the sensor, not a map. It cannot see closer than 0.5 m. Green is the zone where it can read breathing and heart rate. Anything past the blue <b>Max range</b> line is ignored.</div>
</div>

<h2>Vitals</h2><div class="card">
<div class="row">
<div><div class="big" id="brv">&mdash;</div><div class="sub">breaths a minute</div></div>
<div><div class="big" id="hrv">&mdash;</div><div class="sub">heart beats a minute</div></div>
<div><div class="mid" id="vit" style="font-size:16px">&mdash;</div><div class="sub">what the radar can read</div></div>
</div>
<div class="help">Vitals need someone sitting still, facing the sensor, within about 1.5 m. They take around a minute to settle, and a dash means there is no reading right now. These are for interest, not medical use.</div>
</div>

<h2>Trends</h2><div class="card">
<div class="tr"><div><b id="tD">&mdash;</b><span class="sub">distance, m</span></div><svg id="gD" height="64"></svg></div>
<div class="tr"><div><b id="tB">&mdash;</b><span class="sub">breaths</span></div><svg id="gB" height="64"></svg></div>
<div class="tr"><div><b id="tH">&mdash;</b><span class="sub">heart</span></div><svg id="gH" height="64"></svg></div>
<div class="help">The last 5 minutes since this page opened, kept in your browser only.</div>
</div>

<h2>Occupied</h2><div class="card">
<div class="row">
<div><div class="mid" id="prS">&mdash;</div><div class="sub" id="prL">status</div></div>
<div><div class="mid" id="phC">&mdash;</div><div class="sub">current preset</div></div>
<label>Still hold s<input type="number" id="sh" min="10" max="900" step="10"></label>
<label>Clear delay s<input type="number" id="cd" min="5" max="900" step="5"></label>
</div>
<div class="row" style="margin-top:10px"><span class="sub">Presets:</span>
<button id="ph0">Hallway 30 s</button><button id="ph1">Room 60 s</button><button id="ph2">Couch or TV 300 s</button><button id="ph3">Bed 900 s</button>
</div>
<div class="help">Occupied turns on the moment the radar sees someone. When it loses them, how long it waits depends on what they were doing.<br>
<b>Still hold</b>: they were still, so they probably just faded out. Set it longer than the longest gap you see while sitting: 30 s for a hallway, 60 s (default) for a room, 300 s for a couch or TV, 900 s for a bed. Longer never delays someone who walks out normally.<br>
<b>Clear delay</b>: they moved in their last 5 seconds, so they probably left. 5 s (default) clears quickly, 30 s if lights go off too soon when you step out for a moment.</div>
</div>

<h2>Range</h2><div class="card">
<div class="row"><label>Max range m<input type="number" id="mr" min="0.5" max="6" step="0.25"></label></div>
<div class="help"><b>Max range</b>: set it just past the far wall, so someone moving in the next room doesn't count. Watch the distance above while you stand at the far wall, then add 0.25 m. 6 m (default) means no limit.</div>
</div>

<h2>Light and LED</h2><div class="card">
<div class="row">
<div><div class="big" id="lux">&mdash;</div><div class="sub">lux</div></div>
<div><span class="pill" id="pDk">Dark</span> <span class="pill" id="pLed">LED off</span></div>
<label>Dark threshold lx<input type="number" id="dt" min="1" max="200" step="1"></label>
<label>LED blue hold s<input type="number" id="lb" min="3" max="120" step="1"></label>
<label class="ck"><input type="checkbox" id="led">Presence LED</label>
</div>
<div class="help"><b>Dark threshold</b>: Dark turns on below this and clears at 50 % above it. 5&ndash;10 lx for a room that counts as dark with the TV on, 20 lx (default) for dusk, 50 lx if you want lights on earlier.<br>
<b>LED blue hold</b>: how long the LED stays blue after movement before going green. 3 s for a snappy LED, 10 s (default) for a calm one. It only changes the LED, nothing else.</div>
</div>

<h2>Radar health</h2><div class="card">
<div class="row">
<div><div class="big" id="fps">&mdash;</div><div class="sub">frames a second</div></div>
<div><div class="big" id="pps">&mdash;</div><div class="sub">phase frames a second</div></div>
<div><div class="big" id="rs">&mdash;</div><div class="sub">resets since boot</div></div>
<div><span class="pill" id="pOff">Offline</span> <span class="pill" id="pHung">Hung</span> <span class="pill" id="pStat">Target static</span></div>
</div>
<div class="help">Both rates normally sit around 10. Phase frames pause while the radar reads breathing up close, which is normal. Target static only means the numbers stopped changing for 3 minutes, usually someone sitting still beyond 1.5 m. The radar needs movement to find someone again after a reset, so reset it only if it looks wrong.</div>
</div>
</div>

<script>
const $=id=>document.getElementById(id),MAXM=6,WIN=300;let D=null,busy=0,hist=[],lastS=0;
const setv=async(k,v)=>{busy++;try{await fetch('/set?k='+k+'&v='+encodeURIComponent(v),{signal:AbortSignal.timeout(4000)});}catch(e){}busy--;};
function fill(id,v){const el=$(id);if(document.activeElement===el||busy)return;if(el.type==='checkbox')el.checked=!!v;else el.value=v==null?'':v;}
for(const k of ['sh','cd','mr','dt','lb'])$(k).addEventListener('change',e=>setv(k,e.target.value));
$('led').addEventListener('change',e=>setv('led',e.target.checked?1:0));
[30,60,300,900].forEach((v,i)=>$('ph'+i).addEventListener('click',()=>setv('sh',v)));
const mmss=s=>Math.floor(s/60)+':'+String(Math.floor(s%60)).padStart(2,'0');
const dm=()=>D.d!=null&&D.d>0?D.d/100:null,pos=v=>v!=null&&v>0?v:null;
function strip(){const el=$('strip'),W=Math.max(280,Math.round(el.clientWidth||900)),H=78,L=10,R=W-14,top=20,bh=26,X=m=>L+(R-L)*m/MAXM,mr=D.mr==null?MAXM:D.mr,d=dm();
 let s=`<rect x="${X(0)}" y="${top}" width="${X(MAXM)-X(0)}" height="${bh}" rx="4" fill="#ffffff0c"/><rect x="${X(0)}" y="${top}" width="${X(.5)-X(0)}" height="${bh}" fill="#00000055"/><rect x="${X(.5)}" y="${top}" width="${X(1.5)-X(.5)}" height="${bh}" fill="#34d39930"/>`;
 if(mr<MAXM)s+=`<rect x="${X(mr)}" y="${top}" width="${X(MAXM)-X(mr)}" height="${bh}" fill="#00000066"/>`;
 for(let m=0;m<=MAXM;m++)s+=`<line x1="${X(m)}" y1="${top+bh}" x2="${X(m)}" y2="${top+bh+5}" stroke="#6d7d92"/><text x="${X(m)}" y="${top+bh+17}" font-size="10" fill="#6d7d92" text-anchor="${m?m===MAXM?'end':'middle':'start'}">${m?m+' m':'sensor'}</text>`;
 s+=`<line x1="${X(mr)}" y1="${top-6}" x2="${X(mr)}" y2="${top+bh+3}" stroke="#4aa3ff" stroke-width="2" stroke-dasharray="4 3"/><text x="${Math.min(X(mr),R-58)}" y="${top-9}" font-size="10" fill="#4aa3ff" text-anchor="${X(mr)>R-58?'start':'middle'}">max ${mr} m</text>`;
 if(d!=null){const c=d>mr?'#6d7d92':D.mv?'#4aa3ff':'#34d399',x=X(Math.min(d,MAXM));s+=`<line x1="${x}" y1="${top-2}" x2="${x}" y2="${top+bh+2}" stroke="${c}" stroke-width="4"/><circle cx="${x}" cy="${top+bh/2}" r="7" fill="${c}"/>`;}
 if(s===strip.last&&W===strip.W)return;strip.last=s;strip.W=W;el.setAttribute('viewBox',`0 0 ${W} ${H}`);el.innerHTML=s;}
function graph(id,key,c,lo,hi){const el=$(id),W=Math.max(200,Math.round(el.clientWidth||800)),H=64,now=Date.now(),X=t=>W-(now-t)/1000/WIN*W,pts=hist.filter(p=>p[key]!=null);
 let a=lo,b=hi;for(const p of pts){a=Math.min(a,p[key]);b=Math.max(b,p[key]);}const Y=v=>H-5-(v-a)/((b-a)||1)*(H-10);let s=`<rect width="${W}" height="${H}" rx="4" fill="#ffffff07"/><text x="5" y="12" font-size="10" fill="#6d7d92">${Math.round(b*10)/10}</text><text x="5" y="${H-4}" font-size="10" fill="#6d7d92">${Math.round(a*10)/10}</text>`,seg=[],prev=0;
 const flush=()=>{if(seg.length>1)s+=`<polyline points="${seg.join(' ')}" fill="none" stroke="${c}" stroke-width="2"/>`;else if(seg.length)s+=`<circle cx="${seg[0].split(',')[0]}" cy="${seg[0].split(',')[1]}" r="2" fill="${c}"/>`;seg=[];};
 for(const p of hist){if(p[key]==null||p.t-prev>5000)flush();if(p[key]!=null)seg.push(X(p.t).toFixed(1)+','+Y(p[key]).toFixed(1));prev=p.t;}flush();el.setAttribute('viewBox',`0 0 ${W} ${H}`);el.innerHTML=s;}
function render(){const d=dm(),br=pos(D.br),hr=pos(D.hr),now=Date.now();$('ttl').textContent=D.name+' - radar';
 $('meta').textContent=`ESPHome ${D.ver} · up ${Math.round((D.up||0)/60)} min · rssi ${D.rssi==null?'?':Math.round(D.rssi)} dBm`;
 $('dist').textContent=d==null?'—':d.toFixed(2);$('cnt').textContent=D.t==null?'—':Math.round(D.t);$('stat').textContent=D.st||'';
 for(const[p,v]of[['pOcc',D.occ],['pMv',D.occ&&D.mv],['pSt',D.occ&&!D.mv],['pBr',D.brd],['pRaw',D.raw],['pDk',D.dark]])$(p).classList.toggle('on',!!v);
 for(const[p,v]of[['pOff',D.off],['pHung',D.hung]])$(p).classList.toggle('bad',!!v);$('pStat').classList.toggle('on',!!D.stat);
 $('brv').textContent=br==null?'—':Math.round(br);$('hrv').textContent=hr==null?'—':Math.round(hr);$('vit').textContent=D.vit||'—';
 {const hold=D.wm?D.cd:D.sh;if(!D.occ){$('prS').textContent='clear';$('prL').textContent=D.since==null?'nobody seen yet':'nobody for '+mmss(D.since);}else if(D.seen==null||D.seen<2){$('prS').textContent='in view';$('prL').textContent='radar can see them';}else{$('prS').textContent=mmss(Math.max(0,hold-D.seen));$('prL').textContent=D.wm?'moved then lost, clearing':'lost while still, holding';}}
 for(let i=0;i<4;i++)$('ph'+i).classList.toggle('on',$('ph'+i).textContent===D.ps);$('phC').textContent=D.ps||'—';
 $('lux').textContent=D.lux==null?'—':Math.round(D.lux);{const p=$('pLed'),c=!D.led||!D.occ?'off':D.ledmv?'blue':'green';p.textContent='LED '+c;p.style.background=c==='blue'?'#1d3f66':c==='green'?'':'';p.style.color=c==='blue'?'#a8d2ff':'';p.classList.toggle('on',c==='green');}
 $('fps').textContent=D.fps==null?'—':Math.round(D.fps);$('pps').textContent=D.pps==null?'—':Math.round(D.pps);$('rs').textContent=D.rs==null?'—':D.rs;
 fill('sh',D.sh);fill('cd',D.cd);fill('mr',D.mr);fill('dt',D.dt);fill('lb',D.lb);fill('led',D.led);strip();
 if(now-lastS>=1000){lastS=now;hist.push({t:now,d,br,hr});hist=hist.filter(p=>now-p.t<WIN*1000);$('tD').textContent=d==null?'—':d.toFixed(2);$('tB').textContent=br==null?'—':Math.round(br);$('tH').textContent=hr==null?'—':Math.round(hr);graph('gD','d','#4aa3ff',0,3);graph('gB','br','#34d399',10,20);graph('gH','hr','#f0a0e0',60,80);}}
async function poll(){try{const r=await fetch('/data.json',{cache:'no-store',signal:AbortSignal.timeout(3000)});D=await r.json();render();}catch(e){$('meta').textContent='no answer from the device';}}
window.addEventListener('resize',()=>{if(D)strip();});poll();setInterval(poll,500);
</script></body></html>)ESPH";

static esp_err_t mw_html(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  const char *p = MW_HTML; size_t rem = sizeof(MW_HTML) - 1;
  while (rem > 0) { size_t n = rem < 4096 ? rem : 4096; if (httpd_resp_send_chunk(req, p, (ssize_t) n) != ESP_OK) { httpd_resp_send_chunk(req, nullptr, 0); return ESP_FAIL; } p += n; rem -= n; }
  return httpd_resp_send_chunk(req, nullptr, 0);
}
static esp_err_t mw_health(httpd_req_t *req) { httpd_resp_set_type(req, "text/plain"); return httpd_resp_sendstr(req, "OK"); }
static esp_err_t mw_data(httpd_req_t *req) {
  char b[sizeof mw_snap]; size_t n;
  taskENTER_CRITICAL(&mw_mux);
  n = mw_snap_len; memcpy(b, mw_snap, n);
  taskEXIT_CRITICAL(&mw_mux);
  httpd_resp_set_type(req, "application/json"); httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, b, n);
}
static esp_err_t mw_set(httpd_req_t *req) {
  char q[64], k[8], v[16]; httpd_resp_set_type(req, "text/plain");
  if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK || httpd_query_key_value(q, "k", k, sizeof k) != ESP_OK || httpd_query_key_value(q, "v", v, sizeof v) != ESP_OK)
    return httpd_resp_sendstr(req, "ERR query");
  if (strcmp(k, "cd") && strcmp(k, "sh") && strcmp(k, "mr") && strcmp(k, "lb") && strcmp(k, "dt") && strcmp(k, "led")) return httpd_resp_sendstr(req, "ERR key");
  float f = strtof(v, nullptr); if (std::isnan(f) || std::isinf(f)) return httpd_resp_sendstr(req, "ERR value");
  return httpd_resp_sendstr(req, mw_push(k, f) ? "queued" : "ERR busy");
}

// from a yaml interval; returns at once when it is already running
static void mw_start() {
  static bool started = false; if (started) return;
  if (millis() < 70000) return;
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  // ctrl_port must be a real port: 0 binds an ephemeral one but httpd still sends its work queue to port 0, so the LRU
  // close never runs. With all 5 slots taken it then refuses every client and spins on the listen socket.
  cfg.server_port = 8080; cfg.ctrl_port = 32769; cfg.stack_size = 8192; cfg.max_uri_handlers = 5; cfg.max_open_sockets = 5; cfg.lru_purge_enable = true;
  cfg.keep_alive_enable = true; cfg.keep_alive_idle = 10; cfg.keep_alive_interval = 5; cfg.keep_alive_count = 3;   // a vanished browser frees its slot
  cfg.task_priority = 1;
  httpd_handle_t srv = nullptr; esp_err_t err = httpd_start(&srv, &cfg);
  if (err != ESP_OK) { mw_page = 2; ESP_LOGE("mr60_web", "httpd_start failed: %s - will retry", esp_err_to_name(err)); return; }
  started = true;
  static const httpd_uri_t uris[] = {
    {"/", HTTP_GET, mw_html, nullptr}, {"/data.json", HTTP_GET, mw_data, nullptr}, {"/set", HTTP_GET, mw_set, nullptr}, {"/healthz", HTTP_GET, mw_health, nullptr},
  };
  int ok = 0; for (const auto &u : uris) if (httpd_register_uri_handler(srv, &u) == ESP_OK) ok++;
  mw_page = ok == 4 ? 1 : 2;
  ESP_LOGI("mr60_web", "radar page at http://<device-ip>:8080/");
}
