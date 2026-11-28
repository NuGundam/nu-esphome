#pragma once
// ──────────────────────────────────────────────────────────────────
//  NU-Presence-24GHz  radar tuning page  v1.19  (radar_web.h)  2026-11-27
//  v1.19: "Rotate view" buttons (as radar_web_2450.h v1.16): turn the whole room plan a quarter turn left or right,
//        so the map faces the way you look at the room. Width / depth swap, the sensor spot and aim turn with it.
//  v1.18: "Air quality" card for devices with an air sensor (NU-Garage-1). The yaml builds one JSON object a second
//        on the main loop and hands it over with radar_web_aq(); data.json passes it on as "aq". Devices that never
//        call it show the page as before.
//  v1.17: room layout saved on the device (flash, /room?v=<json>, /room?reset=1) - every browser sees the same plan;
//        "Reset layout" goes back to radar_web_cfg().room_default.
//  v1.16: room view redesigned - the room to scale (ft / m) with the sensor on a wall (presets or drag), an aim knob,
//        the fan clipped to the room (faint past the walls), coverage %, per-device default radar_web_cfg().room_default.
//  v1.15: warnings when somebody is in view: note beside the Relearn button, stronger confirm text, amber learning
//        panel ("not learning" when the filter is blocked = JSON blk, "target in view" otherwise), "Room not learned".
//  v1.14: Relearn room button (radar_web_cfg().hf_relearn, /sw?k=relearn) + learning panel: countdown, progress bar,
//        "Room learned" note, and a sweep over the room view while the filter learns.
//  v1.13: person filter Ready / Learning N s / No data pill (the filter relearns the room for 60 s after a restart).
//  v1.12: the person label sits left of centre in its ring (it overlapped the distance label).
//  v1.11: person filter (human_filter.h) on the page when the device sets radar_web_cfg().hf_json: Person pill +
//        status, the person's ring in the room view, the learned thresholds as purple dashes in the gate chart.
//        Devices without the filter (hf_json unset) show the page as before.
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
  const char *room_default = nullptr;               // v1.16: room view default, JSON {"w":cm,"d":cm,"place":"tr","unit":"ft"}
  void (*hf_relearn)() = nullptr;                   // v1.14: person filter "Relearn room" (human_filter_relearn), optional
  void (*hf_json)(char *out, size_t n) = nullptr;   // v1.11: person filter state as JSON (human_filter_json), optional
};
static RadarWebCfg &radar_web_cfg() { static RadarWebCfg c; return c; }

// raw (unfiltered) copies, filled by callbacks so the page sees the 10 Hz stream, not the recorder-filtered values
static float rw_dist = NAN, rw_mdist = NAN, rw_sdist = NAN, rw_men = NAN, rw_sen = NAN, rw_gm[9], rw_gs[9];
static char rw_fw[32] = "", rw_res[16] = "";   // cached on the main loop by callbacks
static char rw_hf[2][240] = {"", ""};          // v1.11: person filter JSON, written on the main loop (two buffers,
static volatile int rw_hf_i = 0;               //        the httpd task reads the one that is not being written)

// write queue: httpd task -> main loop
static char rw_aq[2][700] = {"", ""};          // v1.18: air quality JSON from the yaml (two buffers, index swap)
static volatile int rw_aq_i = 0;
static void radar_web_aq(const char *js) { const int w = 1 - rw_aq_i; strlcpy(rw_aq[w], js, sizeof rw_aq[w]); rw_aq_i = w; }
static void rw_kn(std::string &o, const char *k, float v) {   // "k":number|null,
  char b[40]; if (std::isnan(v)) snprintf(b, sizeof b, "\"%s\":null,", k); else snprintf(b, sizeof b, "\"%s\":%g,", k, v); o += b; }
static void rw_ks(std::string &o, const char *k, const std::string &v) {   // "k":"escaped text",
  o += "\""; o += k; o += "\":\"";
  for (const char ch : v) { if (ch == '"' || ch == '\\') { o += '\\'; o += ch; } else if ((unsigned char) ch >= 0x20) o += ch; }
  o += "\","; }
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
.pill.on{background:#1e5a3f;color:#9ff0c9}.pill.hf.on{background:#5b2a86;color:#e9d0ff}.pill.hf.chk{background:#4a3a1a;color:#ffd79a}.pill.lrn{background:#1b3f73;color:#bcd8ff}.pill.bad{background:#5a1f1f;color:#ffc4c4}.pill.za.on{background:#145c43;color:#b2f5dc}.pill.zb.on{background:#1b3f73;color:#bcd8ff}
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
#hfLrn{margin-top:10px;padding:9px 11px;border-radius:8px;background:#12233c;border:1px solid #1b3f73;font-size:13px}#hfLrn.warn{background:#3a2a12;border-color:#8a5a14}#hfLrn.warn .pbar i{background:#d99a2b;animation:none}#hfLrn.warn .sub{color:#ffd79a}#hfOcc{color:#ffd79a;font-size:12px;font-weight:600}#hfLrn.done{background:#12301f;border-color:#1e5a3f}#hfLrn.done .rdr{display:none}#hfLrn.done .pbar i{background:#3fbf7f;animation:none}
.pbar{height:10px;border-radius:999px;background:#0d1218;border:1px solid var(--line);overflow:hidden;margin:7px 0 5px}
.pbar i{display:block;height:100%;width:0;border-radius:999px;background:linear-gradient(90deg,#2f6fd0,#8fc0ff,#2f6fd0);background-size:200% 100%;animation:shim 1.4s linear infinite;transition:width 1s linear}
.rdr{display:inline-block;width:13px;height:13px;border-radius:50%;border:2px solid #8fc0ff;border-top-color:transparent;animation:spin 1s linear infinite;vertical-align:-2px;margin-right:7px}
@keyframes shim{to{background-position:-200% 0}}@keyframes spin{to{transform:rotate(360deg)}}@keyframes swp{from{transform:scale(.02);opacity:.95}to{transform:scale(1);opacity:0}}
@media (prefers-reduced-motion:reduce){.pbar i,.rdr{animation:none}}
</style></head><body><div class="wrap">
<h1 id="ttl">Radar tuning</h1><div class="sub" id="meta">connecting&hellip;</div>
<button id="lock">&#128274; Locked &ndash; tap to edit</button>

<h2>Live</h2><div class="card"><div class="row">
<div><div class="big" id="dist">&mdash;</div><div class="sub">target distance</div></div>
<div><span class="pill" id="pOcc">Occupied</span> <span class="pill" id="pMv">Moving</span> <span class="pill" id="pSt">Still</span></div>
<div><span class="pill za" id="pZa">Zone A</span> <span class="pill zb" id="pZb">Zone B</span></div>
<div class="sub">moving <b id="men">&mdash;</b> @ <span id="mdist">&mdash;</span> &middot; still <b id="sen">&mdash;</b> @ <span id="sdist">&mdash;</span></div>
<div id="hfBox" style="display:none"><span class="pill" id="pHfR">Ready</span> <span class="pill hf" id="pHf">Person</span> <span class="sub">person filter: <b id="hfTxt">&mdash;</b></span> <button id="hfRl" style="padding:3px 9px;font-size:12px;margin-left:6px">&#8635; Relearn room</button> <span id="hfOcc" style="display:none"></span></div>
</div><div id="hfLrn" style="display:none"><span class="rdr"></span><b id="hfLrnT">Learning the room</b><span id="hfEta"></span><div class="pbar"><i id="hfBar"></i></div><div class="sub" id="hfLrnS">Keep the room empty until it finishes.</div></div><div class="help" id="hfHelp" style="display:none">Person filter: learns each ring's normal energy (printer, car fan, the unit's own fan) and only reports a person when energy above that travels two rings; then holds the person by still energy. It does not use the thresholds below. Purple dashes in the gate chart = what it has learned (100 = ring tuned out); purple ring in the room view = where the person is.</div></div>

<div id="aqWrap" style="display:none">
<h2>Air quality</h2><div class="card"><div class="row">
<div><div class="big" id="aqLvl" style="font-size:26px">&mdash;</div><div class="sub">air quality</div></div>
<div class="sub" id="aqWhy"></div></div>
<table style="margin-top:10px;width:100%;border-collapse:collapse;font-size:13px"><tbody id="aqT"></tbody></table>
</div></div>

<h2>Room view</h2><div class="card">
<svg id="fan" viewBox="0 0 900 520" preserveAspectRatio="xMidYMid meet"></svg>
<div class="row" style="margin-top:8px">
<label><span id="lblW">Room width</span><input type="number" id="room_w" min="1" max="70" step="0.1"></label>
<label><span id="lblD">Room depth</span><input type="number" id="room_d" min="1" max="70" step="0.1"></label>
<label class="sel">Units<select id="r_unit"><option value="ft">feet</option><option value="m">metres</option></select></label>
<label class="sel">Sensor on<select id="r_place"><option value="tl">top-left corner</option><option value="tm">top wall, middle</option><option value="tr">top-right corner</option><option value="rm">right wall, middle</option><option value="br">bottom-right corner</option><option value="bm">bottom wall, middle</option><option value="bl">bottom-left corner</option><option value="lm">left wall, middle</option><option value="custom">where I dragged it</option></select></label>
<label>Aim &deg;<input type="number" id="r_aim" min="0" max="359" step="1" style="width:70px"></label>
<button id="r_center" type="button">Aim at room centre</button> <button id="r_ccw" type="button" title="Turn the map a quarter turn to the left, so it faces the way you look at the room">&#x21BA; Rotate view</button> <button id="r_cw" type="button" title="Turn the map a quarter turn to the right, so it faces the way you look at the room">Rotate view &#x21BB;</button> <button id="r_reset" type="button">Reset layout</button>
<span id="roomTxt" class="sub"></span>
<button id="applyMax">Apply suggested max gates</button>
</div>
<div class="help">Top-down plan of the room, drawn to scale. <b>Drag the white sensor dot</b> along the walls to where the unit hangs and <b>drag the small knob</b> to aim it (or pick a spot and type the aim: 0&deg; = up, clockwise). The fan is the radar's ~120&deg; view: full strength inside the room, faint where it sees past the walls. The text under the plan gives the farthest point it sees in the room, the max gate to use and how much floor it covers. Saved on the device, so every browser and phone sees the same plan; <b>Reset layout</b> goes back to the layout built into this device.</div>
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
<div class="leg"><span><i style="background:var(--mv)"></i>move energy</span><span><i style="background:var(--st)"></i>still energy</span><span><i style="background:#fff;height:2px"></i>threshold (drag it)</span><span><i style="background:#ffffff66;height:2px"></i>peak since reset</span><span id="hfLeg" style="display:none"><i style="background:#c084fc;height:2px"></i>person filter's learned level</span><span>boxes: move / still threshold</span></div>
<div class="help">A gate only counts when its energy is above its threshold; 100 = that ring off. Recipe: leave the room (or sit still where you do NOT want detection), Reset peaks, wait a few minutes, then "Thresholds = peaks + margin" lifts every gate just above what it saw. Then lower the gates where you DO want to be seen if they ended up too high. Stand at least 1.5 m away while reading the bars - a target closer than ~0.5 m floods every gate.</div>
</div>

<script>
const $=id=>document.getElementById(id), N=9; let D=null, gl=75, peakM=Array(N).fill(0), peakS=Array(N).fill(0), drag=null, busy=0;
const numKeys=['a_near','a_far','b_near','b_far','edge','hold','timeout'], cmKeys=['a_near','a_far','b_near','b_far','edge']; /* v1.7: metres on the ESP and in the boxes; data.json sends these in cm */
let optGl=0; function buildMax(){ if(optGl===gl) return; optGl=gl; for (const k of ['maxm','maxs']) { const cur=$(k).value; $(k).innerHTML=''; for (let i=0;i<=8;i++) { const o=document.createElement('option'); o.value=i; o.textContent='gate '+i+' \u2192 '+((i+1)*gl/100).toFixed(2)+' m'; $(k).appendChild(o);} if(cur!=='') $(k).value=cur; } }
buildMax();
/* v1.16 room plan: cm, top-down, origin top-left, y down; sensor on a wall at (sx,sy), aim deg clockwise from up */
const FT=30.48, room={w:457,d:518,unit:'ft',place:'tr',sx:457,sy:0,aim:0,aimAuto:true}; let roomInit=false, geo=null, rdrag=null;
const uf=()=>room.unit==='ft'?FT:100, ul=()=>room.unit==='ft'?'ft':'m', fmtU=cm=>(cm/uf()).toFixed(1)+' '+ul();
function aimCentre(){ const dx=room.w/2-room.sx, dy=room.d/2-room.sy; room.aim=Math.round(((Math.atan2(dx,-dy)*180/Math.PI)+360)%360); }
function placeAt(k){ const X={l:0,m:room.w/2,r:room.w}, Y={t:0,m:room.d/2,b:room.d};
  const map={tl:['l','t'],tm:['m','t'],tr:['r','t'],rm:['r','m'],br:['r','b'],bm:['m','b'],bl:['l','b'],lm:['l','m']};
  if(!map[k]) return; room.place=k; room.sx=X[map[k][0]]; room.sy=Y[map[k][1]]; room.aimAuto=true; aimCentre(); }
function roomUi(){ $('room_w').value=(room.w/uf()).toFixed(1); $('room_d').value=(room.d/uf()).toFixed(1); $('r_unit').value=room.unit; $('r_place').value=room.place; $('r_aim').value=Math.round(room.aim);
  $('lblW').textContent='Room width '+ul(); $('lblD').textContent='Room depth '+ul(); }
let lastDev=null, lastSent=null, pushT=0;
function applyDev(r){ if(!r) return; if(r.sx!=null&&r.sy!=null){ Object.assign(room,r); } else { room.w=r.w||room.w; room.d=r.d||room.d; room.unit=r.unit||room.unit; placeAt(r.place||'bm'); } roomUi(); }
function saveRoom(){ roomUi(); if(D) fan(); clearTimeout(pushT);   /* v1.17: stored on the device, 1 s after the last change */
  pushT=setTimeout(()=>{ pushT=0; lastSent=JSON.stringify(room); fetch('/room?v='+encodeURIComponent(lastSent)).catch(()=>{}); },1000); }
function initRoom(dev){ if(roomInit) return; roomInit=true; if(dev){ lastDev=JSON.stringify(dev); applyDev(dev); } else { room.w=400; room.d=400; room.unit='m'; placeAt('bm'); roomUi(); } }
function syncRoom(){ if(!D||!D.room||rdrag||pushT) return; const js=JSON.stringify(D.room); if(js===lastDev) return; lastDev=js; if(js!==lastSent) applyDev(D.room); }
$('room_w').addEventListener('change',()=>{ const v=+$('room_w').value; if(v>0){ const k=room.sx/room.w; room.w=Math.round(v*uf()); room.sx=Math.round(k*room.w); if(room.aimAuto) aimCentre(); } saveRoom(); });
$('room_d').addEventListener('change',()=>{ const v=+$('room_d').value; if(v>0){ const k=room.sy/room.d; room.d=Math.round(v*uf()); room.sy=Math.round(k*room.d); if(room.aimAuto) aimCentre(); } saveRoom(); });
$('r_unit').addEventListener('change',()=>{ room.unit=$('r_unit').value; saveRoom(); });
$('r_place').addEventListener('change',()=>{ placeAt($('r_place').value); saveRoom(); });
$('r_aim').addEventListener('change',()=>{ room.aim=((+$('r_aim').value%360)+360)%360; room.aimAuto=false; saveRoom(); });
/* v1.19: turn the plan a quarter turn (cw: the top wall becomes the right wall); room + sensor turn together */
function rotRoom(cw){ const w=room.w, d=room.d, x=room.sx, y=room.sy, ring=['tl','tm','tr','rm','br','bm','bl','lm'], i=ring.indexOf(room.place);
  room.w=d; room.d=w; if(cw){ room.sx=Math.round(d-y); room.sy=Math.round(x); } else { room.sx=Math.round(y); room.sy=Math.round(w-x); }
  room.aim=Math.round(room.aim+(cw?90:270))%360; if(i>=0) room.place=ring[(i+(cw?2:6))%8]; saveRoom(); }
$('r_cw').onclick=()=>rotRoom(true); $('r_ccw').onclick=()=>rotRoom(false);
$('r_center').onclick=()=>{ room.aimAuto=true; aimCentre(); saveRoom(); };
$('r_reset').onclick=()=>{ if(!confirm('Reset the room plan to the layout built into this device?')) return; clearTimeout(pushT); pushT=0; lastSent=null; lastDev=null; fetch('/room?reset=1').catch(()=>{}); };
/* drag: the sensor dot slides along the walls, the knob turns the aim */
const fanEl=$('fan');
const svgPt=e=>{ const r=fanEl.getBoundingClientRect(); return [(e.clientX-r.left)*geo.W/r.width,(e.clientY-r.top)*geo.H/r.height]; };
fanEl.addEventListener('pointerdown',e=>{ const h=e.target&&e.target.dataset?e.target.dataset.h:null; if(!h||!geo) return; rdrag=h; fanEl.setPointerCapture(e.pointerId); e.preventDefault(); });
fanEl.addEventListener('pointermove',e=>{ if(!rdrag||!geo) return; const [x,y]=svgPt(e), rx=(x-geo.ox)/geo.sc, ry=(y-geo.oy)/geo.sc;
  if(rdrag==='s'){ const cx=Math.min(room.w,Math.max(0,rx)), cy=Math.min(room.d,Math.max(0,ry));   /* snap to the nearest wall */
    const d=[cy, room.w-cx, room.d-cy, cx], m=d.indexOf(Math.min(...d));
    room.sx=m==1?room.w:m==3?0:Math.round(cx); room.sy=m==0?0:m==2?room.d:Math.round(cy); room.place='custom'; if(room.aimAuto) aimCentre(); }
  else { room.aim=Math.round(((Math.atan2(rx-room.sx,-(ry-room.sy))*180/Math.PI)+360)%360); room.aimAuto=false; }
  roomUi(); fan(); });
const endDrag=()=>{ if(!rdrag) return; rdrag=null; saveRoom(); };
fanEl.addEventListener('pointerup',endDrag); fanEl.addEventListener('pointercancel',endDrag);
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
function fan(){ if(!roomInit) initRoom(D.room); else syncRoom();
  const el=$('fan'), W=Math.max(280,Math.round(el.clientWidth||900)), n=D.n, HALF=60*Math.PI/180, ax=N*gl, M=30;
  const maxH=Math.min(640,Math.round(W*0.9)), sc=Math.min((W-2*M)/room.w,(maxH-2*M)/room.d), RW=room.w*sc, RD=room.d*sc, H=Math.round(RD+2*M), ox=(W-RW)/2, oy=M;
  geo={W,H,ox,oy,sc}; el.setAttribute('viewBox',`0 0 ${W} ${H}`); el.style.height=H+'px';
  const cx=ox+room.sx*sc, cy=oy+room.sy*sc, th=room.aim*Math.PI/180, thin=gl*sc<26;
  const P=(r,a)=>[cx+r*Math.sin(th+a),cy-r*Math.cos(th+a)];           // a: 0 = where the sensor aims, +clockwise
  const arc=(r,a0,a1)=>{ const [x0,y0]=P(r,a0),[x1,y1]=P(r,a1); return `${x0.toFixed(1)} ${y0.toFixed(1)} A ${r.toFixed(1)} ${r.toFixed(1)} 0 ${a1-a0>Math.PI?1:0} 1 ${x1.toFixed(1)} ${y1.toFixed(1)}`; };
  const wedge=(r0,r1)=>{ const [xa,ya]=P(r0,HALF),[xb,yb]=P(r0,-HALF); return `M ${arc(r1,-HALF,HALF)} L ${xa.toFixed(1)} ${ya.toFixed(1)} A ${r0.toFixed(1)} ${r0.toFixed(1)} 0 0 0 ${xb.toFixed(1)} ${yb.toFixed(1)} Z`; };
  let s=`<defs><clipPath id="rclip"><rect x="${ox.toFixed(1)}" y="${oy.toFixed(1)}" width="${RW.toFixed(1)}" height="${RD.toFixed(1)}"/></clipPath></defs>`;
  s+=`<rect x="${ox.toFixed(1)}" y="${oy.toFixed(1)}" width="${RW.toFixed(1)}" height="${RD.toFixed(1)}" fill="#ffffff05"/>`;
  { let st=uf(); while(st*sc<14) st*=2; for(let x=st;x<room.w-1;x+=st) s+=`<line x1="${(ox+x*sc).toFixed(1)}" y1="${oy}" x2="${(ox+x*sc).toFixed(1)}" y2="${(oy+RD).toFixed(1)}" stroke="#ffffff0b"/>`;
    for(let y=st;y<room.d-1;y+=st) s+=`<line x1="${ox}" y1="${(oy+y*sc).toFixed(1)}" x2="${(ox+RW).toFixed(1)}" y2="${(oy+y*sc).toFixed(1)}" stroke="#ffffff0b"/>`; }
  const mg=Math.max(+n.maxm,+n.maxs);
  let f=`<path d="${wedge(0.01,ax*sc)}" fill="#8fb4ff10" stroke="#ffffff22"/>`;
  for(let i=0;i<N;i++){ const gs=D.gs[i],gm=D.gm[i],r0=i*gl*sc,r1=(i+1)*gl*sc;
    if(gs!=null&&gs>2) f+=`<path d="${wedge(Math.max(0.01,r0),r1)}" fill="var(--st)" opacity="${(gs/100*0.5).toFixed(2)}"/>`;
    if(gm!=null&&gm>2) f+=`<path d="M ${arc((r0+r1)/2,-HALF,HALF)}" fill="none" stroke="var(--mv)" stroke-width="${(1+gm/100*(r1-r0)*0.8).toFixed(1)}" opacity=".7"/>`; }
  if(mg<8) f+=`<path d="${wedge((mg+1)*gl*sc,ax*sc)}" fill="#000" opacity=".4"/>`;
  f+=`<path d="${wedge(Math.max(0.01,n.a_near*sc),Math.max(n.a_near+1,n.a_far)*sc)}" fill="var(--za)" opacity=".28"/>`;
  f+=`<path d="${wedge(Math.max(0.01,n.b_near*sc),Math.max(n.b_near+1,n.b_far)*sc)}" fill="var(--zb)" opacity=".28"/>`;
  for(let i=1;i<=N;i++) f+=`<path d="M ${arc(i*gl*sc,-HALF,HALF)}" fill="none" stroke="${i-1>mg?'#ffffff18':'#ffffff40'}"/>`;
  const drawArc=(d,col,w)=>{ const r=d*sc; if(r>2&&r<ax*sc) f+=`<path d="M ${arc(r,-HALF,HALF)}" fill="none" stroke="${col}" stroke-width="${w}" opacity=".9"/>`; };
  if(D.hf&&D.hf.st>0&&D.hf.hg>=0){ const g=D.hf.hg, c=D.hf.st==2?'#c084fc':'#f0c060'; f+=`<path d="${wedge(Math.max(0.01,g*gl*sc),(g+1)*gl*sc)}" fill="${c}" opacity=".45" stroke="${c}" stroke-width="2"/>`; }
  if(D.mv&&D.mdist>0) drawArc(D.mdist,'var(--mv)',3); if(D.st&&D.sdist>0) drawArc(D.sdist,'var(--st)',3);
  if(D.occ&&D.dist>0) drawArc(D.dist,'#fff',4);
  s+=`<g opacity=".2">${f}</g><g clip-path="url(#rclip)">${f}</g>`;
  if(D.hf&&D.hf.st==-1) for(const k of [0,1]) s+=`<path d="M ${arc(ax*sc,-HALF,HALF)}" fill="none" stroke="#8fc0ff" stroke-width="3" vector-effect="non-scaling-stroke" clip-path="url(#rclip)" style="transform-origin:${cx.toFixed(1)}px ${cy.toFixed(1)}px;animation:swp 2.4s linear infinite;animation-delay:-${(((Date.now()+k*1200)%2400)/1000).toFixed(2)}s"/>`;
  // walls + dimensions
  s+=`<rect x="${ox.toFixed(1)}" y="${oy.toFixed(1)}" width="${RW.toFixed(1)}" height="${RD.toFixed(1)}" fill="none" stroke="#f0c060" stroke-width="2.5"/>`;
  s+=`<text x="${(ox+RW/2).toFixed(1)}" y="${(oy-9).toFixed(1)}" font-size="12" fill="#f0c060" text-anchor="middle">${fmtU(room.w)}</text>`;
  s+=`<text x="${(ox-9).toFixed(1)}" y="${(oy+RD/2).toFixed(1)}" font-size="12" fill="#f0c060" text-anchor="middle" transform="rotate(-90 ${(ox-9).toFixed(1)} ${(oy+RD/2).toFixed(1)})">${fmtU(room.d)}</text>`;
  // ring labels along the left edge of the fan, only where they land inside the plan
  for(let i=1;i<=N;i++){ if(thin&&i%2) continue; const r=i*gl*sc-gl*sc/2;   /* first angle where the label lands inside the plan */
    for(const a of [-0.95,0.95,-0.7,0.7,-0.35,0.35]){ const [tx,ty]=P(r,a); if(tx>ox+10&&tx<ox+RW-10&&ty>oy+12&&ty<oy+RD-4){ s+=`<text x="${tx.toFixed(1)}" y="${(ty+4).toFixed(1)}" font-size="10.5" fill="#c3cfdd" text-anchor="middle">g${i-1}</text>`; break; } } }
  if(D.occ&&D.dist>0){ const [tx,ty]=P(D.dist*sc+14,0); s+=`<text x="${tx.toFixed(1)}" y="${ty.toFixed(1)}" font-size="13" font-weight="700" fill="#fff" text-anchor="middle">${(D.dist/100).toFixed(2)} m</text>`; }
  if(D.hf&&D.hf.st>0&&D.hf.hg>=0){ const [px,py]=P((D.hf.hg+0.5)*gl*sc,-0.6); s+=`<text x="${px.toFixed(1)}" y="${(py+4).toFixed(1)}" font-size="12" font-weight="700" fill="#fff" text-anchor="middle">${D.hf.st==2?'person':'checking'}</text>`; }
  if(D.hf&&D.hf.st==-1){ const [qx,qy]=P(Math.min(ax*sc*0.5,Math.hypot(RW,RD)*0.35),0); s+=`<text x="${qx.toFixed(1)}" y="${qy.toFixed(1)}" font-size="14" font-weight="700" fill="#bcd8ff" text-anchor="middle">learning the room – ${D.hf.lrn} s</text>`; }
  // what the fan sees of the room: far point in view, floor coverage within the max gate
  let far=0, cov=0, tot=0; const reach=(mg+1)*gl;
  for(let i=0;i<=24;i++) for(let j=0;j<=24;j++){ const x=room.w*i/24, y=room.d*j/24, dx=x-room.sx, dy=y-room.sy, d=Math.hypot(dx,dy);
    let a=Math.atan2(dx,-dy)-th; a=Math.atan2(Math.sin(a),Math.cos(a)); const inF=d<1||Math.abs(a)<=HALF;
    if(inF) far=Math.max(far,d); if(i>0&&i<24&&j>0&&j<24){ tot++; if(inF&&d<=reach) cov++; } }
  suggMax=Math.min(8,Math.max(0,Math.ceil(far/gl)-1));
  // sensor + aim knob (drag handles)
  const kr=Math.max(46,Math.min(110,gl*1.4*sc)), [kx,ky]=P(kr,0);
  s+=`<line x1="${cx.toFixed(1)}" y1="${cy.toFixed(1)}" x2="${kx.toFixed(1)}" y2="${ky.toFixed(1)}" stroke="#fff" stroke-width="2" stroke-dasharray="4 3" opacity=".8"/>`;
  s+=`<circle data-h="a" cx="${kx.toFixed(1)}" cy="${ky.toFixed(1)}" r="9" fill="#2a3a52" stroke="#fff" stroke-width="2" style="cursor:grab"/>`;
  s+=`<circle data-h="s" cx="${cx.toFixed(1)}" cy="${cy.toFixed(1)}" r="10" fill="#fff" stroke="#16202c" stroke-width="2" style="cursor:move"/>`;
  { const lx=Math.min(ox+RW-4,Math.max(ox+4,cx)), ly=cy<oy+RD/2?cy+24:cy-16; s+=`<text x="${lx.toFixed(1)}" y="${ly.toFixed(1)}" font-size="11" fill="#8b9bb0" text-anchor="${cx>ox+RW*0.66?'end':cx<ox+RW*0.33?'start':'middle'}">sensor &middot; ${Math.round(room.aim)}&deg;</text>`; }
  el.innerHTML=s;
  const pName=$('r_place').selectedOptions[0]?$('r_place').selectedOptions[0].textContent:'';
  $('roomTxt').innerHTML=`<span style="color:#8b9bb0">${D.room_src==='device'?'saved on the device':'device default layout'}</span> &middot; sensor ${pName} aiming ${Math.round(room.aim)}&deg; &middot; farthest point in view <b>${(far/100).toFixed(2)} m</b> (${fmtU(far)}) &rarr; suggested max gate <b>${suggMax}</b> (${((suggMax+1)*gl/100).toFixed(2)} m)${mg>suggMax?` &mdash; now ${mg}, scanning past the walls`:mg<suggMax?` &mdash; now ${mg}, the far part of the room is cut off`:''} &middot; covers <b>${tot?Math.round(cov/tot*100):0}%</b> of the floor (gates 0&ndash;${mg})`; }
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
    if(D.hf&&D.hf.tm){ const hm=D.hf.tm[i], hs=D.hf.ts[i]; s+=`<line x1="${xm-2}" y1="${Y(hm)}" x2="${xm+bw+2}" y2="${Y(hm)}" stroke="#c084fc" stroke-width="2" stroke-dasharray="4 3"/><line x1="${xs-2}" y1="${Y(hs)}" x2="${xs+bw+2}" y2="${Y(hs)}" stroke="#c084fc" stroke-width="2" stroke-dasharray="4 3"/>`; }
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
const AQC=['#00B050','#E6C700','#F08C00','#D93025','#8E44AD'],AQN=['Excellent','Good','Moderate','Sensitive','Unhealthy'];
function aq(){ const a=D&&D.aq; if(!a) return; $('aqWrap').style.display='';   /* v1.18 */
 const l=a.lvl, w=l==null||l<0, e=$('aqLvl'); e.textContent=w?'Warming up':AQN[Math.min(4,l)]; e.style.color=w?'#8b9bb0':AQC[Math.min(4,l)];
 $('aqWhy').textContent=w||!a.why?'':'set by '+a.why;
 const f=(v,d,u)=>v==null?'\u2014':v.toFixed(d)+(u||'');
 const rows=[['PM2.5',f(a.pm25,1,' \u00B5g/m\u00B3'),'PM AQI',f(a.pmaqi,0)],['PM1 / PM4 / PM10',f(a.pm1,1)+' / '+f(a.pm4,1)+' / '+f(a.pm10,1),'PM status',a.pmt||''],
  ['VOC index',f(a.voc,0),'VOC status',a.voct||''],['NOx index',f(a.nox,0),'NOx status',a.noxt||''],
  ['Temperature',f(a.tf,1,' \u00B0F'),'Humidity',f(a.rh,0,' %')],['Dew point',f(a.dew,1,' \u00B0F'),'Absolute humidity',f(a.ah,1,' g/m\u00B3')]];
 const tb=$('aqT'); tb.textContent=''; for(const r of rows){ const tr=document.createElement('tr'); r.forEach((c,i)=>{ const td=document.createElement('td'); td.textContent=c; td.style.padding='4px 10px 4px 0'; td.style.borderTop='1px solid rgba(255,255,255,.07)'; if(i%2==0) td.style.color='#8b9bb0'; else td.style.fontWeight='600'; tr.appendChild(td); }); tb.appendChild(tr); } }
function render(){ aq(); gl=(D.res||'').startsWith('0.2')?20:75; gtab(); buildMax();
  $('ttl').textContent=D.name+' - radar tuning'; document.title=D.name+' - radar tuning'; $('meta').textContent=`fw ${D.fw||'?'} · up ${Math.round((D.up||0)/60)} min · rssi ${D.rssi==null?'?':Math.round(D.rssi)} dBm`;
  $('dist').textContent=D.occ?fmt(D.dist):'no target'; $('men').textContent=D.men==null?'—':Math.round(D.men); $('sen').textContent=D.sen==null?'—':Math.round(D.sen);
  $('mdist').textContent=fmt(D.mdist); $('sdist').textContent=fmt(D.sdist);
  for(const [p,v] of [['pOcc',D.occ],['pMv',D.mv],['pSt',D.st],['pZa',D.za],['pZb',D.zb]]) $(p).classList.toggle('on',!!v);
  { const h=D.hf; for(const id of ['hfBox','hfHelp','hfLeg']) $(id).style.display=h?'':'none'; if(h){ $('pHf').classList.toggle('on',h.st==2); $('pHf').classList.toggle('chk',h.st==1); $('pHf').textContent=h.st==1?'Checking':'Person'; $('hfTxt').textContent=h.txt; const r=$('pHfR'); r.className='pill '+(h.st==-1?'lrn':h.st==-2?'bad':'on'); r.textContent=h.st==-1?`Learning ${h.lrn} s`:h.st==-2?'No data':'Ready'; const L=$('hfLrn'); $('hfRl').disabled=h.st==-1; { const o=$('hfOcc'), w=h.st!=-1&&!!D.occ; o.style.display=w?'':'none'; if(w) o.textContent='\u26A0 target in view at '+fmt(D.dist)+' \u2013 do not relearn now'; } if(h.st==-1){ hfWas=true; hfReq=0; clearTimeout(hfDoneT); hfDoneT=0; const tot=h.tot||60; L.style.display=''; L.classList.remove('done'); const wb=!!h.blk, wo=!wb&&!!D.occ; if(wb) hfBlk=true; L.classList.toggle('warn',wb||wo); $('hfLrnT').textContent=wb?'\u26A0 Someone is in view \u2013 not learning':'Learning the room'; $('hfEta').textContent=' \u2013 '+(wb?'ends':'ready')+' in '+h.lrn+' s'; $('hfLrnS').textContent=wb?'The radar sees a target at '+fmt(D.dist)+', so nothing is being learned. Leave the room, then press Relearn room again.':wo?'\u26A0 The radar sees a target at '+fmt(D.dist)+'. If that is a person they are being learned as part of the room: leave, then relearn.':'Keep the room empty until it finishes.'; $('hfBar').style.width=Math.max(2,Math.min(100,(tot-h.lrn+1)/tot*100)).toFixed(1)+'%'; } else if(hfWas&&h.st>=0){ hfWas=false; L.style.display=''; L.classList.add('done'); L.classList.toggle('warn',hfBlk); L.classList.toggle('done',!hfBlk); $('hfLrnT').textContent=hfBlk?'\u26A0 Room not learned':'\u2713 Room learned'; $('hfEta').textContent=hfBlk?' \u2013 someone was in view':' \u2013 the filter is ready'; $('hfLrnS').textContent=hfBlk?'Typical empty-room levels are in use until the room is empty. Relearn when nobody is in view.':'The learned levels are the purple dashes in the gate chart.'; const keep=hfBlk?20000:6000; hfBlk=false; $('hfBar').style.width='100%'; hfDoneT=setTimeout(()=>{ hfDoneT=0; L.style.display='none'; },keep); } else if(!hfDoneT&&!(hfReq&&Date.now()-hfReq<5000)) L.style.display='none'; } else $('hfLrn').style.display='none'; }
  $('res').textContent=D.res||'?'; fill('eng',D.eng); $('engWarn').textContent=D.eng?'':'engineering mode is off - no gate energies';
  for(const k of numKeys.concat(['maxm','maxs'])) fill(k,D.n[k]); for(let i=0;i<N;i++){ fill('gm'+i,D.n['gm'+i]); fill('gs'+i,D.n['gs'+i]); if(D.gm[i]!=null) peakM[i]=Math.max(peakM[i],D.gm[i]); if(D.gs[i]!=null) peakS[i]=Math.max(peakS[i],D.gs[i]); }
  fan(); axis(); gates(); }
async function poll(){ try{ const r=await fetch('/data.json',{cache:'no-store',signal:AbortSignal.timeout(3000)}); D=await r.json(); render(); }catch(e){ $('meta').textContent='no answer from the device'; } }
let hfWas=false, hfReq=0, hfDoneT=0, hfBlk=false;
$('hfRl').onclick=()=>{ if(!confirm((D&&D.occ?'WARNING: the radar sees a target at '+fmt(D.dist)+' right now. If that is a person, cancel and relearn after leaving the room.\n\n':'')+'Learn the room again? It takes 60 seconds and the room must stay empty the whole time.')) return; hfBlk=false; hfReq=Date.now(); const L=$('hfLrn'); clearTimeout(hfDoneT); hfDoneT=0; L.classList.remove('done','warn'); L.style.display=''; $('hfLrnT').textContent='Starting'; $('hfEta').textContent=' \u2026'; $('hfBar').style.width='2%'; fetch('/sw?k=relearn&v=1').catch(()=>{}); };
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

// v1.17: room layout stored on the device. The httpd task only queues (rw_room_cmd); the main loop saves to flash.
struct RwRoomSaved { uint32_t magic; char js[200]; };
static char rw_room[2][200] = {"", ""};
static volatile int rw_room_i = 0;
static char rw_room_in[200];
static volatile int rw_room_cmd = 0;          // 1 = save rw_room_in, 2 = reset
static esphome::ESPPreferenceObject rw_room_pref;
static bool rw_room_ready = false;
static void rw_room_set(const char *js) { const int w = 1 - rw_room_i; strlcpy(rw_room[w], js, sizeof rw_room[w]); rw_room_i = w; }

static esp_err_t rw_data(httpd_req_t *req) {
  auto &c = radar_web_cfg(); std::string o; o.reserve(2000); o += "{";
  o += "\"name\":\""; o += esphome::App.get_friendly_name(); o += "\",";
  { const char *rj = rw_room[rw_room_i];   // v1.17: the saved layout wins over the yaml default
    if (rj[0] == '{') { o += "\"room\":"; o += rj; o += ",\"room_src\":\"device\","; }
    else if (c.room_default) { o += "\"room\":"; o += c.room_default; o += ",\"room_src\":\"default\","; } }
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
  { const char *hf = rw_hf[rw_hf_i]; if (hf[0] == '{') { o += "\"hf\":"; o += hf; o += ","; } }
  { const char *aj = rw_aq[rw_aq_i]; if (aj[0] == '{') { o += "\"aq\":"; o += aj; o += ","; } }   // v1.18
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
static esp_err_t rw_room_h(httpd_req_t *req) {   // v1.17: /room?v=<url-encoded json> saves, /room?reset=1 clears
  char q[640], v[420]; httpd_resp_set_type(req, "text/plain");
  if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK) return httpd_resp_sendstr(req, "ERR query");
  if (rw_room_cmd) return httpd_resp_sendstr(req, "ERR busy");
  if (httpd_query_key_value(q, "reset", v, sizeof v) == ESP_OK) { rw_room_cmd = 2; return httpd_resp_sendstr(req, "queued"); }
  if (httpd_query_key_value(q, "v", v, sizeof v) != ESP_OK) return httpd_resp_sendstr(req, "ERR key");
  char d[200]; size_t n = 0;
  for (size_t i = 0; v[i] && n < sizeof d - 1; i++) {
    char ch = v[i];
    if (ch == '%' && v[i + 1] && v[i + 2]) { char hx[3] = {v[i + 1], v[i + 2], 0}; ch = (char) strtol(hx, nullptr, 16); i += 2; }
    else if (ch == '+') ch = ' ';
    d[n++] = ch;
  }
  d[n] = 0;
  if (n < 2 || d[0] != '{' || d[n - 1] != '}') return httpd_resp_sendstr(req, "ERR value");
  for (size_t i = 0; i < n; i++) if (!isalnum((unsigned char) d[i]) && !strchr("{}\":,.-_ ", d[i])) return httpd_resp_sendstr(req, "ERR value");
  strlcpy(rw_room_in, d, sizeof rw_room_in); rw_room_cmd = 1;
  return httpd_resp_sendstr(req, "queued");
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
  if (strcmp(k, "relearn") == 0) { if (!c.hf_relearn) return httpd_resp_sendstr(req, "ERR key"); return httpd_resp_sendstr(req, rw_push("!relearn", 1) ? "queued" : "ERR busy"); }
  if (strcmp(k, "eng") != 0 || !c.eng) return httpd_resp_sendstr(req, "ERR key");
  return httpd_resp_sendstr(req, rw_push("!eng", v[0] == '1' ? 1 : 0) ? "queued" : "ERR busy");
}

// main loop (100 ms yaml interval): perform what the httpd task queued
static void radar_web_pump() {
  auto &c = radar_web_cfg();
  if (c.hf_json) {   // v1.11: refresh the person filter JSON twice a second into the buffer the httpd task is not reading
    static uint32_t last = 0; const uint32_t now = esphome::millis();
    if (now - last >= 500) { last = now; const int w = 1 - rw_hf_i; c.hf_json(rw_hf[w], sizeof rw_hf[w]); rw_hf_i = w; }
  }
  if (rw_room_cmd && rw_room_ready) {   // v1.17: store / clear the room layout in flash
    RwRoomSaved sv; memset(&sv, 0, sizeof sv);
    if (rw_room_cmd == 1) { sv.magic = 0x524F4F4D; strlcpy(sv.js, rw_room_in, sizeof sv.js); rw_room_set(rw_room_in); ESP_LOGI("radar_web", "room layout saved: %s", rw_room_in); }
    else { rw_room_set(""); ESP_LOGI("radar_web", "room layout reset to the default"); }
    rw_room_pref.save(&sv); esphome::global_preferences->sync(); rw_room_cmd = 0;
  }
  while (rw_qh != rw_qt) {
    RwCmd cmd;
    taskENTER_CRITICAL(&rw_mux); cmd = rw_q[rw_qh]; rw_qh = (rw_qh + 1) & 7; taskEXIT_CRITICAL(&rw_mux);
    if (strcmp(cmd.k, "!relearn") == 0) { if (c.hf_relearn) { ESP_LOGI("radar_web", "person filter: relearn room"); c.hf_relearn(); } continue; }
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
    rw_room_pref = esphome::global_preferences->make_preference<RwRoomSaved>(0x52574D31UL);   // v1.17
    { RwRoomSaved sv; if (rw_room_pref.load(&sv) && sv.magic == 0x524F4F4D) { sv.js[sizeof sv.js - 1] = 0; rw_room_set(sv.js); } }
    rw_room_ready = true;
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
  cfg.server_port = 8080; cfg.ctrl_port = 32769; cfg.stack_size = 8192; cfg.max_uri_handlers = 8; cfg.max_open_sockets = 5; cfg.lru_purge_enable = true;
  cfg.keep_alive_enable = true; cfg.keep_alive_idle = 10; cfg.keep_alive_interval = 5; cfg.keep_alive_count = 3;   // a vanished browser frees its slot
  httpd_handle_t srv = nullptr; esp_err_t err = httpd_start(&srv, &cfg);
  if (err != ESP_OK) { ESP_LOGE("radar_web", "httpd_start failed: %s - will retry", esp_err_to_name(err)); return; }
  registered = true;
  static const httpd_uri_t uris[] = {
    {"/", HTTP_GET, rw_html, nullptr}, {"/data.json", HTTP_GET, rw_data, nullptr}, {"/set", HTTP_GET, rw_set, nullptr},
    {"/sw", HTTP_GET, rw_sw, nullptr}, {"/healthz", HTTP_GET, rw_health, nullptr}, {"/room", HTTP_GET, rw_room_h, nullptr},
  };
  for (const auto &u : uris) httpd_register_uri_handler(srv, &u);
  ESP_LOGI("radar_web", "tuning page at http://<device-ip>:8080/");
}
