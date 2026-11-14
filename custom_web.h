#pragma once
#include "esphome.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esphome/core/application.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/wifi/wifi_component.h"
#include <math.h>
#include <string.h>
#include <string>

// ─────────────────────────────────────────────────────────────────
//  NU-AQI-3  Custom Web Dashboard  v4.1  (custom_web.h)
//
//  v4.1: a card shows only its level until the unit has enough history for a change.
//  v4.0: new page in the nuarr look (dark panels, cards, tables), every
//        entity of the unit on it, laid out for desktop and phone.
//    - The main loop builds the JSON (nuaqi_dash_tick(), every 2 s) and the
//      web task only copies it out under a mutex. Before, the web task read
//      sensor and text states itself while the main loop could be writing them.
//    - Entities are looked up by name once, not on every request.
//    - The unit keeps 2 h of one-minute history for 11 readings, so the page
//      has sparklines, a 10-minute change and a 2 h low/high the moment it opens.
//    - The page is sent with an ETag: a reload costs a 304 with no body.
//    - The browser polls every 4 s, stops while the tab is hidden, and only
//      touches the text nodes whose value changed.
//    - httpd: real ctrl_port + TCP keep-alive (ctrl_port 0 with LRU purge
//      wedges the server once its sockets fill).
//
//  Endpoints on port 8080:
//    /  /dash    → dashboard HTML
//    /data.json  → every current value, one small document
//    /hist.json  → 2 h history, one sample a minute
//    /healthz    → "OK"
//
//  Levels (same as the Room AQI card in Home Assistant):
//    0 Excellent · 1 Good · 2 Monitor · 3 Circulate · 4 Fresh air
//    CO2 600 / 800 / 1000 / 1400 ppm; VOC >150 / 250 / 350 / 450;
//    NOx >5 / 10 / 20 / 50; PM AQI 25 / 50 / 100 / 150.
// ─────────────────────────────────────────────────────────────────

#define NUAQI_PAGE_VERSION "4.1"
#define NUAQI_SNAP_CAP 2048
#define NUAQI_HS 11        // history series (the first NUAQI_HS rows of NUAQI_NUMS)
#define NUAQI_HN 120       // samples kept per series
#define NUAQI_HIST_STEP_MS 60000UL

static const char *const NUAQI_TAG = "nuaqi_dash";

static std::string sanitize_id(const std::string &s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (c >= 'A' && c <= 'Z') out += (char)(c + 32);
    else if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_') out += c;
    else out += '_';
  }
  return out;
}

// key = JSON key, name = entity name run through sanitize_id()
struct NuaqiNum  { const char *key; const char *name; uint8_t dec; esphome::sensor::Sensor *s; };
struct NuaqiText { const char *key; const char *name; esphome::text_sensor::TextSensor *s; };

// The first NUAQI_HS rows are also the history series, in this order.
static NuaqiNum NUAQI_NUMS[] = {
  {"co2",  "co2",                 0, nullptr},
  {"voc",  "voc",                 1, nullptr},
  {"vocf", "voc_fast",            0, nullptr},
  {"nox",  "nox",                 0, nullptr},
  {"pm1",  "pm__1__m",            1, nullptr},
  {"pm25", "pm__2_5__m",          1, nullptr},
  {"pm4",  "pm__4__m",            1, nullptr},
  {"pm10", "pm__10__m",           1, nullptr},
  {"pma",  "average_pm_aqi",      0, nullptr},
  {"t",    "temperature",         2, nullptr},
  {"rh",   "humidity",            1, nullptr},
  {"ah",   "absolute_humidity",   2, nullptr},
  {"fl",   "aqi_feels_like_temp", 1, nullptr},
  {"off",  "temp_offset_applied", 1, nullptr},
  {"esp",  "esp32_internal_temperature__f_", 0, nullptr},
  {"cpu",  "esp32_cpu_frequency", 0, nullptr},
  {"wq",   "wifi_signal_quality", 0, nullptr},
  {"tx",   "wifi_tx_power",       1, nullptr},
  {"rc",   "wifi_reconnects",     0, nullptr},
  {"dc",   "wifi_disconnects",    0, nullptr},
};
static NuaqiText NUAQI_TEXTS[] = {
  {"co2T",  "aqi_co2",                 nullptr},
  {"vocT",  "aqi_tvoc",                nullptr},
  {"noxT",  "aqi_nox",                 nullptr},
  {"pmT",   "pm_aqi",                  nullptr},
  {"ahT",   "aqi_ah",                  nullptr},
  {"lvlT",  "aqi_combined_level",      nullptr},
  {"unitT", "unit_temperature_status", nullptr},
  {"ip",    "esp32_ip_address",        nullptr},
  {"ssid",  "ssid",                    nullptr},
  {"bssid", "bssid",                   nullptr},
  {"ap",    "access_point",            nullptr},
};
static const size_t NUAQI_NUM_COUNT  = sizeof(NUAQI_NUMS) / sizeof(NUAQI_NUMS[0]);
static const size_t NUAQI_TEXT_COUNT = sizeof(NUAQI_TEXTS) / sizeof(NUAQI_TEXTS[0]);

static esphome::binary_sensor::BinarySensor *nuaqi_ready_bs = nullptr;
static esphome::switch_::Switch *nuaqi_sw_night = nullptr;
static esphome::switch_::Switch *nuaqi_sw_adapt = nullptr;
static esphome::number::Number  *nuaqi_num_base = nullptr;
static esphome::number::Number  *nuaqi_num_coef = nullptr;
static esphome::light::LightState *nuaqi_led = nullptr;

static SemaphoreHandle_t nuaqi_lock = nullptr;
static char   nuaqi_snap[NUAQI_SNAP_CAP];
static size_t nuaqi_snap_len = 0;
static float    nuaqi_hist[NUAQI_HS][NUAQI_HN];
static uint16_t nuaqi_hist_count = 0;     // samples stored (max NUAQI_HN)
static uint16_t nuaqi_hist_head  = 0;     // next slot to write
static uint32_t nuaqi_hist_ms    = 0;     // millis() of the latest sample

// Looks every entity up by name, once. Runs in the main loop.
static void nuaqi_resolve() {
  for (auto *s : esphome::App.get_sensors()) {
    std::string n = sanitize_id(s->get_name());
    for (size_t i = 0; i < NUAQI_NUM_COUNT; i++)
      if (NUAQI_NUMS[i].s == nullptr && n == NUAQI_NUMS[i].name) NUAQI_NUMS[i].s = s;
  }
  for (auto *s : esphome::App.get_text_sensors()) {
    std::string n = sanitize_id(s->get_name());
    for (size_t i = 0; i < NUAQI_TEXT_COUNT; i++)
      if (NUAQI_TEXTS[i].s == nullptr && n == NUAQI_TEXTS[i].name) NUAQI_TEXTS[i].s = s;
  }
  for (auto *s : esphome::App.get_binary_sensors())
    if (sanitize_id(s->get_name()) == "nu_aqi_3_sensor_ready") nuaqi_ready_bs = s;
  for (auto *s : esphome::App.get_switches()) {
    std::string n = sanitize_id(s->get_name());
    if (n == "aqi_auto_night_mode") nuaqi_sw_night = s;
    else if (n == "adaptive_wifi_tx_power") nuaqi_sw_adapt = s;
  }
  for (auto *s : esphome::App.get_numbers()) {
    std::string n = sanitize_id(s->get_name());
    if (n == "temp_offset_base") nuaqi_num_base = s;
    else if (n == "temp_offset_heat_coefficient") nuaqi_num_coef = s;
  }
  for (auto *s : esphome::App.get_lights())
    if (sanitize_id(s->get_name()) == "aqi_led") nuaqi_led = s;

  for (size_t i = 0; i < NUAQI_NUM_COUNT; i++)
    if (NUAQI_NUMS[i].s == nullptr) ESP_LOGW(NUAQI_TAG, "sensor '%s' not found", NUAQI_NUMS[i].name);
  for (size_t i = 0; i < NUAQI_TEXT_COUNT; i++)
    if (NUAQI_TEXTS[i].s == nullptr) ESP_LOGW(NUAQI_TAG, "text sensor '%s' not found", NUAQI_TEXTS[i].name);
}

// Bounded JSON writer: never writes past the buffer, drops what does not fit.
struct NuaqiBuf {
  char *p; size_t cap; size_t n;
  void raw(const char *s) { size_t l = strlen(s); if (n + l < cap) { memcpy(p + n, s, l); n += l; } }
  void key(const char *k) { raw("\""); raw(k); raw("\":"); }
  void num(const char *k, float v, int dec) {
    key(k);
    if (isnan(v) || isinf(v)) raw("null");
    else { char b[24]; snprintf(b, sizeof(b), "%.*f", dec, (double) v); raw(b); }
    raw(",");
  }
  void u32(const char *k, uint32_t v) { key(k); char b[16]; snprintf(b, sizeof(b), "%u", (unsigned) v); raw(b); raw(","); }
  void boolean(const char *k, bool known, bool v) { key(k); raw(!known ? "null" : v ? "true" : "false"); raw(","); }
  void str(const char *k, const char *s, size_t len) {
    key(k);
    if (s == nullptr) { raw("null,"); return; }
    if (n + len * 2 + 4 >= cap) { raw("null,"); return; }
    p[n++] = '"';
    for (size_t i = 0; i < len; i++) {
      unsigned char c = (unsigned char) s[i];
      if (c < 0x20) continue;
      if (c == '"' || c == '\\') p[n++] = '\\';
      p[n++] = (char) c;
    }
    p[n++] = '"'; p[n++] = ',';
  }
  void end() { if (n > 0 && p[n - 1] == ',') n--; if (n + 1 < cap) p[n++] = '}'; p[n] = 0; }
};

static float nuaqi_value(const NuaqiNum &e) {
  return (e.s != nullptr && e.s->has_state()) ? e.s->state : NAN;
}

// Builds /data.json. Runs in the main loop, so every state read is safe.
static size_t nuaqi_build(char *out, size_t cap) {
  NuaqiBuf b{out, cap, 0};
  b.raw("{");
  const auto &name = esphome::App.get_friendly_name();
  b.str("name", name.c_str(), strlen(name.c_str()));
  b.str("page", NUAQI_PAGE_VERSION, strlen(NUAQI_PAGE_VERSION));
  b.str("ver", ESPHOME_VERSION, strlen(ESPHOME_VERSION));
#ifdef ESPHOME_PROJECT_VERSION
  b.str("proj", ESPHOME_PROJECT_VERSION, strlen(ESPHOME_PROJECT_VERSION));
#endif
  b.str("built", __DATE__ " " __TIME__, strlen(__DATE__ " " __TIME__));
  b.u32("up", (uint32_t) (esp_timer_get_time() / 1000000LL));
  b.u32("heap", (uint32_t) esp_get_free_heap_size());
  if (esphome::wifi::global_wifi_component != nullptr && esphome::wifi::global_wifi_component->is_connected())
    b.num("rssi", (float) esphome::wifi::global_wifi_component->wifi_rssi(), 0);

  b.boolean("ready", nuaqi_ready_bs != nullptr && nuaqi_ready_bs->has_state(),
            nuaqi_ready_bs != nullptr && nuaqi_ready_bs->state);
  b.boolean("night", nuaqi_sw_night != nullptr, nuaqi_sw_night != nullptr && nuaqi_sw_night->state);
  b.boolean("adapt", nuaqi_sw_adapt != nullptr, nuaqi_sw_adapt != nullptr && nuaqi_sw_adapt->state);
  b.num("offB", (nuaqi_num_base != nullptr && nuaqi_num_base->has_state()) ? nuaqi_num_base->state : NAN, 2);
  b.num("offK", (nuaqi_num_coef != nullptr && nuaqi_num_coef->has_state()) ? nuaqi_num_coef->state : NAN, 2);
  if (nuaqi_led != nullptr) {
    const auto &v = nuaqi_led->remote_values;
    float br = v.get_brightness();
    char c[8];
    snprintf(c, sizeof(c), "#%02x%02x%02x", (int) (v.get_red() * 255.0f + 0.5f), (int) (v.get_green() * 255.0f + 0.5f),
             (int) (v.get_blue() * 255.0f + 0.5f));
    b.boolean("led", true, v.is_on());
    b.num("ledB", br * 100.0f, 0);
    b.str("ledC", c, 7);
  }

  for (size_t i = 0; i < NUAQI_NUM_COUNT; i++) b.num(NUAQI_NUMS[i].key, nuaqi_value(NUAQI_NUMS[i]), NUAQI_NUMS[i].dec);
  for (size_t i = 0; i < NUAQI_TEXT_COUNT; i++) {
    auto *s = NUAQI_TEXTS[i].s;
    if (s != nullptr && s->has_state()) b.str(NUAQI_TEXTS[i].key, s->state.c_str(), s->state.size());
    else b.str(NUAQI_TEXTS[i].key, nullptr, 0);
  }
  b.end();
  return b.n;
}

// ─────────────────────────────────────────────────────────────────
// Dashboard HTML
// ─────────────────────────────────────────────────────────────────
static const char NUAQI_DASH_HTML[] = R"ESPH(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0f1216">
<title>NU-AQI - air quality</title>
<link rel="icon" href="data:,">
<style>
:root{--bg:#0f1216;--panel:#171b21;--line:#242a33;--fg:#e6e9ef;--dim:#8b95a5;--ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#58a6ff;color-scheme:dark}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,Segoe UI,sans-serif;-webkit-text-size-adjust:100%}
header{padding:12px 20px;border-bottom:1px solid var(--line);display:flex;align-items:center;gap:10px;position:sticky;top:0;background:var(--bg);z-index:5}
h1{font-size:17px;margin:0;letter-spacing:.5px;white-space:nowrap}
h1 b{font-weight:inherit}h1 span{color:var(--dim);font-weight:400;font-size:13px;margin-left:8px}
.sp{flex:1}
.dim{color:var(--dim)}
.pill{display:inline-block;padding:1px 9px;border-radius:20px;font-size:11px;border:1px solid;white-space:nowrap;color:var(--c,var(--dim))}
.dot{width:6px;height:6px;border-radius:50%;background:var(--ok);animation:pulse 2s infinite;flex:none}
@keyframes pulse{50%{opacity:.35}}
.off .dot{background:var(--bad);animation:none}
.off .wrap{opacity:.55}
#upd{font-size:12px;font-variant-numeric:tabular-nums;white-space:nowrap}
.wrap{padding:16px 20px calc(16px + env(safe-area-inset-bottom));display:grid;gap:14px;max-width:1320px;margin:0 auto}
.t0{--c:var(--ok)}.t1{--c:#c8c24a}.t2{--c:var(--warn)}.t3{--c:var(--bad)}.t4{--c:#b48bf2}.tw{--c:var(--acc)}.tn{--c:var(--dim)}
.tc{color:var(--c,var(--fg))}
.note{background:var(--panel);border:1px solid var(--line);border-left:3px solid var(--c,var(--dim));border-radius:9px;padding:10px 14px}
.cards{display:grid;grid-template-columns:repeat(8,1fr);gap:12px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:9px;padding:12px 14px;min-width:0}
.k{color:var(--dim);font-size:11px;text-transform:uppercase;letter-spacing:.7px}
.v{font-size:23px;margin-top:5px;font-variant-numeric:tabular-nums;white-space:nowrap}
.v small{font-size:12px;color:var(--dim);margin-left:4px}
.s{color:var(--dim);font-size:12px;margin-top:2px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.card svg{display:block;width:100%;height:26px;margin-top:8px;color:var(--c,var(--acc))}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:14px;align-items:start}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:9px;overflow:hidden;min-width:0}
.panel h2{margin:0;padding:11px 14px;font-size:13px;border-bottom:1px solid var(--line);color:var(--dim);text-transform:uppercase;letter-spacing:.7px;font-weight:600}
table{width:100%;border-collapse:collapse;font-size:13px}
th{text-align:left;color:var(--dim);font-weight:500;padding:7px 14px;border-bottom:1px solid var(--line);font-size:11px;text-transform:uppercase;white-space:nowrap}
td{padding:7px 14px;border-bottom:1px solid var(--line);vertical-align:top;font-variant-numeric:tabular-nums}
tr:last-child td{border-bottom:none}
td small{display:block;color:var(--dim);font-size:11px}
td b{font-weight:600}
.r{text-align:right}
.up{color:var(--warn)}.dn{color:var(--acc)}
.foot{padding:8px 14px;border-top:1px solid var(--line);color:var(--dim);font-size:12px}
.kv>div{display:flex;justify-content:space-between;gap:14px;padding:7px 14px;border-bottom:1px solid var(--line)}
.kv>div:last-child{border-bottom:none}
.kv span{color:var(--dim);white-space:nowrap}
.kv b{font-weight:500;text-align:right;overflow-wrap:anywhere;font-variant-numeric:tabular-nums}
.on{color:var(--ok)}
.sw{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px;border:1px solid var(--line);vertical-align:-1px}
@media(hover:hover){tr:hover td{background:#1c2129}}
@media(max-width:1100px){.cards{grid-template-columns:repeat(4,1fr)}}
@media(max-width:640px){.cards{grid-template-columns:repeat(2,1fr);gap:10px}header{padding:10px 14px}h1 span{display:none}.wrap{padding:12px 12px calc(12px + env(safe-area-inset-bottom));gap:12px}.card{padding:10px 12px}.v{font-size:21px}td,th{padding:7px 10px}.hm{display:none}}
@media(prefers-reduced-motion:reduce){.dot{animation:none}}
</style></head><body>
<header><h1><b id="nm">NU-AQI</b><span>air quality</span></h1><div class="sp"></div><span class="pill tn" id="lvl">connecting</span><span class="dot"></span><span class="dim" id="upd"></span></header>
<div class="wrap">
<div class="note tn" id="note">Loading…</div>
<div class="cards" id="cards"></div>
<div class="panel"><h2>Air readings</h2><table><thead><tr><th>Reading</th><th class="r">Now</th><th>Level</th><th class="r">10 min</th><th class="r hm">2 h low</th><th class="r hm">2 h high</th></tr></thead><tbody id="air"></tbody></table>
<div class="foot">CO₂: circulate from 1000 ppm, fresh air from 1400 ppm. Changes and lows/highs come from the unit's own 2 h history.</div></div>
<div class="grid">
<div class="panel"><h2>Comfort</h2><div class="kv" id="p0"></div></div>
<div class="panel"><h2>Wi-Fi</h2><div class="kv" id="p1"></div></div>
<div class="panel"><h2>Device</h2><div class="kv" id="p2"></div></div>
<div class="panel"><h2>Settings</h2><div class="kv" id="p3"></div><div class="foot">Shown only. Change them in Home Assistant.</div></div>
</div></div>
<script>
(()=>{
const $=i=>document.getElementById(i),TL=['Excellent','Good','Monitor','Circulate','Fresh air'];
// label, unit, decimals, level steps, 10-minute change [noticeable, fast], unit's own text
const M={
co2:['CO₂','ppm',0,[600,800,1000,1400],[20,80],'co2T'],
voc:['VOC','index',0,[150.01,250.01,350.01,450.01],[8,30],'vocT'],
vocf:['VOC fast','index',0,[150.01,250.01,350.01,450.01],[8,30]],
nox:['NOx','index',0,[5.01,10,20,50],[2,8],'noxT'],
pm1:['PM1','µg/m³',1,[5,5,20,35],[2,10]],
pm25:['PM2.5','µg/m³',1,[25,50,100,150],[2,10]],
pm4:['PM4','µg/m³',1,[5,5,20,35],[2,10]],
pm10:['PM10','µg/m³',1,[20,20,75,150],[2,10]],
pma:['PM AQI','NowCast',0,[25,50,100,150],[3,12],'pmT'],
t:['Temperature','°F',1,0,[.4,1.5]],
rh:['Humidity','%',1,0,[1,4]]};
const CARDS=['co2','voc','vocf','nox','pma','pm25','t','rh'],ROWS=['co2','voc','vocf','nox','pm1','pm25','pm4','pm10','pma'];
const KV=[
[['tF','Temperature'],['rh2','Humidity'],['fl','Feels like'],['ah','Absolute humidity'],['ahT','Humidity status'],['off','Temperature offset applied']],
[['ssid','Network'],['ap','Access point'],['bssid','BSSID'],['ip','IP address'],['wq','Signal quality'],['rssi','Signal strength'],['tx','Transmit power'],['rc','Reconnects'],['dc','Disconnects']],
[['ready','Sensor ready'],['lvlT','Combined level'],['up','Uptime'],['fw','Firmware'],['built','Built'],['page','Page'],['cpu','CPU speed'],['esp','Chip temperature'],['unitT','Chip temperature status'],['heap','Free memory']],
[['night','Auto night mode'],['adapt','Adaptive transmit power'],['led','AQI light'],['offB','Temperature offset base'],['offK','Heat coefficient']]];
let D={},H={},bad=0,tmr=0,hNext=0;

// build the fixed parts once
let h='';
CARDS.forEach(k=>{h+='<div class="card"><div class="k">'+M[k][0]+'</div><div class="v"><span id="v-'+k+'">—</span><small>'+(k==='pma'?'AQI':M[k][1])+'</small></div><div class="s" id="s-'+k+'">&nbsp;</div><svg id="q-'+k+'" viewBox="0 0 120 26" preserveAspectRatio="none"><polyline id="g-'+k+'" fill="none" stroke="currentColor" stroke-width="1.5" vector-effect="non-scaling-stroke"/></svg></div>'});
$('cards').innerHTML=h;h='';
ROWS.forEach(k=>{h+='<tr><td>'+M[k][0]+'<small id="x-'+k+'"></small></td><td class="r"><b id="r-'+k+'">—</b> <span class="dim">'+(k==='pma'?'':M[k][1])+'</span></td><td id="l-'+k+'">—</td><td class="r" id="d-'+k+'">—</td><td class="r hm" id="lo-'+k+'">—</td><td class="r hm" id="hi-'+k+'">—</td></tr>'});
$('air').innerHTML=h;
KV.forEach((p,i)=>{h='';p.forEach(r=>{h+='<div><span>'+r[1]+'</span><b id="k-'+r[0]+'">—</b></div>'});$('p'+i).innerHTML=h});

// write a node only when its text or class really changed
function tx(id,s,c){const e=$(id);if(!e)return;if(e.textContent!==s)e.textContent=s;if(c!==undefined&&e.className!==c)e.className=c}
const conv=(k,v)=>v==null?null:k==='t'?v*1.8+32:v;
const val=k=>conv(k,D[k]);
const fmt=(v,d)=>v==null?'—':d?v.toFixed(d):Math.round(v).toLocaleString();
function tier(k,v){const s=M[k][3];if(!s||v==null)return -1;let t=0;while(t<4&&v>=s[t])t++;return t}
const tcl=t=>t<0?'tn':'t'+t;
// history plus the live value, oldest first
function ser(k){const a=(H[k]||[]).map(v=>conv(k,v)),v=val(k);if(v!=null)a.push(v);return a}
// change over about 10 minutes; null until there is enough history
function delta(k){const a=ser(k),n=a.length;if(n<3||a[n-1]==null)return null;let i=Math.max(0,n-11);while(i<n-1&&a[i]==null)i++;return i<n-1?a[n-1]-a[i]:null}
// -2 falling fast .. 0 steady .. 2 rising fast
function trend(k,d){if(d==null)return 0;const s=M[k][4],m=Math.abs(d);return m<s[0]?0:(d>0?1:-1)*(m>=s[1]?2:1)}
function dtx(k,d){if(d==null)return'—';const m=Math.abs(d),s=M[k][2]||m<10?m.toFixed(1):Math.round(m)+'';return(d>0?'+':d<0?'−':'')+s}
function spark(k){const a=ser(k).filter(v=>v!=null),p=$('g-'+k),n=a.length;if(!p)return;if(n<2){p.setAttribute('points','');return}
let lo=Math.min(...a),hi=Math.max(...a),r=Math.max(hi-lo,M[k][4][1]),m=(lo+hi)/2,s='';
for(let i=0;i<n;i++)s+=(i*120/(n-1)).toFixed(1)+','+(13-(a[i]-m)/r*22).toFixed(1)+' ';
if(p.getAttribute('points')!==s)p.setAttribute('points',s)}
function upTxt(s){if(s==null)return'—';const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return d?d+' d '+h+' h':h?h+' h '+m+' min':m?m+' min':s+' s'}
const onoff=(id,v)=>tx('k-'+id,v==null?'—':v?'On':'Off',v?'on':'');
const txt=(id,v)=>tx('k-'+id,v==null||v===''?'—':String(v));
const num=(id,v,d,u)=>tx('k-'+id,v==null?'—':fmt(v,d)+u);

function note(lv,ready){
if(!ready)return['tw','Sensor is warming up. Readings appear shortly.'];
const why=['co2','voc','nox','pma'].filter(k=>tier(k,val(k))===lv&&lv>0).map(k=>M[k][0]+' '+fmt(val(k),0)+(k==='co2'?' ppm':'')).join(', ');
const tr=trend('co2',delta('co2')),rise=tr>0?' CO₂ is rising'+(tr>1?' fast.':'.'):'';
if(lv>=4)return['t4','Bring in fresh air now: '+why+'.'];
if(lv>=3)return['t3','Recirculate air: '+why+'.'+rise];
if(lv>=2)return['t2','Monitor: '+why+'.'+(rise||' No action needed yet.')];
if(lv>=1)return['t1','Good: '+why+'.'+(rise||' Air quality is acceptable.')];
return['t0','Air quality is excellent. No ventilation needed.'+rise]}

function render(){
const ready=D.ready===true||D.co2!=null;
const lv=ready?Math.max(0,tier('co2',val('co2')),tier('voc',val('voc')),tier('nox',val('nox')),tier('pma',val('pma'))):-1;
if(D.name){tx('nm',D.name);const t=D.name+' - air quality';if(document.title!==t)document.title=t}
tx('lvl',ready?TL[lv]:'Warming up','pill '+(ready?tcl(lv):'tw'));
const n=note(lv,ready);tx('note',n[1],'note '+n[0]);
CARDS.forEach(k=>{const v=val(k),t=tier(k,v),d=delta(k),tr=trend(k,d),c=M[k][3]?(ready?tcl(t):'tw'):'';
tx('v-'+k,fmt(v,M[k][2]),'tc '+c);
const lb=t>=0&&ready?TL[t]:'';tx('s-'+k,d==null?(lb||'no history yet'):(lb?lb+' · ':'')+(tr?(tr>0?'▲ ':'▼ ')+dtx(k,d):'steady'));
const q=$('q-'+k);if(q.getAttribute('class')!==c)q.setAttribute('class',c);spark(k)});
ROWS.forEach(k=>{const v=val(k),t=tier(k,v),d=delta(k),tr=trend(k,d),a=ser(k).filter(x=>x!=null),dc=M[k][2];
tx('r-'+k,fmt(v,dc),'tc '+tcl(t));tx('l-'+k,t<0?'—':TL[t],'tc '+tcl(t));
tx('d-'+k,d==null?'—':(tr>0?'▲ ':tr<0?'▼ ':'')+dtx(k,d),'r '+(tr>0?'up':tr<0?'dn':'dim'));
tx('lo-'+k,a.length?fmt(Math.min(...a),dc):'—');tx('hi-'+k,a.length?fmt(Math.max(...a),dc):'—');
if(M[k][5])tx('x-'+k,D[M[k][5]]||'')});
const tc=D.t;tx('k-tF',tc==null?'—':(tc*1.8+32).toFixed(1)+' °F · '+tc.toFixed(1)+' °C');
num('rh2',D.rh,1,' %');num('fl',D.fl,1,' °F');num('ah',D.ah,2,' g/m³');txt('ahT',D.ahT);num('off',D.off,1,' °F');
txt('ssid',D.ssid);txt('ap',D.ap);txt('bssid',D.bssid);txt('ip',D.ip);num('wq',D.wq,0,' %');num('rssi',D.rssi,0,' dBm');num('tx',D.tx,1,' dBm');num('rc',D.rc,0,'');num('dc',D.dc,0,'');
tx('k-ready',D.ready==null?'—':D.ready?'Yes':'Warming up',D.ready?'on':'');txt('lvlT',D.lvlT);tx('k-up',upTxt(D.up));
txt('fw',(D.proj?'v'+D.proj+' · ':'')+(D.ver?'ESPHome '+D.ver:''));txt('built',D.built);txt('page',D.page?'v'+D.page:null);
num('cpu',D.cpu,0,' MHz');num('esp',D.esp,0,' °F');txt('unitT',D.unitT);num('heap',D.heap==null?null:D.heap/1024,0,' KB');
onoff('night',D.night);onoff('adapt',D.adapt);
const e=$('k-led'),ls=D.led==null?'—':D.led?'On · '+fmt(D.ledB,0)+' %':'Off';
if(e.dataset.s!==ls+D.ledC){e.dataset.s=ls+D.ledC;e.textContent=ls;if(D.led&&D.ledC){const i=document.createElement('i');i.className='sw';i.style.background=D.ledC;e.prepend(i)}}
num('offB',D.offB,2,' °C');num('offK',D.offK,2,'');
}

async function get(u){const c=new AbortController(),t=setTimeout(()=>c.abort(),4000);
try{const r=await fetch(u,{signal:c.signal,cache:'no-store'});if(!r.ok)throw new Error(r.status);return await r.json()}finally{clearTimeout(t)}}
async function tick(){
clearTimeout(tmr);if(document.hidden)return;
if(Date.now()>=hNext){try{const j=await get('/hist.json'),o={};j.k.forEach((k,i)=>o[k]=j.d[i]);H=o;hNext=Date.now()+(62-Math.min(j.age,60))*1000}catch(e){hNext=Date.now()+15000}}
try{D=await get('/data.json');bad=0;if(document.body.className)document.body.className='';render();tx('upd',new Date().toLocaleTimeString())}
catch(e){if(++bad>1){document.body.className='off';tx('lvl','Offline','pill t3');tx('upd','no answer × '+bad)}}
tmr=setTimeout(tick,4000)}
document.addEventListener('visibilitychange',()=>{if(!document.hidden)tick()});
tick();
})();
</script></body></html>
)ESPH";

// ─────────────────────────────────────────────────────────────────
//  C++ HTTP server — port 8080
// ─────────────────────────────────────────────────────────────────

// Changes with every firmware build, so a browser re-fetches the page only then.
static const char NUAQI_ETAG[] = "\"" NUAQI_PAGE_VERSION "-" __DATE__ "-" __TIME__ "\"";

static esp_err_t nuaqi_html_handler(httpd_req_t *req) {
  char seen[64];
  httpd_resp_set_hdr(req, "ETag", NUAQI_ETAG);
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  if (httpd_req_get_hdr_value_str(req, "If-None-Match", seen, sizeof(seen)) == ESP_OK && strcmp(seen, NUAQI_ETAG) == 0) {
    httpd_resp_set_status(req, "304 Not Modified");
    return httpd_resp_send(req, nullptr, 0);
  }
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, NUAQI_DASH_HTML, sizeof(NUAQI_DASH_HTML) - 1);
}

static esp_err_t nuaqi_healthz_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, "OK");
}

// Sends the snapshot the main loop built. One web task serves every request, so the static buffer is safe.
static esp_err_t nuaqi_data_handler(httpd_req_t *req) {
  static char out[NUAQI_SNAP_CAP];
  size_t n = 0;
  if (nuaqi_lock != nullptr && xSemaphoreTake(nuaqi_lock, pdMS_TO_TICKS(300)) == pdTRUE) {
    n = nuaqi_snap_len;
    memcpy(out, nuaqi_snap, n);
    xSemaphoreGive(nuaqi_lock);
  }
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  if (n == 0) return httpd_resp_sendstr(req, "{}");
  return httpd_resp_send(req, out, n);
}

// 2 h of history: copied out under the lock, then written one series per chunk.
static esp_err_t nuaqi_hist_handler(httpd_req_t *req) {
  static float copy[NUAQI_HS][NUAQI_HN];
  uint16_t count = 0, head = 0;
  uint32_t age = 0;
  if (nuaqi_lock != nullptr && xSemaphoreTake(nuaqi_lock, pdMS_TO_TICKS(300)) == pdTRUE) {
    memcpy(copy, nuaqi_hist, sizeof(copy));
    count = nuaqi_hist_count;
    head = nuaqi_hist_head;
    age = (esphome::millis() - nuaqi_hist_ms) / 1000;
    xSemaphoreGive(nuaqi_lock);
  }
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  char buf[1280];
  int n = snprintf(buf, sizeof(buf), "{\"step\":%u,\"age\":%u,\"n\":%u,\"k\":[", (unsigned) (NUAQI_HIST_STEP_MS / 1000),
                   (unsigned) age, (unsigned) count);
  for (int s = 0; s < NUAQI_HS; s++)
    n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\"", s ? "," : "", NUAQI_NUMS[s].key);
  n += snprintf(buf + n, sizeof(buf) - n, "],\"d\":[");
  if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) return ESP_FAIL;

  for (int s = 0; s < NUAQI_HS; s++) {
    n = 0;
    buf[n++] = '[';
    for (uint16_t i = 0; i < count; i++) {
      // oldest first: when the ring is full the oldest sample sits at head
      uint16_t idx = (uint16_t) ((head + NUAQI_HN - count + i) % NUAQI_HN);
      float v = copy[s][idx];
      if ((size_t) n + 16 >= sizeof(buf)) break;
      if (i) buf[n++] = ',';
      if (isnan(v) || isinf(v)) n += snprintf(buf + n, sizeof(buf) - n, "null");
      else n += snprintf(buf + n, sizeof(buf) - n, "%.*f", NUAQI_NUMS[s].dec, (double) v);
    }
    buf[n++] = ']';
    if (s < NUAQI_HS - 1) buf[n++] = ',';
    if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) return ESP_FAIL;
  }
  if (httpd_resp_send_chunk(req, "]}", 2) != ESP_OK) return ESP_FAIL;
  return httpd_resp_send_chunk(req, nullptr, 0);
}

// Starts the server. Safe to call repeatedly: returns at once when it is running,
// and tries again on the next call when the start failed.
void nuaqi_dash_register() {
  static bool registered = false;
  if (registered) return;
  if (nuaqi_lock == nullptr) nuaqi_lock = xSemaphoreCreateMutex();
  if (nuaqi_lock == nullptr) return;

  httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
  config.server_port      = 8080;
  // ctrl_port must be a real port: 0 binds an ephemeral one, but httpd still sends its work queue to
  // port 0, so the LRU purge never closes a socket and the server stops answering once they are all taken.
  config.ctrl_port        = 32769;
  config.stack_size       = 10240;
  config.max_uri_handlers = 6;
  config.max_open_sockets = 4;
  config.lru_purge_enable = true;
  config.keep_alive_enable   = true;    // a browser that vanished frees its socket
  config.keep_alive_idle     = 10;
  config.keep_alive_interval = 5;
  config.keep_alive_count    = 3;

  httpd_handle_t server = nullptr;
  esp_err_t err = httpd_start(&server, &config);
  if (err != ESP_OK) {
    ESP_LOGE(NUAQI_TAG, "httpd_start failed: 0x%x (%s) — will retry", err, esp_err_to_name(err));
    return;
  }
  registered = true;

  static const httpd_uri_t uris[] = {
    { "/",          HTTP_GET, nuaqi_html_handler,    nullptr },
    { "/dash",      HTTP_GET, nuaqi_html_handler,    nullptr },
    { "/data.json", HTTP_GET, nuaqi_data_handler,    nullptr },
    { "/hist.json", HTTP_GET, nuaqi_hist_handler,    nullptr },
    { "/healthz",   HTTP_GET, nuaqi_healthz_handler, nullptr },
  };
  for (const auto &u : uris) httpd_register_uri_handler(server, &u);

  ESP_LOGI(NUAQI_TAG, "Dashboard v" NUAQI_PAGE_VERSION " running at http://<device-ip>:8080/ (page %u bytes)",
           (unsigned) (sizeof(NUAQI_DASH_HTML) - 1));
}

// Call from the main loop every 2 s (interval: in the yaml). Builds the JSON the web task serves
// and adds one history sample a minute. Never blocks: when the web task holds the lock, this
// round is skipped and the next one catches up.
void nuaqi_dash_tick() {
  static bool resolved = false;
  static uint32_t last_try = 0;
  static char work[NUAQI_SNAP_CAP];
  uint32_t now = esphome::millis();

  if (nuaqi_lock == nullptr || now - last_try >= 10000 || last_try == 0) {
    last_try = now ? now : 1;
    nuaqi_dash_register();
  }
  if (nuaqi_lock == nullptr) return;
  if (!resolved) { resolved = true; nuaqi_resolve(); }

  size_t n = nuaqi_build(work, sizeof(work));
  bool sample = nuaqi_hist_count == 0 || now - nuaqi_hist_ms >= NUAQI_HIST_STEP_MS;

  if (xSemaphoreTake(nuaqi_lock, 0) != pdTRUE) return;
  memcpy(nuaqi_snap, work, n + 1);
  nuaqi_snap_len = n;
  if (sample) {
    for (int s = 0; s < NUAQI_HS; s++) nuaqi_hist[s][nuaqi_hist_head] = nuaqi_value(NUAQI_NUMS[s]);
    nuaqi_hist_head = (uint16_t) ((nuaqi_hist_head + 1) % NUAQI_HN);
    if (nuaqi_hist_count < NUAQI_HN) nuaqi_hist_count++;
    nuaqi_hist_ms = now;
  }
  xSemaphoreGive(nuaqi_lock);
}
