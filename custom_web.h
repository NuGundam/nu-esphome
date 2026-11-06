#pragma once
#include "esphome.h"
#include "esp_http_server.h"
#include "esphome/core/application.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <math.h>
#include <string>

// ──────────────────────────────────────────────────────────────────
//  NU-AQI-3  Custom Web Dashboard  v3.3  (custom_web.h)
//
//  Architecture:
//    1. Port 8080 serves the dashboard HTML + a single /data.json endpoint.
//    2. /data.json reads sensor values DIRECTLY from C++ globals via
//       App.get_sensors() — no HTTP loopback, no proxy, no CORS issues.
//    3. JS makes ONE fetch every 8 seconds.
//
//  CO₂ tier thresholds (aligned to standard ventilation guidelines):
//    Tier 0 Excellent: < 600 ppm
//    Tier 1 Good:      600–800 ppm
//    Tier 2 Monitor:   800–1000 ppm  (ASHRAE upper bound, slight stuffiness)
//    Tier 3 Circulate: 1000–1400 ppm (noticeable fatigue, recirculate air)
//    Tier 4 Fresh air: ≥ 1400 ppm    (significant impairment, open window/ERV)
//
//  Endpoints on port 8080:
//    /          → dashboard HTML
//    /dash      → alias
//    /data.json → all sensor values in one JSON document
//    /healthz   → "OK" liveness ping
// ──────────────────────────────────────────────────────────────────

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

static esphome::sensor::Sensor *find_sensor(const char *oid) {
  std::string t = sanitize_id(oid);
  for (auto *s : esphome::App.get_sensors())
    if (sanitize_id(s->get_name()) == t) return s;
  return nullptr;
}
static esphome::text_sensor::TextSensor *find_text_sensor(const char *oid) {
  std::string t = sanitize_id(oid);
  for (auto *s : esphome::App.get_text_sensors())
    if (sanitize_id(s->get_name()) == t) return s;
  return nullptr;
}
static esphome::binary_sensor::BinarySensor *find_binary_sensor(const char *oid) {
  std::string t = sanitize_id(oid);
  for (auto *s : esphome::App.get_binary_sensors())
    if (sanitize_id(s->get_name()) == t) return s;
  return nullptr;
}

static void append_sensor(std::string &out, const char *key, const char *oid) {
  auto *s = find_sensor(oid);
  out += "\""; out += key; out += "\":";
  if (s == nullptr || !s->has_state() || isnan(s->state)) {
    out += "null";
  } else {
    char buf[24];
    snprintf(buf, sizeof(buf), "%.3f", s->state);
    out += buf;
  }
  out += ",";
}

static void append_text(std::string &out, const char *key, const char *oid) {
  auto *s = find_text_sensor(oid);
  out += "\""; out += key; out += "\":";
  if (s == nullptr || !s->has_state()) {
    out += "null";
  } else {
    out += "\"";
    for (char c : s->state) {
      if (c == '"' || c == '\\') out += '\\';
      out += c;
    }
    out += "\"";
  }
  out += ",";
}

static void append_binary(std::string &out, const char *key, const char *oid) {
  auto *s = find_binary_sensor(oid);
  out += "\""; out += key; out += "\":";
  if (s == nullptr || !s->has_state()) {
    out += "null";
  } else {
    out += (s->state ? "true" : "false");
  }
  out += ",";
}

// ─────────────────────────────────────────────────────────────────
// Dashboard HTML
// ─────────────────────────────────────────────────────────────────
static const char NUAQI_DASH_HTML[] = R"ESPH(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0">
<title>NU-AQI-3</title>
<style>
:root{
  --bg:#0e1117;--surf:#161b25;--surf2:#1c2233;
  --border:rgba(255,255,255,0.07);--border2:rgba(255,255,255,0.13);
  --text:#dde2ee;--muted:#6b7585;
  --green:#68d391;--yellow:#f0c060;--orange:#f0a050;--red:#e87c6e;
  --purple:rgb(180,110,190);--blue:#74b9e8;--blue-raw:rgb(116,185,232);
  --mono:'JetBrains Mono','Consolas',monospace;
  --sans:'IBM Plex Sans',ui-sans-serif,system-ui,sans-serif
}
*{box-sizing:border-box;margin:0;padding:0}
html{font-size:14px}
body{background:var(--bg);color:var(--text);font-family:var(--sans);min-height:100vh;-webkit-font-smoothing:antialiased}

/* ── Topbar ── */
.topbar{display:flex;align-items:center;justify-content:space-between;padding:11px 16px;background:var(--surf);border-bottom:1px solid var(--border);position:sticky;top:0;z-index:50}
.tb-title{font-family:var(--mono);font-size:12px;font-weight:700;letter-spacing:0.14em;text-transform:uppercase;color:var(--text)}
.tb-right{display:flex;align-items:center;gap:8px}
.dot{width:7px;height:7px;border-radius:50%;background:var(--muted);transition:background 0.4s,box-shadow 0.4s;flex-shrink:0}
.dot.ok{background:var(--green);box-shadow:0 0 7px var(--green)}
.dot.err{background:var(--red);box-shadow:0 0 7px var(--red)}
.dot.spin{background:var(--yellow);animation:blink 1.1s infinite}
.dot.warm{background:var(--blue);box-shadow:0 0 7px var(--blue);animation:blink 1.4s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:0.25}}
.tb-age{font-family:var(--mono);font-size:10px;color:var(--muted)}
.tb-badge{font-family:var(--mono);font-size:10px;font-weight:700;letter-spacing:0.06em;padding:2px 9px;border-radius:20px;border-width:1px;border-style:solid;transition:all 0.5s}

/* ── Layout ── */
.page{max-width:860px;margin:0 auto;padding:16px;display:flex;flex-direction:column;gap:12px}
.card{background:var(--surf);border:1px solid var(--border);border-radius:12px;overflow:hidden}
.card-bar{height:4px;transition:background 0.6s}
.card-body{padding:14px 16px}
.two-col{display:grid;grid-template-columns:1fr 1fr;gap:12px}

/* ── Hero ── */
.hero-row{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:12px}
.hero-main{display:flex;align-items:flex-end;gap:7px}
.hero-num{font-family:var(--mono);font-size:56px;font-weight:800;line-height:1;letter-spacing:-2px;transition:color 0.5s}
.hero-unit{font-family:var(--mono);font-size:19px;color:var(--muted);padding-bottom:9px}
.hero-sub{font-size:11px;color:var(--muted);margin-top:3px}
.hero-aside{text-align:right;flex-shrink:0;padding-top:2px}
.ha-label{font-family:var(--mono);font-size:9px;font-weight:700;letter-spacing:0.1em;text-transform:uppercase;color:var(--muted);margin-bottom:3px}
.ha-num{font-family:var(--mono);font-size:32px;font-weight:700;line-height:1.05;transition:color 0.5s}
.ha-unit{font-size:11px;color:var(--muted);margin-top:2px}

/* ── Tiles ── */
.tiles{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:10px}
.tile{padding:8px 9px;border-radius:8px;border-width:1px;border-style:solid;transition:background 0.5s,border-color 0.5s}
.tile-lbl{font-family:var(--mono);font-size:9px;font-weight:700;letter-spacing:0.08em;text-transform:uppercase;margin-bottom:3px}
.tile-val{font-family:var(--mono);font-size:15px;font-weight:800}
.tile-sub{font-size:9px;font-weight:600;margin-top:2px;opacity:0.85}

/* ── Banner ── */
.vbanner{display:flex;align-items:center;padding:7px 12px 7px 0;margin-bottom:8px;border-radius:8px;border-width:1px;border-style:solid;overflow:hidden;transition:background 0.5s,border-color 0.5s}
.vb-bar{width:3px;align-self:stretch;border-radius:8px 0 0 8px;margin-right:12px;flex-shrink:0;transition:background 0.5s}
.vb-icon{font-size:13px;flex-shrink:0;margin-right:8px}
.vb-msg{font-size:12px;font-weight:600;color:var(--text);line-height:1.4}

/* ── Sensor rows ── */
.col-hdr{display:flex;padding:0 10px 5px 13px;border-bottom:1px solid var(--border);margin-bottom:4px}
.ch{font-family:var(--mono);font-size:9px;color:var(--muted);letter-spacing:0.08em;text-transform:uppercase}
.ch.f1{flex:1}
.srow{display:flex;align-items:center;padding:7px 10px 7px 0;margin:2px 0;border-radius:8px;border-width:1px;border-style:solid;overflow:hidden;transition:background 0.4s,border-color 0.4s}
.srow-bar{width:3px;align-self:stretch;border-radius:8px 0 0 8px;margin-right:10px;flex-shrink:0;transition:background 0.4s,opacity 0.4s}
.srow-lbl{flex:1;font-size:12px}
.srow-val{width:68px;text-align:right;font-family:var(--mono);font-size:13px;font-weight:600}
.srow-delta{width:60px;text-align:right;font-family:var(--mono);font-size:11px;color:var(--muted)}
.srow-trend{width:100px;text-align:right;font-family:var(--mono);font-size:10px}
.srow-age{width:36px;text-align:right;font-family:var(--mono);font-size:10px;color:var(--muted)}

/* ── Environment / Device cards ── */
.sec-lbl{font-family:var(--mono);font-size:9px;font-weight:700;letter-spacing:0.12em;text-transform:uppercase;color:var(--muted);margin-bottom:10px}
.mgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:6px}
.mcell{background:var(--surf2);border:1px solid var(--border);border-radius:8px;padding:10px 12px}
.mc-lbl{font-family:var(--mono);font-size:9px;font-weight:700;letter-spacing:0.09em;text-transform:uppercase;color:var(--muted);margin-bottom:4px}
.mc-val{font-family:var(--mono);font-size:22px;font-weight:800;color:var(--text)}
.mc-unit{font-family:var(--mono);font-size:11px;color:var(--muted);margin-left:2px}
.mc-sub{font-size:11px;color:var(--muted);margin-top:3px;line-height:1.3}
.drow{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid var(--border)}
.drow:last-child{border-bottom:none}
.dr-lbl{font-size:12px;color:var(--muted)}
.dr-val{font-family:var(--mono);font-size:12px;font-weight:600;color:var(--text)}

/* ── Warmup / Error screens ── */
.fullcard{display:flex;flex-direction:column;align-items:center;justify-content:center;padding:60px 20px;gap:14px;text-align:center}
.wu-ring{width:52px;height:52px;border-radius:50%;border:3px solid var(--border2);border-top-color:var(--blue);animation:spin 1s linear infinite}
@keyframes spin{to{transform:rotate(360deg)}}
.wu-title{font-family:var(--mono);font-size:13px;font-weight:700;letter-spacing:0.1em;text-transform:uppercase}
.wu-sub{font-size:12px;color:var(--muted);max-width:320px;line-height:1.5}
.wu-bar{width:100%;max-width:240px;height:3px;border-radius:2px;background:var(--border2);overflow:hidden;margin-top:4px}
.wu-bar-fill{height:100%;background:var(--blue);border-radius:2px;animation:warmfill 3s ease-in-out infinite}
@keyframes warmfill{0%{width:0%}60%{width:80%}100%{width:100%}}

/* ── Responsive ── */
@media(max-width:600px){
  .hero-num{font-size:44px}
  .tiles{grid-template-columns:repeat(2,1fr)}
  .two-col{grid-template-columns:1fr}
}
</style>
</head>
<body>
<div class="topbar">
  <div class="tb-title">NU-AQI-3</div>
  <div class="tb-right">
    <span class="tb-badge" id="tb-badge">—</span>
    <div class="dot spin" id="dot"></div>
    <div class="tb-age" id="tb-age">connecting…</div>
  </div>
</div>
<div class="page" id="page">
  <div class="card">
    <div class="card-bar" style="background:var(--blue);"></div>
    <div class="card-body">
      <div class="fullcard">
        <div class="wu-ring"></div>
        <div class="wu-title" style="color:var(--blue);">Connecting…</div>
        <div class="wu-sub">Polling device, please wait.</div>
      </div>
    </div>
  </div>
</div>
<script>
'use strict';
const REFRESH_MS = 8000;
let D = {}, firstRender = true, errCount = 0;

// ── Data helpers ──────────────────────────────────────────────────
const N = k => { const v = D[k]; return (v == null || isNaN(v)) ? NaN : Number(v); };
const T = k => D[k] == null ? '—' : String(D[k]);
const isReady = () => D.ready === true || !isNaN(N('co2'));

// ── Unified 5-tier palette — exact match to Room AQI HA card ─────
// Tier: 0=Excellent  1=Good  2=Monitor  3=Circulate  4=Fresh air
// Warming-up uses a separate blue scheme (not a tier).
const RAW   = ['rgb(0,228,0)','rgb(255,255,0)','rgb(255,126,0)','rgb(255,0,0)','rgb(143,63,151)'];
const TEXT  = ['#68d391','#f0c060','#f0a050','#e87c6e','rgb(180,110,190)'];
const BBG   = ['rgba(39,174,96,0.18)','rgba(184,134,11,0.14)','rgba(184,134,11,0.18)','rgba(192,57,43,0.18)','rgba(143,63,151,0.18)'];
const BBR   = ['rgba(39,174,96,0.35)','rgba(184,134,11,0.28)','rgba(184,134,11,0.35)','rgba(192,57,43,0.35)','rgba(143,63,151,0.35)'];
const TLBL  = ['Excellent','Good','Monitor','Circulate','Fresh Air'];

// Blue warming-up palette — matches Room AQI card's falling/unavailable blue
const WARM_TEXT   = '#74b9e8';
const WARM_RAW    = 'rgb(116,185,232)';
const WARM_BG     = 'rgba(26,107,174,0.18)';
const WARM_BR     = 'rgba(26,107,174,0.35)';
const WARM_DIMMED = 'rgba(26,107,174,0.08)';
const WARM_DIMBD  = 'rgba(26,107,174,0.20)';

const rawColor  = t => RAW[t]  ?? 'rgba(255,255,255,0.15)';
const textColor = t => TEXT[t] ?? '#6b7585';
const badgeBg   = t => BBG[t]  ?? 'rgba(158,158,158,0.14)';
const badgeBr   = t => BBR[t]  ?? 'rgba(158,158,158,0.24)';
const tierLabel = t => TLBL[t] ?? '—';
const ra = (raw, a) => raw.replace(/^rgb\(/, 'rgba(').replace(')', `,${a})`);

// ── Tier mapping ──────────────────────────────────────────────────
// CO₂ aligned to standard ventilation guidelines:
//   Tier 0 Excellent: < 600 ppm
//   Tier 1 Good:      600–800 ppm
//   Tier 2 Monitor:   800–1000 ppm  (ASHRAE upper bound, slight stuffiness)
//   Tier 3 Circulate: 1000–1400 ppm (noticeable fatigue, recirculate air)
//   Tier 4 Fresh air: ≥ 1400 ppm    (significant impairment, open window/ERV)
const valTier = (type, v) => {
  if (isNaN(v)) return -1;
  if (type === 'co2')  return v >= 1400 ? 4 : v >= 1000 ? 3 : v >= 800 ? 2 : v >= 600 ? 1 : 0;
  if (type === 'voc')  return v > 450   ? 4 : v > 350   ? 3 : v > 250  ? 2 : v > 150  ? 1 : 0;
  if (type === 'nox')  return v >= 50   ? 4 : v >= 20   ? 3 : v >= 10  ? 2 : v > 5    ? 1 : 0;
  if (type === 'pm25') return v >= 150  ? 4 : v >= 100  ? 3 : v >= 50  ? 2 : v >= 25  ? 1 : 0;
  if (type === 'pm1')  return v >= 35   ? 4 : v >= 20   ? 3 : v >= 5   ? 2 : 0;
  if (type === 'pm4')  return v >= 35   ? 4 : v >= 20   ? 3 : v >= 5   ? 2 : 0;
  if (type === 'pm10') return v >= 150  ? 4 : v >= 75   ? 3 : v >= 20  ? 2 : 0;
  return -1;
};

const tc     = (type, v) => textColor(valTier(type, v));
const co2C   = v => tc('co2',  v);
const vocC   = v => tc('voc',  v);
const noxC   = v => tc('nox',  v);
const pm25C  = v => tc('pm25', v);
const pm1C   = v => tc('pm1',  v);
const pm4C   = v => tc('pm4',  v);
const pm10C  = v => tc('pm10', v);

const aqiLvl = (co2, voc, nox, pm) => Math.max(
  isNaN(co2) ? 0 : valTier('co2',  co2),
  isNaN(voc) ? 0 : valTier('voc',  voc),
  isNaN(nox) ? 0 : valTier('nox',  nox),
  isNaN(pm)  ? 0 : valTier('pm25', pm)
);

// ── Formatting ────────────────────────────────────────────────────
const fv = (v, d = null) => {
  if (isNaN(v)) return '—';
  return d != null ? v.toFixed(d) : Number.isInteger(v) ? v.toLocaleString() : v.toFixed(1);
};
const fmtDelta = d => {
  if (isNaN(d)) return '—';
  const n = Math.abs(d) < 10 ? d.toFixed(1) : Math.round(d).toString();
  return (d > 0 ? '+' : '') + n;
};

// ── Sensor history (delta / trend / age) ─────────────────────────
const HIST = {};
function track(key, v) {
  if (isNaN(v)) return;
  if (!HIST[key]) HIST[key] = { prev: NaN, t: 0, samples: [] };
  const h = HIST[key], now = Date.now();
  if (isNaN(h.prev) || Math.abs(v - h.prev) >= 0.05) {
    if (!isNaN(h.prev)) h.samples.push({ v: h.prev, t: h.t });
    if (h.samples.length > 6) h.samples.shift();
    h.prev = v; h.t = now;
  }
}
const getDelta = (key, v) => {
  const h = HIST[key];
  if (!h || isNaN(h.prev) || !h.samples.length) return NaN;
  return v - h.samples[0].v;
};
function getTrend(key, v) {
  const h = HIST[key];
  if (!h || h.samples.length < 2) return { label: 'Stable', color: '#6b7585', rising: false, falling: false };
  const d = getDelta(key, v);
  if (isNaN(d) || Math.abs(d) < 0.5) return { label: 'Stable', color: '#6b7585', rising: false, falling: false };
  const fast = Math.abs(d) > 5;
  if (d > 0) return fast
    ? { label: '↑↑ Rising fast', color: 'rgb(180,110,190)', rising: true,  falling: false }
    : { label: '↑ Rising',       color: '#e87c6e',          rising: true,  falling: false };
  return fast
    ? { label: '↓↓ Falling fast', color: '#74b9e8', rising: false, falling: true }
    : { label: '↓ Falling',       color: '#a0c8e8', rising: false, falling: true };
}
function getAge(key) {
  const h = HIST[key];
  if (!h || !h.t) return '—';
  const s = Math.floor((Date.now() - h.t) / 1000);
  if (s < 60)   return s + 's';
  if (s < 3600) return Math.floor(s / 60) + 'm';
  return Math.floor(s / 3600) + 'h';
}
const ageSpan = key => {
  const a = getAge(key);
  return a === '—' ? '' : ` <span style="font-size:9px;color:var(--muted);margin-left:3px;">${a}</span>`;
};

// ── UI components ─────────────────────────────────────────────────

// Tile: warm=true renders blue warming-up placeholder
function tile(label, value, t, warm = false) {
  const c  = warm ? WARM_TEXT  : textColor(t);
  const bg = warm ? WARM_BG    : badgeBg(t);
  const br = warm ? WARM_BR    : badgeBr(t);
  const sub = warm ? 'Warming…' : tierLabel(t);
  return `<div class="tile" style="background:${bg};border-color:${br};">
<div class="tile-lbl" style="color:${c};">${label}</div>
<div class="tile-val" style="color:${c};">${value}</div>
<div class="tile-sub" style="color:${c};">${sub}</div>
</div>`;
}

// Ventilation banner
// lv is the precomputed worst tier so it matches the top bar exactly.
// Banner messages distinguish recirculate (tier 3) from fresh air (tier 4),
// and handle tier 1+rising so a trending-up good room isn't falsely green.
function vbanner(lv, co2, voc, nox, pm25) {
  const co2Trend = getTrend('co2', co2);
  const co2Rising = co2Trend.rising;
  const co2Fast   = co2Trend.label.includes('fast');
  const co2Tier   = valTier('co2', co2);

  // Threshold booleans — named to match standard action levels
  const c2fresh  = !isNaN(co2) && co2 >= 1400;  // tier 4: bring in outdoor air
  const c2circ   = !isNaN(co2) && co2 >= 1000;  // tier 3: recirculate
  const c2mon    = !isNaN(co2) && co2 >= 800;   // tier 2: monitor
  const c2good   = !isNaN(co2) && co2 >= 600;   // tier 1: good
  const vocCrit  = !isNaN(voc)  && voc  > 450;
  const vocHigh  = !isNaN(voc)  && voc  > 350;
  const vocMod   = !isNaN(voc)  && voc  > 250;
  const noxCrit  = !isNaN(nox)  && nox  >= 50;
  const noxHigh  = !isNaN(nox)  && nox  >= 20;
  const noxMod   = !isNaN(nox)  && nox  >= 10;
  const pmCrit   = !isNaN(pm25) && pm25 >= 35;
  const pmHigh   = !isNaN(pm25) && pm25 >= 12;

  let ic, msg;
  if (lv >= 4) {
    ic = '⚡';
    msg = 'Bring in fresh air now — ' +
      [c2fresh&&'CO₂ ≥1400 ppm', vocCrit&&'VOC >450', noxCrit&&'NOx ≥50', pmCrit&&'PM2.5 ≥35']
        .filter(Boolean).join(', ');
  } else if (lv >= 3) {
    ic = '↺';
    msg = 'Recirculate air — ' +
      [c2circ&&'CO₂ 1000–1400 ppm', vocHigh&&'VOC >350', noxHigh&&'NOx elevated', pmHigh&&'PM2.5 ≥12']
        .filter(Boolean).join(', ');
  } else if (lv >= 2 && co2Rising) {
    ic = '⚠';
    msg = co2Fast
      ? 'CO₂ 800–1000 ppm and rising fast — prepare to circulate'
      : 'CO₂ 800–1000 ppm and rising — monitor closely';
  } else if (lv >= 2) {
    ic = '~';
    msg = 'CO₂ 800–1000 ppm — no action needed yet';
  } else if (co2Tier >= 1 && co2Rising) {
    // tier 1 (600–800) but trending upward — show yellow warning not false green
    ic = '~';
    msg = co2Fast
      ? 'CO₂ good but rising fast — watch closely'
      : 'CO₂ good but rising — trending up';
  } else if (lv >= 1) {
    ic = '~';
    msg = 'CO₂ good — air quality acceptable';
  } else {
    ic = '✓';
    msg = 'Air quality good — no ventilation needed';
  }

  // For tier 1+rising use tier 1 yellow styling even though lv may be 0
  const styleTier = (co2Tier >= 1 && co2Rising && lv < 2) ? 1 : lv;
  const bg = ra(rawColor(styleTier), styleTier >= 3 ? 0.14 : styleTier >= 2 ? 0.12 : 0.10);
  const br = ra(rawColor(styleTier), styleTier >= 3 ? 0.35 : styleTier >= 2 ? 0.30 : 0.28);
  const ac = rawColor(styleTier);

  return `<div class="vbanner" style="background:${bg};border-color:${br};">
<div class="vb-bar" style="background:${ac};"></div>
<span class="vb-icon">${ic}</span>
<span class="vb-msg">${msg}</span>
</div>`;
}

// Blue warming-up banner
const warmBanner = `<div class="vbanner" style="background:${WARM_DIMMED};border-color:${WARM_DIMBD};">
<div class="vb-bar" style="background:${WARM_RAW};"></div>
<span class="vb-icon">⏳</span>
<span class="vb-msg">Sensor is warming up — please wait, readings will appear shortly</span>
</div>`;

// Sensor row
function srow(key, label, v, type) {
  track(key, v);
  const t   = valTier(type, v);
  const vc  = textColor(t);
  const raw = rawColor(t);
  const d   = getDelta(key, v);
  const tr  = getTrend(key, v);
  const age = getAge(key);
  const dc  = isNaN(d) ? '#6b7585' : d > 0 ? '#e87c6e' : d < 0 ? '#74b9e8' : '#6b7585';
  const rowBg = tr.rising  ? ra(rawColor(3), 0.06)
    : tr.falling ? 'rgba(26,107,174,0.06)'
    : t >= 4     ? ra(rawColor(4), 0.07)
    : t >= 3     ? ra(rawColor(3), 0.06)
    :              'rgba(255,255,255,0.02)';
  const rowBd = tr.rising  ? ra(rawColor(3), 0.20)
    : tr.falling ? 'rgba(26,107,174,0.20)'
    : t >= 4     ? ra(rawColor(4), 0.20)
    : t >= 3     ? ra(rawColor(3), 0.16)
    :              'rgba(255,255,255,0.05)';
  const accent = tr.rising || tr.falling ? tr.color : t >= 0 ? raw : 'transparent';
  const acOp   = tr.rising || tr.falling ? '0.7' : t >= 0 ? '0.5' : '0.15';
  const bold   = tr.rising || tr.falling;
  return `<div class="srow" style="background:${rowBg};border-color:${rowBd};">
<div class="srow-bar" style="background:${accent};opacity:${acOp};"></div>
<div class="srow-lbl" style="color:${bold?'var(--text)':'#6b7585'};font-weight:${bold?600:400};">${label}</div>
<div class="srow-delta" style="color:${dc};">${fmtDelta(d)}</div>
<div class="srow-trend" style="color:${tr.color};">${tr.label}</div>
<div class="srow-age">${age}</div>
<div class="srow-val" style="color:${vc};font-weight:${bold?700:500};">${fv(v)}</div>
</div>`;
}

const colHdr = `<div class="col-hdr">
<div class="ch f1">Sensor</div>
<div class="ch" style="width:60px;text-align:right;">Delta</div>
<div class="ch" style="width:100px;text-align:right;">Trend</div>
<div class="ch" style="width:36px;text-align:right;">Age</div>
<div class="ch" style="width:68px;text-align:right;">Value</div>
</div>`;

const sw = str => (!str || str === '—' || str === 'Warming up') ? '' : str.split('(')[0].trim();

// ── Poll ──────────────────────────────────────────────────────────
function setDot(cls) { document.getElementById('dot').className = 'dot ' + cls; }

async function poll() {
  setDot('spin');
  try {
    const r = await fetch('/data.json', { signal: AbortSignal.timeout(4000), cache: 'no-store' });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    D = await r.json();
    errCount = 0;
    document.getElementById('tb-age').textContent = new Date().toLocaleTimeString();
    render();
  } catch (e) {
    errCount++;
    setDot('err');
    document.getElementById('tb-age').textContent = 'error · ' + errCount;
    if (firstRender) showError(e.message);
  }
}

// ── Render ────────────────────────────────────────────────────────
function render() {
  const co2   = N('co2'),   voc  = N('voc'),   nox  = N('nox');
  const pm1   = N('pm1'),   pm25 = N('pm25'),  pm4  = N('pm4'),  pm10 = N('pm10');
  const temp  = N('temp'),  humid = N('humid'), absH = N('absHumid'), fl = N('feelsLike');
  const pmAqi = N('pmAqi'), wQ   = N('wifiQ'), espT = N('espTemp');

  track('temp', temp); track('humid', humid); track('fl', fl);
  track('absH', absH); track('wQ', wQ);       track('espT', espT);

  const ready = isReady();
  const lv    = Math.min(aqiLvl(co2, voc, nox, pmAqi), 4);

  // Colors — warming state is blue throughout
  const barC  = ready ? rawColor(lv)   : WARM_RAW;
  const badC  = ready ? textColor(lv)  : WARM_TEXT;
  const badBg = ready ? badgeBg(lv)    : WARM_BG;
  const badBr = ready ? badgeBr(lv)    : WARM_BR;
  const heroC = ready ? co2C(co2)      : WARM_TEXT;
  const vocHC = ready ? vocC(voc)      : WARM_TEXT;
  const ll    = ready ? tierLabel(lv)  : 'Warming Up';

  // Badge
  const badge = document.getElementById('tb-badge');
  badge.textContent = ll;
  badge.style.cssText = `background:${badBg};border-color:${badBr};color:${badC};font-family:var(--mono);font-size:10px;font-weight:700;letter-spacing:0.06em;padding:2px 9px;border-radius:20px;border-width:1px;border-style:solid;`;
  setDot(ready ? 'ok' : 'warm');

  // Display values
  const co2d = !ready ? '—' : isNaN(co2)   ? '—' : Math.round(co2).toLocaleString();
  const vocd = !ready ? '—' : isNaN(voc)   ? '—' : Math.round(voc).toLocaleString();
  const noxd = isNaN(nox)   ? '—' : Math.round(nox).toString();
  const pmd  = isNaN(pmAqi) ? '—' : Math.round(pmAqi).toString();
  const ahw  = T('ahTxt').split('(')[0].trim();

  // Tier values for tiles
  const c2t = valTier('co2',  co2);
  const vt  = valTier('voc',  voc);
  const nt  = valTier('nox',  nox);
  const pt  = valTier('pm25', pmAqi);

  // Warming tile — blue placeholder
  const wt = `<div class="tile" style="background:${WARM_BG};border-color:${WARM_BR};">
<div class="tile-lbl" style="color:${WARM_TEXT};">—</div>
<div class="tile-val" style="color:${WARM_TEXT};">—</div>
<div class="tile-sub" style="color:${WARM_TEXT};">Warming…</div>
</div>`;

  document.getElementById('page').innerHTML = `
<div class="card">
<div class="card-bar" style="background:${barC};"></div>
<div class="card-body">
<div class="sec-lbl">CO₂</div>
<div class="hero-row">
  <div>
    <div class="hero-main">
      <div class="hero-num" style="color:${heroC};">${co2d}</div>
      <div class="hero-unit">ppm</div>
    </div>
    <div class="hero-sub">${ready ? 'circ ≥1000 ppm · fresh air ≥1400 ppm' : 'Sensor warming up — readings will appear shortly'}</div>
  </div>
  <div class="hero-aside">
    <div class="ha-label">VOC Index</div>
    <div class="ha-num" style="color:${vocHC};">${vocd}</div>
    <div class="ha-unit">index</div>
  </div>
</div>
<div class="tiles">
${ready
  ? tile('CO₂',co2d,c2t)+tile('VOC',vocd,vt)+tile('NOx',noxd,nt)+tile('PM AQI',pmd,pt)
  : wt+wt+wt+wt}
</div>
${ready ? vbanner(lv,co2,voc,nox,pm25) : warmBanner}
${colHdr}
${srow('co2',  'CO₂',      co2,   'co2')}
${srow('voc',  'VOC Index', voc,   'voc')}
${srow('nox',  'NOx Index', nox,   'nox')}
${srow('pm25', 'PM 2.5',    pm25,  'pm25')}
${srow('pm1',  'PM 1.0',    pm1,   'pm1')}
${srow('pm4',  'PM 4.0',    pm4,   'pm4')}
${srow('pm10', 'PM 10',     pm10,  'pm10')}
${srow('pmAqi','PM AQI',    pmAqi, 'pm25')}
</div></div>

<div class="two-col">
<div class="card">
<div class="card-bar" style="background:#74b9e8;"></div>
<div class="card-body">
<div class="sec-lbl">Environment</div>
<div class="mgrid">
<div class="mcell"><div class="mc-lbl">Temperature</div><div><span class="mc-val">${fv(isNaN(temp)?NaN:(temp*9/5+32),1)}</span><span class="mc-unit">°F</span>${ageSpan('temp')}</div></div>
<div class="mcell"><div class="mc-lbl">Humidity</div><div><span class="mc-val">${fv(humid,1)}</span><span class="mc-unit">%</span>${ageSpan('humid')}</div></div>
<div class="mcell"><div class="mc-lbl">Feels Like</div><div><span class="mc-val">${fv(fl,1)}</span><span class="mc-unit">°F</span>${ageSpan('fl')}</div></div>
<div class="mcell"><div class="mc-lbl">Absolute Humidity</div><div><span class="mc-val">${fv(absH,2)}</span><span class="mc-unit">g/m³</span>${ageSpan('absH')}</div>${ahw?`<div class="mc-sub">${ahw}</div>`:''}</div>
</div>
</div></div>

<div class="card">
<div class="card-bar" style="background:#6b7585;"></div>
<div class="card-body">
<div class="sec-lbl">Device</div>
${[
  ['Sensor Ready',  ready ? 'Yes' : 'Warming up…', ''],
  ['Combined AQI',  sw(T('combinedTxt')) || ll,     ''],
  ['WiFi Quality',  isNaN(wQ)   ? '—' : Math.round(wQ)   + '%',   ageSpan('wQ')],
  ['ESP32 Temp',    isNaN(espT) ? '—' : espT.toFixed(1)   + '°F',  ageSpan('espT')],
].map(([l,v,a]) => `<div class="drow"><div class="dr-lbl">${l}</div><div class="dr-val">${v}${a}</div></div>`).join('')}
<div style="margin-top:10px;padding-top:8px;border-top:1px solid var(--border);">
<div class="sec-lbl" style="margin-bottom:6px;">Text Sensors</div>
${[['CO₂',T('co2Txt')],['VOC',T('vocTxt')],['NOx',T('noxTxt')],['PM',T('pmTxt')],['AH',T('ahTxt')]].map(([l,v])=>`<div class="drow"><div class="dr-lbl">${l}</div><div class="dr-val" style="font-size:10px;max-width:160px;text-align:right;line-height:1.3;">${v}</div></div>`).join('')}
</div>
</div></div>
</div>`;

  firstRender = false;
}

function showError(msg) {
  if (!firstRender) return;
  document.getElementById('page').innerHTML = `<div class="card">
<div class="card-bar" style="background:var(--red);"></div>
<div class="card-body"><div class="fullcard">
<div style="font-size:36px;color:var(--red);">✕</div>
<div class="wu-title" style="color:var(--red);">Connection Lost</div>
<div class="wu-sub">${msg}<br><br>Check that the device is powered and on the network.</div>
</div></div></div>`;
}

poll();
setInterval(poll, REFRESH_MS);
</script>
</body>
</html>
)ESPH";

// ─────────────────────────────────────────────────────────────────
//  C++ HTTP server — port 8080
// ─────────────────────────────────────────────────────────────────

static esp_err_t nuaqi_html_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  const char  *p         = NUAQI_DASH_HTML;
  size_t       remaining = sizeof(NUAQI_DASH_HTML) - 1;
  const size_t CHUNK     = 4096;
  while (remaining > 0) {
    size_t to_send = remaining < CHUNK ? remaining : CHUNK;
    if (httpd_resp_send_chunk(req, p, (ssize_t)to_send) != ESP_OK) {
      httpd_resp_send_chunk(req, nullptr, 0);
      return ESP_FAIL;
    }
    p += to_send; remaining -= to_send;
  }
  return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t nuaqi_healthz_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, "OK");
}

static esp_err_t nuaqi_data_handler(httpd_req_t *req) {
  std::string out;
  out.reserve(1024);
  out += "{";

  append_sensor(out, "co2",       "co2");
  append_sensor(out, "voc",       "voc");
  append_sensor(out, "nox",       "nox");
  append_sensor(out, "pm1",       "pm__1__m");
  append_sensor(out, "pm25",      "pm__2_5__m");
  append_sensor(out, "pm4",       "pm__4__m");
  append_sensor(out, "pm10",      "pm__10__m");
  append_sensor(out, "temp",      "temperature");
  append_sensor(out, "humid",     "humidity");
  append_sensor(out, "absHumid",  "absolute_humidity");
  append_sensor(out, "feelsLike", "aqi_feels_like_temp");
  append_sensor(out, "pmAqi",     "average_pm_aqi");
  append_sensor(out, "wifiQ",     "wifi_signal_quality");
  append_sensor(out, "espTemp",   "esp32_internal_temperature__f_");

  append_text(out, "co2Txt",      "aqi_co2");
  append_text(out, "vocTxt",      "aqi_tvoc");
  append_text(out, "noxTxt",      "aqi_nox");
  append_text(out, "pmTxt",       "pm_aqi");
  append_text(out, "ahTxt",       "aqi_ah");
  append_text(out, "combinedTxt", "aqi_combined_level");

  append_binary(out, "ready", "nu_aqi_3_sensor_ready");

  if (!out.empty() && out.back() == ',') out.pop_back();
  out += "}";

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, out.c_str(), out.size());
}

void nuaqi_dash_register() {
  static bool registered = false;
  if (registered) return;

  static bool dumped = false;
  if (!dumped) {
    dumped = true;
    ESP_LOGI("nuaqi_dash", "--- Registered sensors ---");
    for (auto *s : esphome::App.get_sensors())
      ESP_LOGI("nuaqi_dash", "  sensor:        name='%s' sanitized='%s'",
               s->get_name().c_str(), sanitize_id(s->get_name()).c_str());
    for (auto *s : esphome::App.get_text_sensors())
      ESP_LOGI("nuaqi_dash", "  text_sensor:   name='%s' sanitized='%s'",
               s->get_name().c_str(), sanitize_id(s->get_name()).c_str());
    for (auto *s : esphome::App.get_binary_sensors())
      ESP_LOGI("nuaqi_dash", "  binary_sensor: name='%s' sanitized='%s'",
               s->get_name().c_str(), sanitize_id(s->get_name()).c_str());
    ESP_LOGI("nuaqi_dash", "--------------------------");
  }

  httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
  config.server_port      = 8080;
  config.ctrl_port        = 0;
  config.stack_size       = 10240;
  config.max_uri_handlers = 4;
  config.max_open_sockets = 3;
  config.lru_purge_enable = true;

  httpd_handle_t server = nullptr;
  esp_err_t err = httpd_start(&server, &config);
  if (err != ESP_OK) {
    ESP_LOGE("nuaqi_dash", "httpd_start failed: 0x%x (%s) — will retry", err, esp_err_to_name(err));
    return;
  }
  registered = true;

  static const httpd_uri_t uris[] = {
    { "/",          HTTP_GET, nuaqi_html_handler,    nullptr },
    { "/dash",      HTTP_GET, nuaqi_html_handler,    nullptr },
    { "/data.json", HTTP_GET, nuaqi_data_handler,    nullptr },
    { "/healthz",   HTTP_GET, nuaqi_healthz_handler, nullptr },
  };
  for (const auto &u : uris) httpd_register_uri_handler(server, &u);

  ESP_LOGI("nuaqi_dash", "Dashboard v3.3 running at http://<device-ip>:8080/");
}