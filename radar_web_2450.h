#pragma once
// ─────────────────────────────────────────────────
//  NU-Presence-24GHz-2  LD2450 tuning page  v1.15  (radar_web_2450.h)  2026-11-25
//  v1.15: "Air quality and HVAC" card for units that also measure air (Nu-MasterBath-1). The device yaml builds a
//         JSON object once a second in the main loop and hands it over with radar_web_aq(); data.json passes it on
//         as aq. Not called = no card (NU-Presence-24GHz-2).
//  v1.14: presence_2450.h v1.6 lock-on: trk state 3 = checking (not counted yet; grey dashed ring, "checking"), data.json
//         cl = still-clutter spots [x, y, times]; spots seen twice or more drawn as a grey cross "still object".
//  v1.13: activities card per device: radar_web_cfg().actt (title), .acts ("Label:min|Label:min|..."), .actd (defaults
//         "zh1,zh2,zh3,sth" in s). Not set = the bathroom card as before.
//  v1.12: Zones card moved directly under the Room map card.
//  v1.11: polygon zones for presence_2450.h v1.5 - "Shape" per zone (drag corners on the map, add / remove, 3-8 corners),
//         "Fit targets" (records target positions for N s and fits an octagon + 30 cm). /poly?z=N&v=x,y;x,y;.. (mm,
//         empty = rectangle) saved in flash; the zone rectangle is set to the bounding box (what the chip filters by).
//         data.json zp.
//  v1.10: "Bathroom activities" card for presence_2450.h v1.4 zone holds: per zone a Still hold box (minutes) and activity
//         buttons (hands, shaving, dressing, toilet, nails, shower, tub soak), "Bathroom defaults". Shown only when the yaml
//         sets radar_web_cfg().zh[] (and zname[]); /set keys zh1..zh3 (s); data.json zh + zname.
//  v1.9: for presence_2450.h v1.3 - rw_room_geom() hands the room plan to the tracker, "track only inside the room"
//        checkbox (layout key fence), "Fit room" per zone, held / leaving from the tracker's exit rule.
//  v1.8: room map redesigned like radar_web.h v1.17 - the room to scale (ft / m) with the sensor on a wall (presets or
//        drag), an aim knob, targets / trails / zones turned into the room, zones drawn square to the sensor, layout
//        saved on the device (/room?v=, /room?reset=1, default radar_web_cfg().room_default).
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
  esphome::number::Number *zh[3] = {};   // v1.10: Still hold per zone (s), set from the device yaml
  const char *zname[3] = {};             // v1.10: zone names for the Bathroom activities card
  const char *actt = nullptr, *acts = nullptr, *actd = nullptr;   // v1.13: activities card title / list / defaults
  esphome::binary_sensor::BinarySensor *mvraw = nullptr, *occraw = nullptr;
  esphome::switch_::Switch *multi = nullptr;
  esphome::select::Select *zt = nullptr;
  esphome::text_sensor::TextSensor *fw = nullptr;
};
static RadarWebCfg &radar_web_cfg() { static RadarWebCfg c; return c; }
static const char *rw_room_default = nullptr;   // v1.8: room map default JSON {"w":cm,"d":cm,"place":"bm","unit":"ft"} (set from yaml)

// v1.15: air quality + HVAC block for the page, built by the device yaml in the main loop (two buffers, index swap)
static char rw_aq[2][1100] = {"", ""};
static volatile int rw_aq_i = 0;
static void radar_web_aq(const char *js) { const int w = 1 - rw_aq_i; strlcpy(rw_aq[w], js, sizeof rw_aq[w]); rw_aq_i = w; }
static void rw_kn(std::string &o, const char *k, float v) {   // "k":number|null,
  char b[40]; if (std::isnan(v)) snprintf(b, sizeof b, "\"%s\":null,", k); else snprintf(b, sizeof b, "\"%s\":%g,", k, v); o += b; }
static void rw_ks(std::string &o, const char *k, const std::string &v) {   // "k":"escaped text",
  o += "\""; o += k; o += "\":\"";
  for (const char ch : v) { if (ch == '"' || ch == '\\') { o += '\\'; o += ch; } else if (ch == '\n') o += "\\n"; else if ((unsigned char) ch >= 0x20) o += ch; }
  o += "\","; }
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

<div id="aqWrap" style="display:none">
<h2>Air quality and HVAC</h2><div class="card"><div class="row">
<div><div class="big" id="aqLvl" style="font-size:26px">&mdash;</div><div class="sub">air quality</div></div>
<div><span class="pill" id="hvHead">HVAC</span> <span class="sub" id="hvSub"></span></div></div>
<div class="row" style="margin-top:10px;align-items:flex-start"><table style="flex:1 1 320px;width:auto"><tbody id="aqT"></tbody></table>
<div style="flex:1 1 240px"><div class="sub">HVAC for this room</div><div id="hvDet" style="white-space:pre-line;font-size:13px;margin-top:4px"></div></div></div>
</div></div>

<h2>Room map</h2><div class="card">
<svg id="map"></svg>
<div class="row" style="margin-top:8px">
<span class="sub">Draw:</span><button id="d0">Zone 1</button><button id="d1">Zone 2</button><button id="d2">Zone 3</button>
<label class="ck"><input type="checkbox" id="trail" checked>trails</label>
<label class="ck"><input type="checkbox" id="flip">mirror left / right</label>
<label class="ck"><input type="checkbox" id="r_fence" checked>track only inside the room</label>
</div>
<div class="row" id="pbar" style="margin-top:8px;display:none"><b id="pttl"></b><button id="psave">Save shape</button><button id="pcancel">Cancel</button><button id="prect">Back to rectangle</button>
<span class="sub">drag the corners &middot; drag a small dot to add a corner (up to 8) &middot; double-tap or long-press a corner to remove it</span></div>
<div class="row" id="cbar" style="margin-top:8px;display:none"><b id="cttl"></b><span class="sub" id="ctxt"></span><button id="cstop">Fit now</button><button id="ccancel">Cancel</button></div>
<div class="row" style="margin-top:8px">
<label><span id="lblW">Room width</span><input type="number" id="room_w" min="1" max="70" step="0.1"></label>
<label><span id="lblD">Room depth</span><input type="number" id="room_d" min="1" max="70" step="0.1"></label>
<label>Units<select id="r_unit"><option value="ft">feet</option><option value="m">metres</option></select></label>
<label>Sensor on<select id="r_place"><option value="tl">top-left corner</option><option value="tm">top wall, middle</option><option value="tr">top-right corner</option><option value="rm">right wall, middle</option><option value="br">bottom-right corner</option><option value="bm">bottom wall, middle</option><option value="bl">bottom-left corner</option><option value="lm">left wall, middle</option><option value="custom">where I dragged it</option></select></label>
<label>Aim &deg;<input type="number" id="r_aim" min="0" max="359" step="1" style="width:70px"></label>
<button id="r_center" type="button">Aim at room centre</button> <button id="r_reset" type="button">Reset layout</button>
<span id="roomTxt" class="sub" style="flex-basis:100%"></span>
</div>
<div class="help">Top-down plan of the room, drawn to scale. <b>Drag the white sensor dot</b> along the walls to where the unit hangs and <b>drag the small knob</b> to aim it (or pick a spot and type the aim: 0&deg; = up, clockwise). The fan is the radar's ~120&deg; / 6 m view: full strength inside the room, faint past the walls; targets, trails and zones are turned to match. To draw a zone pick Zone 1-3 and drag on the map: the LD2450 keeps zones as rectangles square to its own view, so the zone you get is the box square to the sensor around your two corners. For any other shape use <b>Shape</b> or <b>Fit targets</b> in Zones below. If someone walking to their right moves left on the map, tick mirror. <b>Track only inside the room</b>: targets more than 0.3 m outside the plan (hallway, stairs, through a wall) are ignored, and a person counts as gone only when they were last seen heading out at the edge of what the radar can see; lost in the open floor they are held as still. Saved on the device, so every browser and phone sees the same plan; <b>Reset layout</b> goes back to the layout built into this device.</div>
</div>

<h2>Zones</h2><div class="card">
<table><thead><tr><th>Zone</th><th>X1 m</th><th>Y1 m</th><th>X2 m</th><th>Y2 m</th><th></th></tr></thead><tbody id="zt"></tbody></table>
<div class="row" style="margin-top:10px">
<label>Zone type<select id="ztype"><option value="0">Disabled</option><option value="1">Detection</option><option value="2">Filter</option></select></label>
<label>Clear delay s<input type="number" id="timeout" min="0" max="3600" step="1"></label>
<label>Zone hold s<input type="number" id="hold" min="0" max="600" step="5"></label>
<label class="ck"><input type="checkbox" id="multi">multi target</label>
<label>Fit targets for<select id="capd"><option value="15">15 s</option><option value="30" selected>30 s</option><option value="60">60 s</option><option value="120">2 min</option></select></label>
</div>
<div class="help">Disabled: zones just report who is inside them. Detection: the radar only sees targets inside the zones. Filter: it ignores targets inside them, for a fan or a curtain. Multi target off tracks one person, which is steadier.<br><b>Shape</b> turns a zone into any shape with up to 8 corners: drag the corners on the map, then Save shape. <b>Fit targets</b> records where the radar sees people for the time set above (walk round the edge of the area, or sit where the zone should be) and fits a shape around it with 30 cm to spare. The radar chip itself only knows rectangles: Detection and Filter use the box round each shape (dashed on the map, and the X / Y numbers above); the shape decides who is in the zone and its still hold. Changing the X / Y numbers, Fit room or Clear turns the zone back into a rectangle.</div>
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
<b>Lock-on</b>: a new target only counts once it has <b>travelled half a metre</b> (people walk in); until then the map shows it grey, "checking".
Something that only twitches in place (a towel or curtain in a draught, a fan) never locks on and is dropped after 10 s; a spot that does that twice is
marked with a grey cross, "still object", and no longer trips Moving target or keeps a lost person alive. Spots are forgotten after an hour.<br>
<b>Still hold</b>: set it longer than the longest gap you see while sitting. 60 s suits a hallway; 300 s (default) a desk; 900 s a couch or TV; 1800 s a bed.
Longer never affects someone who walks out normally, it only sets how long Occupied can stay on if they slip out unseen.<br>
A target only counts once it has moved, so a grey dot on the map is a return that never moved: the radar reports it, Occupied ignores it. That also means
that after a power cut a seated person is not counted until they move; a restart or a flash keeps them. <b>Moving speed</b> and <b>Moving range</b> below decide what counts as moving, so they
affect Occupied too: a high Moving speed makes it slower to notice someone.<br>
<b>Ignore from / to</b>: targets this far from the sensor (the red band on the map) are not counted at all, so a reflection that sits there cannot
hold Occupied on. The office has one at about 4.4 m, so the default is 3.75 to 5.05 m. Anyone in the band is picked up again as soon as they step out of it.
Set both to 0 to turn it off.</div>
</div>

<div id="bathWrap" style="display:none">
<h2 id="bathTtl">Bathroom activities</h2><div class="card">
<div id="bathRows"></div>
<div class="row" style="margin-top:12px"><button id="bathDef">Defaults</button><span class="sub" id="bathDefTxt"></span></div>
<div class="help">Each zone gets its own <b>Still hold</b>: how long someone who goes still (or out of sight) inside that zone still counts as there.
Pick the <b>longest thing anyone does in that zone</b>; the buttons are typical times. The radar loses people who hold still (on a couch, in a tub)
and cannot see well through shower glass and spray, so where people sit, lie or soak needs the longest hold. Walking out through the door still clears
after <b>Clear delay</b> as before: a long hold only matters if someone slips out unseen. 0 = the zone uses the general Still hold above,
which also covers the floor outside the zones. Where zones overlap, the <b>smallest</b> zone you are in decides, so a closet drawn inside
a big main zone keeps its own hold.</div>
</div></div>

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

</div>

<script>
const $=id=>document.getElementById(id), R=6000, HALF=Math.PI/3, TC=['var(--t0)','var(--t1)','var(--t2)'], ZC=['var(--z0)','var(--z1)','var(--z2)'];
let D=null, busy=0, draw=-1, rub=null, geo=null, trails=[[],[],[]];
/* v1.8 room plan: cm, top-down, origin top-left, y down; sensor on a wall at (sx,sy), aim deg clockwise from up */
const FT=30.48, room={w:400,d:300,unit:'m',place:'bm',sx:200,sy:300,aim:0,aimAuto:true,flip:false,trail:true,fence:true};
let roomInit=false, lastDev=null, lastSent=null, pushT=0, rdrag=null, farCm=0;
const uf=()=>room.unit==='ft'?FT:100, ul=()=>room.unit==='ft'?'ft':'m', fmtU=cm=>(cm/uf()).toFixed(1)+' '+ul();
function aimCentre(){ const dx=room.w/2-room.sx, dy=room.d/2-room.sy; room.aim=Math.round(((Math.atan2(dx,-dy)*180/Math.PI)+360)%360); }
function placeAt(k){ const X={l:0,m:room.w/2,r:room.w}, Y={t:0,m:room.d/2,b:room.d};
  const mp={tl:['l','t'],tm:['m','t'],tr:['r','t'],rm:['r','m'],br:['r','b'],bm:['m','b'],bl:['l','b'],lm:['l','m']};
  if(!mp[k]) return; room.place=k; room.sx=X[mp[k][0]]; room.sy=Y[mp[k][1]]; room.aimAuto=true; aimCentre(); }
function roomUi(){ $('room_w').value=(room.w/uf()).toFixed(1); $('room_d').value=(room.d/uf()).toFixed(1); $('r_unit').value=room.unit; $('r_place').value=room.place; $('r_aim').value=Math.round(room.aim);
  $('lblW').textContent='Room width '+ul(); $('lblD').textContent='Room depth '+ul(); $('flip').checked=!!room.flip; $('trail').checked=room.trail!==false; $('r_fence').checked=room.fence!==false; }
function applyDev(r){ if(!r) return; if(r.sx!=null&&r.sy!=null) Object.assign(room,r); else { room.w=r.w||room.w; room.d=r.d||room.d; room.unit=r.unit||room.unit; if(r.flip!=null) room.flip=r.flip; placeAt(r.place||'bm'); } roomUi(); }
function saveRoom(){ roomUi(); map.last=null; if(D) map(); clearTimeout(pushT);
  pushT=setTimeout(()=>{ pushT=0; lastSent=JSON.stringify(room); fetch('/room?v='+encodeURIComponent(lastSent)).catch(()=>{}); },1000); }
function initRoom(dev){ if(roomInit) return; roomInit=true;
  if(dev){ lastDev=JSON.stringify(dev); applyDev(dev); return; }
  let o=null; try{ o=JSON.parse(localStorage.getItem('radar2')||'null'); }catch(e){}   /* old browser-only plan: carry it over once */
  if(o){ room.w=Math.round((o.w||4)*100); room.d=Math.round((o.d||3)*100); room.unit='m'; room.flip=!!o.flip; room.trail=o.trail!==false; placeAt(o.mount==='corner'?'bl':'bm'); saveRoom(); }
  else { placeAt('bm'); roomUi(); } }
function syncRoom(){ if(!D||!D.room||rdrag||pushT) return; const js=JSON.stringify(D.room); if(js===lastDev) return; lastDev=js; if(js!==lastSent){ applyDev(D.room); map.last=null; } }
$('room_w').addEventListener('change',()=>{ const v=+$('room_w').value; if(v>0){ const k=room.sx/room.w; room.w=Math.round(v*uf()); room.sx=Math.round(k*room.w); if(room.aimAuto) aimCentre(); } saveRoom(); });
$('room_d').addEventListener('change',()=>{ const v=+$('room_d').value; if(v>0){ const k=room.sy/room.d; room.d=Math.round(v*uf()); room.sy=Math.round(k*room.d); if(room.aimAuto) aimCentre(); } saveRoom(); });
$('r_unit').addEventListener('change',()=>{ room.unit=$('r_unit').value; saveRoom(); });
$('r_place').addEventListener('change',()=>{ placeAt($('r_place').value); saveRoom(); });
$('r_aim').addEventListener('change',()=>{ room.aim=((+$('r_aim').value%360)+360)%360; room.aimAuto=false; saveRoom(); });
$('r_center').onclick=()=>{ room.aimAuto=true; aimCentre(); saveRoom(); };
$('r_reset').onclick=()=>{ if(!confirm('Reset the room plan to the layout built into this device?')) return; clearTimeout(pushT); pushT=0; lastSent=null; lastDev=null; fetch('/room?reset=1').catch(()=>{}); };
$('flip').addEventListener('change',()=>{ room.flip=$('flip').checked; saveRoom(); });
$('r_fence').addEventListener('change',()=>{ room.fence=$('r_fence').checked; saveRoom(); });
/* v1.9: the room's bounding box in sensor mm (x right, y ahead) */
function roomBox(){ const th=room.aim*Math.PI/180, co=Math.cos(th), si=Math.sin(th), fl=room.flip?-1:1; let x0=1e9,x1=-1e9,y1=0;
  for(const [rx,ry] of [[0,0],[room.w,0],[room.w,room.d],[0,room.d]]){ const dx=(rx-room.sx)*10, dy=(ry-room.sy)*10, x=fl*(dx*co+dy*si), y=dx*si-dy*co; x0=Math.min(x0,x); x1=Math.max(x1,x); y1=Math.max(y1,y); }
  const r10=v=>Math.round(v/10)*10; return [Math.max(-6000,r10(x0)),0,Math.min(6000,r10(x1)),Math.min(6000,r10(y1))]; }
$('trail').addEventListener('change',()=>{ room.trail=$('trail').checked; saveRoom(); });
const setv=async(k,v)=>{busy++;try{await fetch('/set?k='+k+'&v='+encodeURIComponent(v),{signal:AbortSignal.timeout(4000)});}catch(e){}busy--;};
const setw=async(k,v)=>{busy++;try{await fetch('/sw?k='+k+'&v='+v,{signal:AbortSignal.timeout(4000)});}catch(e){}busy--;};
const m2=v=>v==null?'—':(v/1000).toFixed(2)+' m';
function fill(id,v,f){const el=$(id);if(document.activeElement===el||busy)return;if(el.type==='checkbox')el.checked=!!v;else el.value=v==null?'':(f?f(v):v);}
function setZone(i,z){fetch('/poly?z='+(i+1)+'&v=').catch(()=>{});z.forEach((v,j)=>setv('z'+(i+1)+'xyxy'[j]+(j<2?1:2),Math.round(v)));}
for(let i=0;i<3;i++){$('d'+i).addEventListener('click',()=>{draw=draw===i?-1:i;for(let j=0;j<3;j++)$('d'+j).classList.toggle('on',j===draw);});}
let tb='';for(let i=0;i<3;i++){tb+=`<tr><td><span class="dot" style="background:${ZC[i]}"></span>Zone ${i+1}</td>`;for(let j=0;j<4;j++)tb+=`<td><input type="number" id="z${i}${j}" step="0.05" min="-6" max="8"></td>`;tb+=`<td><button id="zs${i}" title="Drag the corners to any shape">Shape</button> <button id="zg${i}" title="Record where people are and fit the zone round it">Fit targets</button> <button id="zf${i}" title="Make this zone the whole room">Fit room</button> <button id="zc${i}">Clear</button> <span class="sub" id="zk${i}"></span></td></tr>`;}$('zt').innerHTML=tb;
for(let i=0;i<3;i++){for(let j=0;j<4;j++)$('z'+i+j).addEventListener('change',e=>setv('z'+(i+1)+'xyxy'[j]+(j<2?1:2),Math.round(e.target.value*1000)));$('zc'+i).addEventListener('click',()=>setZone(i,[0,0,0,0]));$('zf'+i).addEventListener('click',()=>setZone(i,roomBox()));}
$('ztype').addEventListener('change',e=>setw('zt',e.target.value));$('multi').addEventListener('change',e=>setw('multi',e.target.checked?1:0));
for(const k of ['timeout','hold','mvs','mvr','mvh','sth','igf','igt'])$(k).addEventListener('change',e=>setv(k,e.target.value));
[60,300,900,1800].forEach((v,i)=>$('ph'+i).addEventListener('click',()=>setv('sth',v)));
/* v1.11 polygon zones: a shape counts while the zone rectangle is its bounding box */
let pe=-1, pts=[], pdrag=-1, cap=null, ptap={}, plong=0, pst=null;
const zOn=z=>!!(z&&(z[0]||z[1]||z[2]||z[3]));
const zn=i=>(D&&D.zname&&D.zname[i])||('Zone '+(i+1));
const pbb=p=>[Math.min(...p.map(q=>q[0])),Math.min(...p.map(q=>q[1])),Math.max(...p.map(q=>q[0])),Math.max(...p.map(q=>q[1]))];
const r10=v=>Math.round(v/10)*10;
function zpoly(i){ const p=D&&D.zp&&D.zp[i], z=D&&D.zc&&D.zc[i]; if(!p||p.length<3||!zOn(z)) return null; const b=pbb(p);
 return Math.abs(b[0]-Math.min(z[0],z[2]))<=15&&Math.abs(b[2]-Math.max(z[0],z[2]))<=15&&Math.abs(b[1]-Math.min(z[1],z[3]))<=15&&Math.abs(b[3]-Math.max(z[1],z[3]))<=15?p:null; }
function shapeStart(i,p){ if(cap){ cap=null; $('cbar').style.display='none'; } draw=-1; for(let j=0;j<3;j++)$('d'+j).classList.remove('on');
 const z=D.zc[i]; let q=p||zpoly(i);
 if(!q&&zOn(z)){ const a=Math.min(z[0],z[2]),b=Math.min(z[1],z[3]),c=Math.max(z[0],z[2]),d=Math.max(z[1],z[3]); q=[[a,b],[c,b],[c,d],[a,d]]; }
 if(!q){ const b=roomBox(), cx=(b[0]+b[2])/2, cy=Math.max(600,(b[1]+b[3])/2); q=[[cx-500,cy-500],[cx+500,cy-500],[cx+500,cy+500],[cx-500,cy+500]]; }
 pts=q.map(v=>[v[0],v[1]]); pe=i; $('pttl').textContent=zn(i)+': shape'; $('pbar').style.display=''; map.last=null; map(); }
function shapeEnd(){ pe=-1; pts=[]; pdrag=-1; $('pbar').style.display='none'; map.last=null; if(D) map(); }
async function shapeSave(){ const i=pe; if(i<0||pts.length<3) return; const p=pts.map(q=>[r10(q[0]),Math.max(0,r10(q[1]))]), b=pbb(p);
 if(b[2]-b[0]<200||b[3]-b[1]<200){ alert('The shape is too small (under 20 cm across).'); return; }
 busy++; let t='';
 try{ const r=await fetch('/poly?z='+(i+1)+'&v='+p.map(q=>q[0]+','+q[1]).join(';'),{signal:AbortSignal.timeout(4000)}); t=await r.text(); }catch(e){ t='no answer'; }
 busy--; if(t!=='queued'){ alert('Shape not saved: '+t); return; }
 for(let j=0;j<4;j++) await setv('z'+(i+1)+'xyxy'[j]+(j<2?1:2),b[j]); shapeEnd(); }
async function shapeRect(){ const i=pe; if(i<0) return; try{ await fetch('/poly?z='+(i+1)+'&v=',{signal:AbortSignal.timeout(4000)}); }catch(e){} shapeEnd(); }
/* Fit targets: record where the radar sees people, then an octagon around it (+30 cm) */
function oct(p,m){ const A=[0,1,2,3,4,5,6,7].map(k=>k*Math.PI/4), d=A.map(a=>Math.max(...p.map(q=>q[0]*Math.cos(a)+q[1]*Math.sin(a)))+m), o=[];
 for(let k=0;k<8;k++){ const a1=A[k],a2=A[(k+1)%8],d1=d[k],d2=d[(k+1)%8],det=Math.cos(a1)*Math.sin(a2)-Math.sin(a1)*Math.cos(a2);
  const v=[r10((d1*Math.sin(a2)-d2*Math.sin(a1))/det),Math.max(0,r10((Math.cos(a1)*d2-Math.cos(a2)*d1)/det))]; const l=o[o.length-1];
  if(!l||Math.hypot(l[0]-v[0],l[1]-v[1])>=20) o.push(v); }
 if(o.length>1&&Math.hypot(o[0][0]-o[o.length-1][0],o[0][1]-o[o.length-1][1])<20) o.pop();
 if(o.length<3){ const b=pbb(p); return [[b[0]-m,Math.max(0,b[1]-m)],[b[2]+m,Math.max(0,b[1]-m)],[b[2]+m,b[3]+m],[b[0]-m,b[3]+m]].map(v=>[r10(v[0]),r10(v[1])]); }
 return o; }
function capStart(i){ if(pe>=0) shapeEnd(); cap={z:i,p:[],end:Date.now()+(+$('capd').value||30)*1000}; $('cttl').textContent=zn(i)+': recording'; $('cbar').style.display=''; capTick(); }
function capTick(){ if(!cap) return;
 if(D){ D.t.forEach(t=>{ if(t&&t[1]!=null&&(t[0]||t[1])) cap.p.push([t[0],t[1]]); }); (D.trk||[]).forEach(q=>{ if(q[2]==1) cap.p.push([q[0],q[1]]); }); }
 const left=Math.max(0,Math.ceil((cap.end-Date.now())/1000));
 $('ctxt').textContent=left+' s left · '+cap.p.length+' positions · walk round the edge of the area, or sit / stand where the zone should be';
 if(left<=0) capDone(); }
function capDone(){ const c=cap; cap=null; $('cbar').style.display='none'; if(!c) return;
 if(c.p.length<5){ alert('The radar saw fewer than 5 positions - nothing to fit. Try again and move a little.'); return; }
 shapeStart(c.z,oct(c.p,300)); $('pttl').textContent=zn(c.z)+': fitted to '+c.p.length+' positions - check it, then Save shape'; }
$('psave').onclick=shapeSave; $('pcancel').onclick=shapeEnd; $('prect').onclick=shapeRect; $('cstop').onclick=()=>{ if(cap) cap.end=0; capTick(); }; $('ccancel').onclick=()=>{ cap=null; $('cbar').style.display='none'; map.last=null; };
for(let i=0;i<3;i++){ $('zs'+i).addEventListener('click',()=>shapeStart(i)); $('zg'+i).addEventListener('click',()=>capStart(i)); }
const pdel=j=>{ clearTimeout(plong); plong=0; pdrag=-1; ptap={}; if(pts.length>3){ pts.splice(j,1); map.last=null; map(); } };
$('map').addEventListener('contextmenu',e=>{ if(pe>=0) e.preventDefault(); });

const AQC=['#00B050','#E6C700','#F08C00','#D93025','#8E44AD'],AQN=['Excellent','Good','Moderate','Sensitive','Unhealthy'],HVC={heat:'#c0392b',cool:'#1f6feb',free:'#1e8e3e',vent:'#00838f',purge:'#8e44ad',fan:'#3a4a66',closed:'#6b5a2a',idle:'#2a3645'};
function aq(){ const a=D&&D.aq; if(!a) return; $('aqWrap').style.display='';   /* v1.15 */
 const l=a.lvl, w=l==null||l<0, e=$('aqLvl'); e.textContent=w?'Warming up':AQN[Math.min(4,l)]; e.style.color=w?'#8b9bb0':AQC[Math.min(4,l)];
 const f=(v,d,u)=>v==null?'—':v.toFixed(d)+(u||'');
 const rows=[['PM2.5',f(a.pm25,1,' µg/m³'),'PM AQI',f(a.pmaqi,0)],['PM1 / PM4 / PM10',f(a.pm1,1)+' / '+f(a.pm4,1)+' / '+f(a.pm10,1),'PM status',a.pmt||''],
  ['VOC index',f(a.voc,0),'VOC Fast',f(a.vocf,0)],['VOC status',a.voct||'','',''],['Temperature',f(a.tf,1,' °F'),'Feels like',f(a.fl,1,' °F')],
  ['Humidity',f(a.rh,0,' %'),'Absolute',f(a.ah,1,' g/m³')],['Dew point',f(a.dew,1,' °F'),'','']];
 const tb=$('aqT'); tb.textContent=''; for(const r of rows){ const tr=document.createElement('tr'); r.forEach((c,i)=>{ const td=document.createElement(i%2?'td':'th'); td.textContent=c; tr.appendChild(td); }); tb.appendChild(tr); }
 const h=$('hvHead'); h.textContent=a.hv||'no data from Home Assistant'; h.style.background=HVC[a.hvk]||'#2a3645'; h.style.color='#fff';
 $('hvSub').textContent=a.hvs||''; $('hvDet').textContent=a.hvd||''; }
let ACT=[['Hands / teeth',5],['Shaving / makeup',10],['Dressing',10],['Toilet',20],['Nails',30],['Shower',30],['Tub soak',60]], ADEF=[1800,1800,600,600];let bathInit=false;
function bath(){ if(!D||!D.zh) return;
 if(!bathInit){ bathInit=true; $('bathWrap').style.display=''; let h='';
  if(D.acts) ACT=D.acts.split('|').map(s=>{ const k=s.lastIndexOf(':'); return [s.slice(0,k),+s.slice(k+1)]; });   /* v1.13 */
  if(D.actd) ADEF=D.actd.split(',').map(Number);
  $('bathTtl').textContent=D.actt||'Bathroom activities';
  $('bathDefTxt').textContent=[0,1,2].map(i=>zn(i)+' '+Math.round(ADEF[i]/60)+' min').join(', ')+', anywhere else (Still hold) '+Math.round(ADEF[3]/60)+' min';
  for(let i=0;i<3;i++){ const nm=(D.zname&&D.zname[i])||('Zone '+(i+1));
   h+=`<div class="row" style="margin-top:${i?12:0}px;align-items:flex-end"><div style="min-width:160px"><span class="dot" style="background:${ZC[i]}"></span><b>${nm}</b><div class="sub" id="bz${i}">&mdash;</div></div><label>Still hold min<input type="number" id="bh${i}" min="0" max="120" step="1"></label><div style="display:flex;flex-wrap:wrap;gap:6px">`+ACT.map((a,j)=>`<button id="ba${i}_${j}">${a[0]} ${a[1]} min</button>`).join('')+`</div></div>`; }
  $('bathRows').innerHTML=h;
  for(let i=0;i<3;i++){ $('bh'+i).addEventListener('change',e=>setv('zh'+(i+1),Math.round((+e.target.value||0)*60))); ACT.forEach((a,j)=>$('ba'+i+'_'+j).addEventListener('click',()=>setv('zh'+(i+1),a[1]*60))); }
  $('bathDef').addEventListener('click',async()=>{ for(const [k,v] of ['zh1','zh2','zh3','sth'].map((k,i)=>[k,ADEF[i]])) await setv(k,v); }); }
 for(let i=0;i<3;i++){ const s=D.zh[i], on=D.z&&D.z[i], drawn=D.zc&&D.zc[i]&&D.zc[i].some(v=>v);
  fill('bh'+i,s,v=>Math.round(v/60)); ACT.forEach((a,j)=>$('ba'+i+'_'+j).classList.toggle('on',s!=null&&Math.round(s)===a[1]*60));
  $('bz'+i).textContent=!drawn?'zone not drawn':(on?'someone here · ':'')+(s>0?'held '+Math.round(s/60)+' min when still':'uses Still hold ('+Math.round((D.n.sth||0)/60)+' min)'); } }
const owned=t=>D&&(D.trk||[]).some(k=>k[2]!=3&&Math.hypot(k[0]-t[0],k[1]-t[1])<1000);const mmss=s=>Math.floor(s/60)+':'+String(Math.round(s%60)).padStart(2,'0');
const roomRange=()=>Math.min(7.5,Math.max(0.5,Math.round(farCm/10)/10));   /* v1.8: farthest point of the room in view */
[[5,10],[10,5],[25,5],[10,30]].forEach((p,i)=>$('pr'+i).addEventListener('click',async()=>{await setv('mvs',p[0]);await setv('mvr',roomRange());await setv('mvh',p[1]);}));
let spk=[];const mvQual=t=>t&&t[1]!=null&&D&&D.n.mvr!=null&&D.n.mvs!=null&&t[1]<=D.n.mvr*1000&&Math.abs(t[2]||0)>=D.n.mvs;
function map(){ if(!D) return; if(!roomInit) initRoom(D.room); else syncRoom();
 const el=$('map'), W=Math.max(280,Math.round(el.clientWidth||900)), M=30, maxH=Math.min(640,Math.round(W*0.9));
 const sc=Math.min((W-2*M)/room.w,(maxH-2*M)/room.d), RW=room.w*sc, RD=room.d*sc, H=Math.round(RD+2*M), ox=(W-RW)/2, oy=M;
 const cx=ox+room.sx*sc, cy=oy+room.sy*sc, th=room.aim*Math.PI/180, fl=room.flip?-1:1, co=Math.cos(th), si=Math.sin(th), k=sc/10;
 const T=(x,y)=>[cx+(fl*x*co+y*si)*k, cy+(fl*x*si-y*co)*k];                    /* sensor mm (x right, y ahead) -> px */
 geo={W,H,ox,oy,sc,cx,cy,co,si,fl,k};
 const P=(a)=>a.map(q=>q[0].toFixed(1)+','+q[1].toFixed(1)).join(' ');
 const arcP=(r,a0,a1)=>{ const o=[]; for(let i=0;i<=28;i++){ const a=a0+(a1-a0)*i/28; o.push(T(r*Math.sin(a),r*Math.cos(a))); } return o; };
 const box=(x0,y0,x1,y1)=>[T(x0,y0),T(x1,y0),T(x1,y1),T(x0,y1)];
 let f=`<polygon points="${P([[cx,cy],...arcP(R,-HALF,HALF)])}" fill="#8fb4ff0d" stroke="#ffffff22"/>`;
 for(let m=1;m<=6;m++) f+=`<polyline points="${P(arcP(m*1000,-HALF,HALF))}" fill="none" stroke="#ffffff1c"/>`;
 if(D.n.igf!=null&&D.n.igt!=null&&D.n.igt>D.n.igf){ const r0=D.n.igf*1000, r1=Math.min(D.n.igt*1000,R); if(r1>r0) f+=`<polygon points="${P([...arcP(r0,-HALF,HALF),...arcP(r1,HALF,-HALF)])}" fill="#f8717126" stroke="#f8717166"/>`; }
 if(D.n.mvr!=null){ const y=D.n.mvr*1000, hx=Math.min(y*Math.tan(HALF),Math.sqrt(Math.max(0,R*R-y*y))); const [a,b]=T(-hx,y),[c2,d2]=T(hx,y); f+=`<line x1="${a.toFixed(1)}" y1="${b.toFixed(1)}" x2="${c2.toFixed(1)}" y2="${d2.toFixed(1)}" stroke="#4aa3ff" stroke-dasharray="5 4" opacity=".75"/>`; }
 D.zc.forEach((z,i)=>{ if(!zOn(z)||i===pe) return; const pp=zpoly(i);
   if(pp) f+=`<polygon points="${P(box(z[0],z[1],z[2],z[3]))}" fill="none" stroke="${ZC[i]}" stroke-dasharray="3 4" opacity=".55"/>`;
   f+=`<polygon points="${P(pp?pp.map(q=>T(q[0],q[1])):box(z[0],z[1],z[2],z[3]))}" fill="${ZC[i]}" opacity="${D.z[i]?.38:.17}" stroke="${ZC[i]}" stroke-width="1.5"/>`; });
 if(room.trail!==false) trails.forEach((t,i)=>{ if(t.length>1) f+=`<polyline points="${P(t.map(q=>T(q[0],q[1])))}" fill="none" stroke="${TC[i]}" stroke-width="2" opacity=".45"/>`; });
 (D.trk||[]).forEach(q=>{ if(q[2]==0) return; const [x,y]=T(q[0],q[1]); f+=`<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="13" fill="none" stroke="${q[2]==3?'#8b9bb0':'#f0c060'}" stroke-width="2" stroke-dasharray="4 3"/>`; });
 (D.cl||[]).forEach(c=>{ if(c[2]<2) return; const [x,y]=T(c[0],c[1]); f+=`<path d="M${(x-7).toFixed(1)} ${(y-7).toFixed(1)}l14 14m0 -14l-14 14" stroke="#8b9bb0" stroke-width="2.5"/>`; });   /* v1.14 still clutter */
 D.t.forEach((t,i)=>{ if(!t||t[1]==null) return; const [x,y]=T(t[0],t[1]), mv=mvQual(t), c=owned(t)?TC[i]:'#6d7d92'; f+=`<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="9" fill="${mv?c:'none'}" stroke="${c}" stroke-width="3"/>`; });
 if(rub){ const a=toS(rub[0],rub[1]), b=toS(rub[2],rub[3]); f+=`<polygon points="${P(box(Math.min(a[0],b[0]),Math.max(0,Math.min(a[1],b[1])),Math.max(a[0],b[0]),Math.max(a[1],b[1])))}" fill="${ZC[draw]}" opacity=".32" stroke="#fff" stroke-dasharray="4 3"/>`; }
 let s=`<defs><clipPath id="rclip"><rect x="${ox.toFixed(1)}" y="${oy.toFixed(1)}" width="${RW.toFixed(1)}" height="${RD.toFixed(1)}"/></clipPath></defs>`;
 s+=`<rect x="${ox.toFixed(1)}" y="${oy.toFixed(1)}" width="${RW.toFixed(1)}" height="${RD.toFixed(1)}" fill="#ffffff05"/>`;
 { let st=uf(); while(st*sc<14) st*=2; for(let x=st;x<room.w-1;x+=st) s+=`<line x1="${(ox+x*sc).toFixed(1)}" y1="${oy}" x2="${(ox+x*sc).toFixed(1)}" y2="${(oy+RD).toFixed(1)}" stroke="#ffffff0b"/>`;
   for(let y=st;y<room.d-1;y+=st) s+=`<line x1="${ox}" y1="${(oy+y*sc).toFixed(1)}" x2="${(ox+RW).toFixed(1)}" y2="${(oy+y*sc).toFixed(1)}" stroke="#ffffff0b"/>`; }
 s+=`<g opacity=".22">${f}</g><g clip-path="url(#rclip)">${f}</g>`;
 s+=`<rect x="${ox.toFixed(1)}" y="${oy.toFixed(1)}" width="${RW.toFixed(1)}" height="${RD.toFixed(1)}" fill="none" stroke="#f0c060" stroke-width="2.5"/>`;
 s+=`<text x="${(ox+RW/2).toFixed(1)}" y="${(oy-9).toFixed(1)}" font-size="12" fill="#f0c060" text-anchor="middle">${fmtU(room.w)}</text>`;
 s+=`<text x="${(ox-9).toFixed(1)}" y="${(oy+RD/2).toFixed(1)}" font-size="12" fill="#f0c060" text-anchor="middle" transform="rotate(-90 ${(ox-9).toFixed(1)} ${(oy+RD/2).toFixed(1)})">${fmtU(room.d)}</text>`;
 const inR=(x,y)=>x>ox+6&&x<ox+RW-6&&y>oy+10&&y<oy+RD-4;
 for(let m=1;m<=6;m++){ for(const a of [-0.95,0.95,-0.6,0.6]){ const [x,y]=T(m*1000*Math.sin(a),m*1000*Math.cos(a)); if(inR(x,y)){ s+=`<text x="${x.toFixed(1)}" y="${(y+4).toFixed(1)}" font-size="10" fill="#8b9bb0" text-anchor="middle">${m} m</text>`; break; } } }
 D.zc.forEach((z,i)=>{ if(!zOn(z)||i===pe) return; const pp=zpoly(i), q=pp?pp.map(v=>T(v[0],v[1])):box(z[0],z[1],z[2],z[3]), mx=q.reduce((a,b)=>a+b[0],0)/q.length, my=q.reduce((a,b)=>a+b[1],0)/q.length; s+=`<text x="${mx.toFixed(1)}" y="${(my+4).toFixed(1)}" font-size="11" fill="#fff" text-anchor="middle">${zn(i)}</text>`; });
 if(cap) cap.p.slice(-400).forEach(v=>{ const [x,y]=T(v[0],v[1]); s+=`<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="2.5" fill="${ZC[cap.z]}"/>`; });
 if(pe>=0&&pts.length>2){ const q=pts.map(v=>T(v[0],v[1])); s+=`<polygon points="${P(q)}" fill="${ZC[pe]}" fill-opacity=".3" stroke="#fff" stroke-width="2"/>`;
   if(pts.length<8) q.forEach((a,j)=>{ const b=q[(j+1)%q.length]; s+=`<circle data-h="m${j}" cx="${((a[0]+b[0])/2).toFixed(1)}" cy="${((a[1]+b[1])/2).toFixed(1)}" r="6" fill="#16202c" stroke="#fff" stroke-dasharray="2 2" style="cursor:copy"/>`; });
   q.forEach((a,j)=>{ s+=`<circle data-h="p${j}" cx="${a[0].toFixed(1)}" cy="${a[1].toFixed(1)}" r="9" fill="${ZC[pe]}" stroke="#fff" stroke-width="2" style="cursor:move;touch-action:none"/>`; }); }
 if(D.n.mvr!=null){ const [x,y]=T(0,D.n.mvr*1000); if(inR(x,y)) s+=`<text x="${x.toFixed(1)}" y="${(y-5).toFixed(1)}" font-size="10" fill="#4aa3ff" text-anchor="middle">moving range ${D.n.mvr} m</text>`; }
 (D.trk||[]).forEach(q=>{ if(q[2]==0) return; const [x,y]=T(q[0],q[1]); s+=`<text x="${(x+17).toFixed(1)}" y="${(y+18).toFixed(1)}" font-size="11" fill="${q[2]==3?'#8b9bb0':'#f0c060'}">${q[2]==3?'checking':q[2]==1?'held':'leaving'} ${mmss(q[3])}</text>`; });
 (D.cl||[]).forEach(c=>{ if(c[2]<2) return; const [x,y]=T(c[0],c[1]); s+=`<text x="${(x+10).toFixed(1)}" y="${(y-9).toFixed(1)}" font-size="10" fill="#8b9bb0">still object ×${c[2]}</text>`; });
 D.t.forEach((t,i)=>{ if(!t||t[1]==null) return; const [x,y]=T(t[0],t[1]); s+=`<text x="${(x+13).toFixed(1)}" y="${(y+4).toFixed(1)}" font-size="12" font-weight="700" fill="#fff">${i+1} &middot; ${(Math.hypot(t[0],t[1])/1000).toFixed(2)} m</text>`; });
 /* what the fan sees of the room */
 let far=0, cov=0, tot=0;
 for(let i=0;i<=24;i++) for(let j=0;j<=24;j++){ const x=room.w*i/24, y=room.d*j/24, dx=x-room.sx, dy=y-room.sy, d=Math.hypot(dx,dy);
   let a=Math.atan2(dx,-dy)-th; a=Math.atan2(Math.sin(a),Math.cos(a)); const inF=d<1||Math.abs(a)<=HALF;
   if(inF) far=Math.max(far,d); if(i>0&&i<24&&j>0&&j<24){ tot++; if(inF&&d<=R/10) cov++; } }
 farCm=far;
 const kr=Math.max(46,Math.min(110,140*sc)), [kx,ky]=T(0,kr/k);
 s+=`<line x1="${cx.toFixed(1)}" y1="${cy.toFixed(1)}" x2="${kx.toFixed(1)}" y2="${ky.toFixed(1)}" stroke="#fff" stroke-width="2" stroke-dasharray="4 3" opacity=".8"/>`;
 s+=`<circle data-h="a" cx="${kx.toFixed(1)}" cy="${ky.toFixed(1)}" r="9" fill="#2a3a52" stroke="#fff" stroke-width="2" style="cursor:grab"/>`;
 s+=`<circle data-h="s" cx="${cx.toFixed(1)}" cy="${cy.toFixed(1)}" r="10" fill="#fff" stroke="#16202c" stroke-width="2" style="cursor:move"/>`;
 { const lx=Math.min(ox+RW-4,Math.max(ox+4,cx)), ly=cy<oy+RD/2?cy+24:cy-16; s+=`<text x="${lx.toFixed(1)}" y="${ly.toFixed(1)}" font-size="11" fill="#8b9bb0" text-anchor="${cx>ox+RW*0.66?'end':cx<ox+RW*0.33?'start':'middle'}">sensor &middot; ${Math.round(room.aim)}&deg;</text>`; }
 const pName=$('r_place').selectedOptions[0]?$('r_place').selectedOptions[0].textContent:'';
 const txt=`<span style="color:#8b9bb0">${D.room_src==='device'?'saved on the device':'device default layout'}</span> &middot; sensor ${pName} aiming ${Math.round(room.aim)}&deg; &middot; farthest point in view <b>${(far/100).toFixed(2)} m</b> (${fmtU(far)})${far>R/10?' &mdash; past the radar\'s 6 m':''} &middot; covers <b>${tot?Math.round(cov/tot*100):0}%</b> of the floor &middot; Moving presets set range ${roomRange()} m &middot; ${room.fence!==false?'tracking only inside the room':'<span style="color:#f0c060">room fence off</span>'}`;
 if($('roomTxt').innerHTML!==txt) $('roomTxt').innerHTML=txt;
 if(s===map.last&&W===map.W&&H===map.H) return; map.last=s; map.W=W; map.H=H; el.setAttribute('viewBox',`0 0 ${W} ${H}`); el.style.height=H+'px'; el.innerHTML=s; }
/* px -> sensor mm */
const toS=(px,py)=>{ const g=geo, dx=(px-g.cx)/g.k, dy=(py-g.cy)/g.k; return [g.fl*(dx*g.co+dy*g.si), dx*g.si-dy*g.co]; };
const sv=$('map'); const pos=e=>{ const r=sv.getBoundingClientRect(); return [(e.clientX-r.left)*geo.W/r.width,(e.clientY-r.top)*geo.H/r.height]; };
sv.addEventListener('pointerdown',e=>{ if(!geo) return; const h=e.target&&e.target.dataset?e.target.dataset.h:null;
  if(h&&h[0]==='p'){ const j=+h.slice(1), now=Date.now(); e.preventDefault();
    if(ptap.h===h&&now-ptap.t<450){ pdel(j); return; }   /* double-tap removes the corner */
    ptap={h,t:now}; pdrag=j; pst=pos(e); clearTimeout(plong); plong=setTimeout(()=>{ if(pdrag===j&&pst) pdel(j); },700);   /* long-press too */
    sv.setPointerCapture(e.pointerId); return; }
  if(h&&h[0]==='m'){ const j=+h.slice(1), a=pts[j], b=pts[(j+1)%pts.length]; pts.splice(j+1,0,[r10((a[0]+b[0])/2),r10((a[1]+b[1])/2)]); pdrag=j+1; sv.setPointerCapture(e.pointerId); e.preventDefault(); map.last=null; map(); return; }
  if(h){ rdrag=h; sv.setPointerCapture(e.pointerId); e.preventDefault(); return; }
  if(draw<0||pe>=0) return; const [p,q]=pos(e); rub=[p,q,p,q]; sv.setPointerCapture(e.pointerId); map.last=null; map(); });
sv.addEventListener('pointermove',e=>{ if(!geo) return;
  if(pdrag>=0){ const [x,y]=pos(e); if(pst){ if(Math.hypot(x-pst[0],y-pst[1])<6) return; pst=null; clearTimeout(plong); ptap={}; } const q=toS(x,y); pts[pdrag]=[r10(q[0]),Math.max(0,r10(q[1]))]; map.last=null; map(); return; }
  if(rdrag){ const [x,y]=pos(e), rx=(x-geo.ox)/geo.sc, ry=(y-geo.oy)/geo.sc;
    if(rdrag==='s'){ const cx=Math.min(room.w,Math.max(0,rx)), cy=Math.min(room.d,Math.max(0,ry)), d=[cy,room.w-cx,room.d-cy,cx], m=d.indexOf(Math.min(...d));
      room.sx=m==1?room.w:m==3?0:Math.round(cx); room.sy=m==0?0:m==2?room.d:Math.round(cy); room.place='custom'; if(room.aimAuto) aimCentre(); }
    else { room.aim=Math.round(((Math.atan2(rx-room.sx,-(ry-room.sy))*180/Math.PI)+360)%360); room.aimAuto=false; }
    roomUi(); map.last=null; map(); return; }
  if(!rub) return; const [p,q]=pos(e); rub[2]=p; rub[3]=q; map.last=null; map(); });
const endP=()=>{ clearTimeout(plong); plong=0; pst=null; if(pdrag>=0){ pdrag=-1; return; } if(rdrag){ rdrag=null; saveRoom(); return; } if(!rub||!geo) return;
  const a=toS(rub[0],rub[1]), b=toS(rub[2],rub[3]); rub=null;
  if(Math.abs(a[0]-b[0])>=200&&Math.abs(a[1]-b[1])>=200){ const r10=v=>Math.round(v/10)*10, y0=Math.max(0,Math.min(a[1],b[1])); setZone(draw,[r10(Math.min(a[0],b[0])),r10(y0),r10(Math.max(a[0],b[0])),r10(Math.max(a[1],b[1]))]); }
  map.last=null; map(); };
sv.addEventListener('pointerup',endP); sv.addEventListener('pointercancel',endP);
function render(){$('ttl').textContent=D.name+' - radar map';document.title=D.name+' - radar map';$('meta').textContent=`fw ${D.fw||'?'} · up ${Math.round((D.up||0)/60)} min · rssi ${D.rssi==null?'?':Math.round(D.rssi)} dBm`;
 $('cnt').textContent=D.count==null?'—':D.count;for(const[p,v]of[['pOcc',D.occ],['pMv',D.mv],['pMv2',D.mv],['pRaw',D.mvraw],['pRawO',D.occraw],['pSt',D.st],['pZ0',D.z[0]],['pZ1',D.z[1]],['pZ2',D.z[2]]])$(p).classList.toggle('on',!!v);
 {const k=(D.trk||[]).filter(q=>q[2]!=3),h=k.filter(q=>q[2]==1),l=k.filter(q=>q[2]==2);$('prN').textContent=k.length;$('prS').textContent=!k.length?'empty':h.length?mmss(Math.max(...h.map(q=>q[3]))):l.length?mmss(Math.max(...l.map(q=>q[3]))):'in view';$('prL').textContent=!k.length?((D.trk||[]).length?'checking a new target':'nobody tracked'):h.length?'lost while still, holding':l.length?'walked out, clearing':'radar can see them';}
 for(const[p,v]of[['ph',D.ps],['pr',D.pm]])for(let i=0;i<4;i++)$(p+i).classList.toggle('on',$(p+i).textContent===v);$('phC').textContent=D.ps||'—';$('pmC').textContent=D.pm||'—';
 {const now=Date.now();let f=null;D.t.forEach(t=>{if(t&&t[1]!=null&&t[2]!=null&&(D.n.mvr==null||t[1]<=D.n.mvr*1000)){const a=Math.abs(t[2]);if(f==null||a>f)f=a;}});if(f!=null)spk.push([now,f]);spk=spk.filter(x=>now-x[0]<30000);
  $('spdNow').textContent=f==null?'—':Math.round(f);$('spdPk').textContent=spk.length?Math.round(Math.max(...spk.map(x=>x[1]))):'—';
  const th=D.n.mvs;for(const id of ['spdNow','spdPk']){const v=+$(id).textContent;$(id).style.color=(th!=null&&v>=th)?'#4aa3ff':'';}}
 let tb='';D.t.forEach((t,i)=>{const on=t&&t[1]!=null;if(on){trails[i].push([t[0],t[1]]);if(trails[i].length>40)trails[i].shift();}else trails[i]=[];
  tb+=`<tr><td><span class="dot" style="background:${TC[i]}"></span>${i+1}</td><td>${on?m2(t[0]):'—'}</td><td>${on?m2(t[1]):'—'}</td><td>${on?m2(Math.hypot(t[0],t[1])):'—'}</td><td>${on?Math.round(Math.atan2(t[0],t[1])*180/Math.PI)+'&deg;':'—'}</td><td style="${mvQual(t)?'color:#4aa3ff;font-weight:700':''}">${on&&t[2]!=null?Math.abs(t[2])+' cm/s':'—'}</td></tr>`;});$('tt').innerHTML=tb;
 for(let i=0;i<3;i++)for(let j=0;j<4;j++)fill('z'+i+j,D.zc[i]?D.zc[i][j]:null,v=>+(v/1000).toFixed(2));
 fill('timeout',D.n.timeout);fill('hold',D.n.hold);fill('mvs',D.n.mvs);fill('mvr',D.n.mvr);fill('mvh',D.n.mvh);fill('sth',D.n.sth);fill('igf',D.n.igf);fill('igt',D.n.igt);fill('multi',D.multi);if(D.zt>=0)fill('ztype',D.zt);bath();aq();capTick();for(let i=0;i<3;i++){const pp=zpoly(i);$('zk'+i).textContent=pp?'shape, '+pp.length+' corners':'';}if(!rub)map();}
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

// v1.8: room layout stored on the device. The httpd task only queues (rw_room_cmd); the main loop saves to flash.
struct RwRoomSaved { uint32_t magic; char js[200]; };
static char rw_room[2][200] = {"", ""};
static volatile int rw_room_i = 0;
static char rw_room_in[200];
static volatile int rw_room_cmd = 0;          // 1 = save rw_room_in, 2 = reset
static esphome::ESPPreferenceObject rw_room_pref;
static bool rw_room_ready = false;
static void rw_room_set(const char *js) { const int w = 1 - rw_room_i; strlcpy(rw_room[w], js, sizeof rw_room[w]); rw_room_i = w; }


// v1.9: the room plan for the tracker (presence_2450.h v1.3). Reads the saved layout, else the yaml default.
static float rw_jnum(const char *js, const char *key, float def) { const char *p = strstr(js, key); return p ? strtof(p + strlen(key), nullptr) : def; }
static bool rw_room_geom(float &w, float &d, float &sx, float &sy, float &aim, bool &flip, bool &fence) {
  const char *js = rw_room[rw_room_i]; if (js[0] != '{') js = rw_room_default; if (!js || js[0] != '{') return false;
  w = rw_jnum(js, "\"w\":", 0); d = rw_jnum(js, "\"d\":", 0); if (w < 50 || d < 50) return false;
  flip = strstr(js, "\"flip\":true") != nullptr; fence = strstr(js, "\"fence\":false") == nullptr;
  sx = rw_jnum(js, "\"sx\":", -1); sy = rw_jnum(js, "\"sy\":", -1);
  if (sx < 0 || sy < 0) {                     // short form: {"w":..,"d":..,"place":"tr"} - same spots as the page
    const char *p = strstr(js, "\"place\":\""); char a = 'b', b = 'm'; if (p) { a = p[9]; b = p[10]; }
    sy = a == 't' ? 0 : a == 'b' ? d : d / 2; sx = b == 'l' ? 0 : b == 'r' ? w : w / 2;
    if (a == 'l' || a == 'r') { sx = a == 'l' ? 0 : w; sy = d / 2; }   // "lm" / "rm" = middle of the left / right wall
  }
  if (strstr(js, "\"aim\":")) aim = rw_jnum(js, "\"aim\":", 0);
  else aim = fmodf(atan2f(w / 2 - sx, -(d / 2 - sy)) * 57.29578f + 360.0f, 360.0f);
  return true;
}

// v1.11: polygon zones (sensor mm), saved in flash; the main loop hands them to the tracker (presence_polys)
struct RwPolySaved { uint32_t magic; int16_t n[3]; int16_t p[3][8][2]; };
static RwPolySaved rw_poly = {};
static RwPolySaved rw_poly_new = {};
static volatile int rw_poly_cmd = 0;   // 1..3 = that zone queued
static esphome::ESPPreferenceObject rw_poly_pref;
static void rw_poly_apply(int k) {
  float p[8][2]; int n = rw_poly.n[k]; if (n < 3 || n > 8) n = 0;
  for (int j = 0; j < n; j++) { p[j][0] = rw_poly.p[k][j][0]; p[j][1] = rw_poly.p[k][j][1]; }
  presence_polys(k, n, p);
}
static esp_err_t rw_poly_h(httpd_req_t *req) {   // /poly?z=1..3&v=x,y;x,y;... (mm, 3-8 corners), v empty = rectangle
  char q[240], z[4], v[200]; httpd_resp_set_type(req, "text/plain");
  if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK) return httpd_resp_sendstr(req, "ERR query");
  if (rw_poly_cmd) return httpd_resp_sendstr(req, "ERR busy");
  if (httpd_query_key_value(q, "z", z, sizeof z) != ESP_OK || z[0] < '1' || z[0] > '3' || z[1]) return httpd_resp_sendstr(req, "ERR zone");
  if (httpd_query_key_value(q, "v", v, sizeof v) != ESP_OK) v[0] = 0;
  for (size_t i = 0; v[i]; i++) if (!isdigit((unsigned char) v[i]) && !strchr("-,;", v[i])) return httpd_resp_sendstr(req, "ERR value");
  const int k = z[0] - '1'; int n = 0; const char *s = v;
  while (*s) {
    if (n >= 8) return httpd_resp_sendstr(req, "ERR too many corners");
    char *e; long x = strtol(s, &e, 10); if (e == s || *e != ',') return httpd_resp_sendstr(req, "ERR value");
    s = e + 1; long y = strtol(s, &e, 10); if (e == s || (*e && *e != ';')) return httpd_resp_sendstr(req, "ERR value");
    if (x < -8000 || x > 8000 || y < 0 || y > 8000) return httpd_resp_sendstr(req, "ERR range");
    rw_poly_new.p[k][n][0] = (int16_t) x; rw_poly_new.p[k][n][1] = (int16_t) y; n++;
    s = *e ? e + 1 : e;
  }
  if (n && n < 3) return httpd_resp_sendstr(req, "ERR need 3 corners");
  rw_poly_new.n[k] = (int16_t) n; rw_poly_cmd = k + 1;
  return httpd_resp_sendstr(req, "queued");
}

static esp_err_t rw_data(httpd_req_t *req) {
  auto &c = radar_web_cfg(); std::string o; o.reserve(2000); o += "{";
  o += "\"name\":\""; o += esphome::App.get_friendly_name(); o += "\",";
  { const char *rj = rw_room[rw_room_i];   // v1.8: the saved layout wins over the yaml default
    if (rj[0] == '{') { o += "\"room\":"; o += rj; o += ",\"room_src\":\"device\","; }
    else if (rw_room_default) { o += "\"room\":"; o += rw_room_default; o += ",\"room_src\":\"default\","; } }
  { const char *aj = rw_aq[rw_aq_i]; if (aj[0] == '{') { o += "\"aq\":"; o += aj; o += ","; } }   // v1.15
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
  // presence tracks: x mm, y mm, state 0 in view / 1 held still / 2 walked out / 3 checking (v1.14), seconds left
  { uint32_t now = millis(); o += "\"trk\":["; bool first = true;
    for (const auto &t : pr_t) { PrTrack k = t; if (!k.used) continue; uint32_t age = now - k.seen;
      int st = !k.lock ? 3 : age < 1500 ? 0 : (k.limit > 0 && pr_out(k) ? 2 : 1);
      if (!k.lock) { k.limit = PR_LOCK_WIN_MS; age = now - k.born; }   // checking: seconds left to travel
      if (!first) o += ","; first = false; o += "["; rw_num(o, roundf(k.x)); o += ","; rw_num(o, roundf(k.y)); o += ","; o += std::to_string(st); o += ",";
      rw_num(o, age < k.limit ? (float) ((k.limit - age) / 1000) : 0.0f); o += "]"; }
    o += "],"; }
  { uint32_t now = millis(); o += "\"cl\":["; bool first = true;   // v1.14 still-clutter spots
    for (const auto &c0 : pr_cl) { PrClut c = c0; if (!c.n) continue; if (!first) o += ","; first = false;
      o += "["; rw_num(o, roundf(c.x)); o += ","; rw_num(o, roundf(c.y)); o += ","; o += std::to_string(c.n); o += ","; rw_num(o, (float) ((now - c.last) / 1000)); o += "]"; }
    o += "],"; }
  o += "\"zp\":[";   // v1.11 polygon corners per zone
  for (int i = 0; i < 3; i++) { o += "["; const int n = rw_poly.n[i] >= 3 && rw_poly.n[i] <= 8 ? rw_poly.n[i] : 0;
    for (int j = 0; j < n; j++) { if (j) o += ","; o += "["; o += std::to_string(rw_poly.p[i][j][0]); o += ","; o += std::to_string(rw_poly.p[i][j][1]); o += "]"; }
    o += "]"; if (i < 2) o += ","; }
  o += "],";
  o += "\"rst\":"; o += std::to_string(pr_restored); o += ",";   // tracks put back at boot, -1 = no snapshot
  if (c.zh[0] || c.zh[1] || c.zh[2]) {   // v1.10
    o += "\"zh\":["; for (int i = 0; i < 3; i++) { rw_num(o, (c.zh[i] && c.zh[i]->has_state()) ? c.zh[i]->state : NAN); if (i < 2) o += ","; }
    o += "],\"zname\":["; for (int i = 0; i < 3; i++) { o += "\""; o += c.zname[i] ? c.zname[i] : ""; o += "\""; if (i < 2) o += ","; } o += "],";
    if (c.actt) { o += "\"actt\":\""; o += c.actt; o += "\","; }   // v1.13
    if (c.acts) { o += "\"acts\":\""; o += c.acts; o += "\","; }
    if (c.actd) { o += "\"actd\":\""; o += c.actd; o += "\","; } }
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
  if (k[0] == 'z' && k[1] == 'h' && k[2] >= '1' && k[2] <= '3' && k[3] == 0) return c.zh[k[2] - '1'];   // v1.10
  if (k[0] == 'z' && k[1] >= '1' && k[1] <= '3' && (k[2] == 'x' || k[2] == 'y') && (k[3] == '1' || k[3] == '2') && k[4] == 0)
    return c.zc[k[1] - '1'][(k[3] - '1') * 2 + (k[2] == 'y' ? 1 : 0)];
  return nullptr;
}
static esp_err_t rw_room_h(httpd_req_t *req) {   // v1.8: /room?v=<url-encoded json> saves, /room?reset=1 clears
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
  { static bool loaded = false;   // v1.11: polygon zones from flash at boot, then whatever the page saves
    if (!loaded) {
      loaded = true; rw_poly_pref = esphome::global_preferences->make_preference<RwPolySaved>(0x5257504CUL);
      RwPolySaved sv; if (rw_poly_pref.load(&sv) && sv.magic == 0x504F4C59) rw_poly = sv;
      for (int k = 0; k < 3; k++) { if (rw_poly.n[k] < 3 || rw_poly.n[k] > 8) rw_poly.n[k] = 0; rw_poly_apply(k); }
    }
    if (rw_poly_cmd) {
      const int k = rw_poly_cmd - 1; rw_poly.magic = 0x504F4C59; rw_poly.n[k] = rw_poly_new.n[k]; memcpy(rw_poly.p[k], rw_poly_new.p[k], sizeof rw_poly.p[k]);
      rw_poly_pref.save(&rw_poly); esphome::global_preferences->sync(); rw_poly_apply(k);
      ESP_LOGI("radar_web", "zone %d: %s", k + 1, rw_poly.n[k] ? "shape saved" : "back to a rectangle"); rw_poly_cmd = 0;
    }
  }
  if (rw_room_cmd && rw_room_ready) {   // v1.8: store / clear the room layout in flash
    RwRoomSaved sv; memset(&sv, 0, sizeof sv);
    if (rw_room_cmd == 1) { sv.magic = 0x524F4F4D; strlcpy(sv.js, rw_room_in, sizeof sv.js); rw_room_set(rw_room_in); ESP_LOGI("radar_web", "room layout saved: %s", rw_room_in); }
    else { rw_room_set(""); ESP_LOGI("radar_web", "room layout reset to the default"); }
    rw_room_pref.save(&sv); esphome::global_preferences->sync(); rw_room_cmd = 0;
  }
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
    rw_room_pref = esphome::global_preferences->make_preference<RwRoomSaved>(0x52574D32UL);   // v1.8
    { RwRoomSaved sv; if (rw_room_pref.load(&sv) && sv.magic == 0x524F4F4D) { sv.js[sizeof sv.js - 1] = 0; rw_room_set(sv.js); } }
    rw_room_ready = true;
    if (c.fw) { c.fw->add_on_state_callback([](const std::string &x) { strlcpy(rw_fw, x.c_str(), sizeof rw_fw); }); if (c.fw->has_state()) strlcpy(rw_fw, c.fw->state.c_str(), sizeof rw_fw); }
    if (c.zt) { c.zt->add_on_state_callback([](size_t i) { rw_zt = (int) i; }); auto a = c.zt->active_index(); if (a.has_value()) rw_zt = (int) *a; }
  }
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  // ctrl_port must be a real port: 0 binds an ephemeral one but httpd still sends its work queue to port 0, so the LRU
  // close never runs. With all 5 slots taken it then refuses every client and spins on the listen socket.
  cfg.server_port = 8080; cfg.ctrl_port = 32769; cfg.stack_size = 8192; cfg.max_uri_handlers = 8; cfg.max_open_sockets = 5; cfg.lru_purge_enable = true;
  cfg.keep_alive_enable = true; cfg.keep_alive_idle = 10; cfg.keep_alive_interval = 5; cfg.keep_alive_count = 3;   // a vanished browser frees its slot
  cfg.task_priority = 1;   // v1.1: same as ESPHome's loop task, so a spin here cannot starve it
  httpd_handle_t srv = nullptr; esp_err_t err = httpd_start(&srv, &cfg);
  if (err != ESP_OK) { rw_page = 2; ESP_LOGE("radar_web", "httpd_start failed: %s - will retry", esp_err_to_name(err)); return; }
  registered = true;
  static const httpd_uri_t uris[] = {
    {"/", HTTP_GET, rw_html, nullptr}, {"/data.json", HTTP_GET, rw_data, nullptr}, {"/set", HTTP_GET, rw_set, nullptr},
    {"/sw", HTTP_GET, rw_sw, nullptr}, {"/healthz", HTTP_GET, rw_health, nullptr}, {"/room", HTTP_GET, rw_room_h, nullptr},
    {"/poly", HTTP_GET, rw_poly_h, nullptr},   // v1.11
  };
  int ok = 0; for (const auto &u : uris) if (httpd_register_uri_handler(srv, &u) == ESP_OK) ok++;
  rw_page = ok == 7 ? 1 : 2;
  ESP_LOGI("radar_web", "radar map at http://<device-ip>:8080/");
}
