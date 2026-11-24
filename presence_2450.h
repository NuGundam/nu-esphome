#pragma once
// ─────────────────────────────────────────────────
//  NU-Presence-24GHz-2  still presence for the LD2450  v1.6  (presence_2450.h)  2026-11-23
//  v1.6: lock-on. A new track only counts (Occupied, zones, holds) once it has TRAVELLED PR_LOCK_MM from where it
//        started (smoothed position, so radar jitter does not count). Until then it is "checking". A checking track
//        that is lost, or has not travelled within PR_LOCK_WIN_MS, is dropped. Made for Nu-MasterBath-1: a towel
//        swaying in the vent draught sat at one spot (+-0.1 m) with one 80 cm/s flicker a minute, and each flicker
//        started a "person" that then got Main's 30 min Still hold. People walk in, so they always travel first.
//        Each checking track that times out marks a still-clutter spot (pr_cl, 4 spots, forgotten after 1 h). From
//        the second time at a spot: a lost track is not fed by a target sitting there, and presence_move_ok() tells
//        Moving target to ignore a flicker there (unless a locked track is next to it).
//  v1.5: polygon zones (presence_polys, 3-8 corners in sensor mm, set by radar_web_2450.h v1.11). A polygon is used
//        only while the zone rectangle (z1x1..) is its bounding box; change the rectangle anywhere and the zone is a
//        rectangle again. The radar chip itself only filters by the rectangle. Smallest zone = smallest real area.
//  v1.4: Still hold per zone (presence_zone_holds, seconds, 0 = off): a track that goes still inside a zone with its
//        own hold is kept that long instead of the general Still hold (where zones overlap the SMALLEST one decides,
//        so a closet drawn inside a big main zone keeps its own short hold). Made for
//        bathrooms (Nu-MasterBath-1): a soak in the tub or a shower behind glass is out of the radar's sight for a long
//        time, a closet is not. No zone holds set = exactly the v1.3 behaviour.
//
//  The LD2450 tracks motion and has no micro-motion (breathing) detection, so it behaves badly in two ways:
//    * a person sitting still fades out: lost at 2.42 m, back at 2.32 m 89 s later, with nobody having moved
//    * a static ghost can sit in a slot for half an hour (2026-11-06: ~4.4 m for 28 min, zero speed, empty room)
//  So targets are not trusted directly. The rule is that people do not vanish: to leave, they have to move.
//
//    start   a track starts only from a target MOVING at Moving speed or faster inside Moving range.
//            A target that never moved is ignored, which is what drops the ghosts.
//    follow  a target within PR_RADIUS_MM of a track keeps it fresh, moving or not. One target per track per frame,
//            closest pairs first, and tracks seen in the last PR_FRESH_MS choose before lost ones: someone walking
//            across a held spot stays on their own track and leaves the held one alone (v1.1). A target with no
//            track of its own still picks up a lost one, which is how a seated person getting up is followed out.
//    lost    moved within PR_EXIT_MS of the last sighting = it walked out: clear after Clear delay.
//            otherwise it faded out while still: keep it for Still hold, and a target back at the same
//            spot picks the track up again and the wait starts over.
//    full    all 3 in use and a new person moves: a walked-out track goes first, then the one lost longest (v1.1).
//    silent  live = false (no radar bytes for PR_SILENT_MS): the driver's sensors keep their last values, so they
//            are not read at all and tracks age out normally (v1.1; before, a dead radar held Occupied on for ever).
//    reboot  tracks are copied to RTC memory every tick and put back at boot with their countdowns where they were
//            (v1.1). Survives Restart, OTA, crashes; a power cut clears it (magic + checksum fail) and nobody is
//            tracked until they move. A new build can move the block, which just reads as no snapshot.
//
//    ignore  (v1.2) a target whose distance from the sensor is inside Ignore from .. Ignore to is not read at all: it
//            cannot start, follow or keep a track. Office 1 has a static ghost at 3.8-5.0 m that held Occupied for
//            hours on 11-06, 11-07 and 11-08 (4,051 readings in the 4.5 m band with no office motion); a held track
//            fed by it then never aged out. Both 0 = off.
//
//    room    (v1.3) the tracker gets the room plan (presence_room) and the zones (presence_zones) from the page / yaml:
//            fence  a target outside the drawn room (+0.3 m) cannot start or feed a track (hallway, stairs, through walls)
//            exit   'walked out' now also needs the track to be lost AT AN EXIT: within 0.7 m of the edge of what the
//                   radar can see (room wall or opening, edge of the 120 deg view, the range, the edge of a Detection zone,
//                   a Filter zone) while heading towards it. Lost in the open floor = people do not vanish there: it is
//                   held as still however recently it moved (sat down and faded within 5 s used to clear in 30 s).
//            zones  a track leaves a zone only 0.3 m past its edge (someone sitting on the line no longer flickers it).
//
//  Up to 3 tracks, one per person, so someone walking out does not clear someone still sitting there.
//  Occupied = any track alive; Zone N = any track inside the zone.
//  No ESPHome types in here: presence_tick() runs on the main loop, the httpd task only reads pr_t for the page.
// ─────────────────────────────────────────────────
#include <cmath>
#include <cstdint>
#include <esp_attr.h>

struct PrTrack { bool used = false; float x = 0, y = 0; uint32_t seen = 0, moved = 0, limit = 0;
                 float ax = 0, ay = 0; uint32_t born = 0; bool exit = true; uint8_t inz = 0;
                 bool lock = true; float bx = 0, by = 0, fx = 0, fy = 0; };   // v1.6: travelled far enough, start point, fast copy of x / y   // v1.3: slow copy of x / y, start, at an exit, zone bits
static PrTrack pr_t[3];
static constexpr float PR_RADIUS_MM = 1000.0f;   // a seated person wanders about +-0.3 m on the radar
static constexpr uint32_t PR_EXIT_MS = 5000;     // walk-outs moved 0 s before the loss, seated fades 37 s or more
static constexpr uint32_t PR_FRESH_MS = 2000;    // seen this recently = still being followed
static constexpr uint32_t PR_SILENT_MS = 2000;   // radar frames come ~10 a second; a restart is a few seconds of nothing
// v1.6 lock-on and still clutter
static float pr_lock_mm = 500.0f;                 // travel needed before a new track counts; towel jitter was +-100 mm, 0 = off
static constexpr uint32_t PR_LOCK_WIN_MS = 10000; // a checking track that has not travelled by then is not a person
static constexpr float PR_CLUT_MM = 500.0f;       // a spot is this big
static constexpr uint32_t PR_CLUT_TTL_MS = 3600000;
struct PrClut { float x = 0, y = 0; uint8_t n = 0; uint32_t last = 0; };
static PrClut pr_cl[4];
static void presence_lock(float mm) { pr_lock_mm = std::isnan(mm) || mm < 0 ? 500.0f : mm; }
static void pr_clut_add(float x, float y, uint32_t now) {
  int best = 0;
  for (int i = 0; i < 4; i++) {
    PrClut &c = pr_cl[i];
    if (c.n && hypotf(c.x - x, c.y - y) < PR_CLUT_MM) { c.x += 0.3f * (x - c.x); c.y += 0.3f * (y - c.y); if (c.n < 250) c.n++; c.last = now; return; }
    if (!pr_cl[best].n) continue;
    if (!c.n || c.last < pr_cl[best].last) best = i;
  }
  pr_cl[best].x = x; pr_cl[best].y = y; pr_cl[best].n = 1; pr_cl[best].last = now;
}
static bool pr_clut_at(float x, float y) {   // a known still-clutter spot (seen twice or more)
  for (const auto &c : pr_cl) if (c.n >= 2 && hypotf(c.x - x, c.y - y) < PR_CLUT_MM) return true;
  return false;
}


// v1.3: what the tracker knows about the room and the zones (set every tick from the yaml)
struct PrGeo { bool room = false, fence = true; float w = 0, d = 0, sx = 0, sy = 0, co = 1, si = 0, fl = 1, ymax = 6000; int zt = 0; float z[3][4] = {}; };
static PrGeo pr_geo;
static uint32_t pr_zhold[3] = {0, 0, 0};   // v1.4: Still hold per zone in ms, 0 = use the general Still hold
static void presence_zone_holds(float s1, float s2, float s3) {
  const float s[3] = {s1, s2, s3};
  for (int i = 0; i < 3; i++) pr_zhold[i] = (std::isnan(s[i]) || s[i] <= 0.0f) ? 0 : (uint32_t) (s[i] * 1000.0f);
}
static constexpr float PR_FENCE_MM = 300.0f;     // a target this far outside the drawn room still counts (walls, radar noise)
static constexpr float PR_EDGE_MM = 700.0f;      // lost within this of the edge of the visible area = at an exit
static constexpr float PR_OUT_MM = 300.0f;       // ... and it moved at least this far towards that edge in the last ~1.5 s (a seated wander is under that)
static constexpr float PR_ZONE_HYS_MM = 300.0f;  // a track leaves a zone only this far past its edge

// room plan in cm (top-down, origin top-left, y down), sensor at sx / sy aiming aim_deg clockwise from up
static void presence_room(bool on, bool fence, float w_cm, float d_cm, float sx_cm, float sy_cm, float aim_deg, bool flip) {
  pr_geo.room = on && w_cm > 50 && d_cm > 50; pr_geo.fence = fence;
  pr_geo.w = w_cm * 10; pr_geo.d = d_cm * 10; pr_geo.sx = sx_cm * 10; pr_geo.sy = sy_cm * 10;
  pr_geo.co = cosf(aim_deg * 0.0174532925f); pr_geo.si = sinf(aim_deg * 0.0174532925f); pr_geo.fl = flip ? -1.0f : 1.0f;
}
// zone type 0 Disabled / 1 Detection / 2 Filter, rectangles in sensor mm (all four 0 = off)
static void presence_zones(int zt, const float z[3][4]) {
  pr_geo.zt = zt;
  for (int i = 0; i < 3; i++) for (int j = 0; j < 4; j++) pr_geo.z[i][j] = std::isnan(z[i][j]) ? 0.0f : z[i][j];
}
// v1.5: polygon zones
static int pr_pn[3] = {0, 0, 0};
static float pr_pp[3][8][2];
static void presence_polys(int k, int n, const float p[][2]) {
  if (k < 0 || k > 2) return;
  if (n < 3) n = 0; if (n > 8) n = 8;
  for (int i = 0; i < n; i++) { pr_pp[k][i][0] = p[i][0]; pr_pp[k][i][1] = p[i][1]; }
  pr_pn[k] = n;
}
static bool pr_poly_ok(int k, const float *z) {   // polygon set and the rectangle is its bounding box
  const int n = pr_pn[k]; if (n < 3) return false;
  float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
  for (int i = 0; i < n; i++) { x0 = fminf(x0, pr_pp[k][i][0]); x1 = fmaxf(x1, pr_pp[k][i][0]); y0 = fminf(y0, pr_pp[k][i][1]); y1 = fmaxf(y1, pr_pp[k][i][1]); }
  return fabsf(x0 - fminf(z[0], z[2])) <= 15 && fabsf(x1 - fmaxf(z[0], z[2])) <= 15 && fabsf(y0 - fminf(z[1], z[3])) <= 15 && fabsf(y1 - fmaxf(z[1], z[3])) <= 15;
}
static bool pr_poly_in(int k, float x, float y, float tol) {   // inside, or within tol of an edge
  const int n = pr_pn[k]; const float (*p)[2] = pr_pp[k]; bool in = false;
  for (int i = 0, j = n - 1; i < n; j = i++)
    if (((p[i][1] > y) != (p[j][1] > y)) && x < (p[j][0] - p[i][0]) * (y - p[i][1]) / (p[j][1] - p[i][1]) + p[i][0]) in = !in;
  if (in || tol <= 0) return in;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const float ex = p[i][0] - p[j][0], ey = p[i][1] - p[j][1], l2 = ex * ex + ey * ey;
    float u = l2 > 0 ? ((x - p[j][0]) * ex + (y - p[j][1]) * ey) / l2 : 0; u = u < 0 ? 0 : (u > 1 ? 1 : u);
    if (hypotf(x - (p[j][0] + u * ex), y - (p[j][1] + u * ey)) <= tol) return true;
  }
  return false;
}
static float pr_zone_area(int k) {
  const float *z = pr_geo.z[k];
  if (pr_poly_ok(k, z)) { float a = 0; const int n = pr_pn[k]; for (int i = 0, j = n - 1; i < n; j = i++) a += pr_pp[k][j][0] * pr_pp[k][i][1] - pr_pp[k][i][0] * pr_pp[k][j][1]; return fabsf(a) / 2; }
  return fabsf((z[2] - z[0]) * (z[3] - z[1]));
}
static bool pr_zone_on(int i) { const float *z = pr_geo.z[i]; return !(z[0] == 0 && z[1] == 0 && z[2] == 0 && z[3] == 0); }
static bool pr_in_zone(int i, float x, float y, float tol) {
  const float *z = pr_geo.z[i];
  if (pr_poly_ok(i, z)) return pr_poly_in(i, x, y, tol);   // v1.5
  return x >= fminf(z[0], z[2]) - tol && x <= fmaxf(z[0], z[2]) + tol && y >= fminf(z[1], z[3]) - tol && y <= fmaxf(z[1], z[3]) + tol;
}
static bool pr_in_room(float x, float y, float tol) {
  if (!pr_geo.room) return true;
  const float rx = pr_geo.sx + pr_geo.fl * x * pr_geo.co + y * pr_geo.si, ry = pr_geo.sy + pr_geo.fl * x * pr_geo.si - y * pr_geo.co;
  return rx >= -tol && rx <= pr_geo.w + tol && ry >= -tol && ry <= pr_geo.d + tol;
}
// can the radar report a person standing here?
static bool pr_visible(float x, float y) {
  if (y <= 0.0f || y > pr_geo.ymax || hypotf(x, y) > 6000.0f || fabsf(x) > y * 1.7320508f) return false;   // range, 120 deg view
  if (pr_geo.fence && !pr_in_room(x, y, 0.0f)) return false;
  if (pr_geo.zt == 1) { bool any = false, in = false; for (int i = 0; i < 3; i++) if (pr_zone_on(i)) { any = true; if (pr_in_zone(i, x, y, 0)) in = true; } if (any && !in) return false; }
  if (pr_geo.zt == 2) for (int i = 0; i < 3; i++) if (pr_zone_on(i) && pr_in_zone(i, x, y, 0)) return false;
  return true;
}

// reboot snapshot, all 32-bit so there is no padding in the checksum
struct PrSave { uint32_t magic, used[3]; float x[3], y[3]; uint32_t age[3], gap[3], sum; };
static RTC_NOINIT_ATTR PrSave pr_save;
static constexpr uint32_t PR_MAGIC = 0x50523136;   // v1.6: new magic, so a snapshot of a pre-lock-on (towel) track is not put back
static int pr_restored = -1;                     // for the page: -1 no snapshot at boot, else tracks put back

static uint32_t pr_sum(const PrSave &p) {
  const uint8_t *b = (const uint8_t *) &p; uint32_t h = 2166136261u;
  for (size_t i = 0; i < sizeof(PrSave) - sizeof(uint32_t); i++) { h ^= b[i]; h *= 16777619u; }
  return h;
}

static void presence_save(uint32_t now) {
  pr_save.magic = PR_MAGIC;
  for (int k = 0; k < 3; k++) {
    const PrTrack &t = pr_t[k];
    pr_save.used[k] = t.used && t.lock; pr_save.x[k] = t.x;   // v1.6: checking tracks are not kept pr_save.y[k] = t.y;
    pr_save.age[k] = now - t.seen; pr_save.gap[k] = t.seen - t.moved;
  }
  pr_save.sum = pr_sum(pr_save);
}

// call once at boot, before the first tick
static void presence_restore(uint32_t now) {
  if (pr_save.magic != PR_MAGIC || pr_save.sum != pr_sum(pr_save)) return;
  pr_restored = 0;
  for (int k = 0; k < 3; k++) {
    if (pr_save.used[k] != 1) continue;
    PrTrack &t = pr_t[k];
    t.used = true; t.x = pr_save.x[k]; t.y = pr_save.y[k];
    t.seen = now - pr_save.age[k]; t.moved = t.seen - pr_save.gap[k];
    t.ax = t.x; t.ay = t.y; t.born = t.seen - 60000; t.exit = true; t.inz = 0;
    t.lock = true; t.bx = t.fx = t.x; t.by = t.fy = t.y;
    pr_restored++;
  }
}


// v1.3: is this track at an exit? Near the edge of the visible area and (once it is 1.5 s old) heading towards it.
static bool pr_at_exit(const PrTrack &t, uint32_t now) {
  if ((!pr_geo.room || !pr_geo.fence) && pr_geo.zt == 0) return true;   // no room plan / fence off, no zones: the old rule (timing only)
  static const float DX[8] = {1, 0.7071f, 0, -0.7071f, -1, -0.7071f, 0, 0.7071f}, DY[8] = {0, 0.7071f, 1, 0.7071f, 0, -0.7071f, -1, -0.7071f};
  const bool young = now - t.born < 1500;
  for (int i = 0; i < 8; i++) {
    if (pr_visible(t.x + DX[i] * PR_EDGE_MM, t.y + DY[i] * PR_EDGE_MM)) continue;
    if (young || DX[i] * (t.x - t.ax) + DY[i] * (t.y - t.ay) >= PR_OUT_MM) return true;
  }
  return false;
}
static bool pr_out(const PrTrack &t) { return t.exit && t.seen - t.moved < PR_EXIT_MS; }   // walked out (clear delay) vs still (hold)

// a free track, else a walked-out one, else the one lost longest; never one a target already took this frame
static int pr_victim(const bool *taken, uint32_t now) {
  for (int k = 0; k < 3; k++) if (!pr_t[k].used) return k;
  for (int k = 0; k < 3; k++) if (!taken[k] && !pr_t[k].lock) return k;   // v1.6: a checking track goes first
  int best = -1; bool bout = false; uint32_t bage = 0;
  for (int k = 0; k < 3; k++) {
    if (taken[k]) continue;
    uint32_t age = now - pr_t[k].seen; bool out = pr_out(pr_t[k]);
    if (best < 0 || (out && !bout) || (out == bout && age > bage)) { best = k; bout = out; bage = age; }
  }
  return best;
}

// x, y in mm and s in cm/s for the 3 radar slots, NaN when empty; ymax in mm; live = the radar is sending
static void presence_tick(const float *x, const float *y, const float *s, bool live, float ymax, float smin,
                          uint32_t clear_ms, uint32_t hold_ms, uint32_t now, float ign_lo = 0.0f, float ign_hi = 0.0f) {
  bool ok[3], moving[3], done[3] = {false, false, false}, taken[3] = {false, false, false};
  pr_geo.ymax = ymax;
  for (int i = 0; i < 3; i++) {
    ok[i] = live && !std::isnan(x[i]) && !std::isnan(y[i]) && y[i] > 0.0f && y[i] <= ymax;
    if (ok[i] && pr_geo.fence && !pr_in_room(x[i], y[i], PR_FENCE_MM)) ok[i] = false;   // v1.3 room fence
    if (ok[i] && ign_hi > ign_lo) { float r = hypotf(x[i], y[i]); if (r >= ign_lo && r <= ign_hi) ok[i] = false; }   // v1.2
    moving[i] = !std::isnan(s[i]) && fabsf(s[i]) >= smin;
  }
  for (int pass = 0; pass < 2; pass++) {         // 0 = tracks being followed, 1 = lost tracks
    for (;;) {
      int bi = -1, bk = -1; float bd = PR_RADIUS_MM;
      for (int i = 0; i < 3; i++) {
        if (!ok[i] || done[i]) continue;
        for (int k = 0; k < 3; k++) {
          if (!pr_t[k].used || taken[k] || (now - pr_t[k].seen < PR_FRESH_MS) != (pass == 0)) continue;
          if (pass == 1 && !moving[i] && pr_clut_at(x[i], y[i])) continue;   // v1.6: still clutter does not feed a lost track
          float d = hypotf(pr_t[k].x - x[i], pr_t[k].y - y[i]);
          if (d < bd) { bd = d; bi = i; bk = k; }
        }
      }
      if (bi < 0) break;
      done[bi] = true; taken[bk] = true;
      pr_t[bk].ax += 0.07f * (pr_t[bk].x - pr_t[bk].ax); pr_t[bk].ay += 0.07f * (pr_t[bk].y - pr_t[bk].ay);   // where it was ~1.5 s ago
      pr_t[bk].x = x[bi]; pr_t[bk].y = y[bi]; pr_t[bk].seen = now;
      pr_t[bk].fx += 0.3f * (x[bi] - pr_t[bk].fx); pr_t[bk].fy += 0.3f * (y[bi] - pr_t[bk].fy);   // v1.6
      if (!pr_t[bk].lock && hypotf(pr_t[bk].fx - pr_t[bk].bx, pr_t[bk].fy - pr_t[bk].by) >= pr_lock_mm) pr_t[bk].lock = true;
      if (moving[bi]) pr_t[bk].moved = now;
      pr_t[bk].exit = pr_at_exit(pr_t[bk], now);
    }
  }
  for (int i = 0; i < 3; i++) {
    if (!ok[i] || done[i] || !moving[i]) continue;   // never moved: not a person yet
    bool body = false;                               // a second return off someone already followed this frame
    for (int k = 0; k < 3; k++)
      if (pr_t[k].used && taken[k] && hypotf(pr_t[k].x - x[i], pr_t[k].y - y[i]) < PR_RADIUS_MM) body = true;
    if (body) continue;
    int k = pr_victim(taken, now);
    if (k < 0) continue;
    pr_t[k].used = true; pr_t[k].x = x[i]; pr_t[k].y = y[i]; pr_t[k].seen = now; pr_t[k].moved = now;
    pr_t[k].ax = x[i]; pr_t[k].ay = y[i]; pr_t[k].born = now; pr_t[k].inz = 0; pr_t[k].exit = pr_at_exit(pr_t[k], now);
    pr_t[k].lock = pr_lock_mm <= 0; pr_t[k].bx = pr_t[k].fx = x[i]; pr_t[k].by = pr_t[k].fy = y[i];   // v1.6: checking until it travels
    taken[k] = true;
  }
  for (auto &t : pr_t) {
    if (!t.used) continue;
    if (!t.lock) {   // v1.6: checking - lost = gone; stood at one spot the whole window = still clutter, gone
      t.limit = PR_FRESH_MS;
      if (now - t.seen >= PR_FRESH_MS) t.used = false;
      else if (now - t.born >= PR_LOCK_WIN_MS) { pr_clut_add(t.fx, t.fy, now); t.used = false; }
      continue;
    }
    uint32_t h = 0; float ha = 0;   // v1.4: the smallest zone with a hold that it went still in decides, else the general Still hold
    for (int k = 0; k < 3; k++) {
      if (!pr_zhold[k] || !pr_zone_on(k) || !pr_in_zone(k, t.x, t.y, PR_ZONE_HYS_MM)) continue;
      const float a = pr_zone_area(k);   // v1.5: real area, polygon or rectangle
      if (!h || a < ha) { h = pr_zhold[k]; ha = a; }
    }
    t.limit = pr_out(t) ? clear_ms : (h ? h : hold_ms);
    if (now - t.seen >= t.limit) t.used = false;
  }
  for (auto &c : pr_cl) if (c.n && now - c.last > PR_CLUT_TTL_MS) c.n = 0;   // v1.6: forget an old spot
  presence_save(now);
}

// v1.6: may a moving target at x / y switch Moving target on? Not a flicker at a known still-clutter spot, unless a
// locked track (a real person) is right there.
static bool presence_move_ok(float x, float y) {
  if (std::isnan(x) || !pr_clut_at(x, y)) return true;
  for (const auto &t : pr_t) if (t.used && t.lock && hypotf(t.x - x, t.y - y) < PR_RADIUS_MM) return true;
  return false;
}

static bool presence_occupied() {
  for (const auto &t : pr_t) if (t.used && t.lock) return true;   // v1.6: checking tracks do not count
  return false;
}

// zone k (0-2): rectangle in mm, corners in any order; all four 0 = zone off.
// v1.3: a track enters inside the rectangle and leaves only PR_ZONE_HYS_MM past its edge.
static bool presence_in_rect(int k, float x1, float y1, float x2, float y2) {
  const uint8_t bit = (uint8_t) (1u << k);
  if (std::isnan(x1) || std::isnan(y1) || std::isnan(x2) || std::isnan(y2) || (x1 == 0 && y1 == 0 && x2 == 0 && y2 == 0)) {
    for (auto &t : pr_t) t.inz &= (uint8_t) ~bit;
    return false;
  }
  float xa = fminf(x1, x2), xb = fmaxf(x1, x2), ya = fminf(y1, y2), yb = fmaxf(y1, y2); bool any = false;
  const float zr[4] = {x1, y1, x2, y2}; const bool poly = pr_poly_ok(k, zr);   // v1.5
  for (auto &t : pr_t) {
    if (!t.used || !t.lock) { t.inz &= (uint8_t) ~bit; continue; }   // v1.6
    const float h = (t.inz & bit) ? PR_ZONE_HYS_MM : 0.0f;
    if (poly ? pr_poly_in(k, t.x, t.y, h) : (t.x >= xa - h && t.x <= xb + h && t.y >= ya - h && t.y <= yb + h)) { t.inz |= bit; any = true; } else t.inz &= (uint8_t) ~bit;
  }
  return any;
}
