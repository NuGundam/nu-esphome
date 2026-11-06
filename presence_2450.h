#pragma once
// ─────────────────────────────────────────────────
//  NU-Presence-24GHz-2  still presence for the LD2450  v1.1  (presence_2450.h)  2026-11-06
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
//  Up to 3 tracks, one per person, so someone walking out does not clear someone still sitting there.
//  Occupied = any track alive; Zone N = any track inside the zone.
//  No ESPHome types in here: presence_tick() runs on the main loop, the httpd task only reads pr_t for the page.
// ─────────────────────────────────────────────────
#include <cmath>
#include <cstdint>
#include <esp_attr.h>

struct PrTrack { bool used = false; float x = 0, y = 0; uint32_t seen = 0, moved = 0, limit = 0; };
static PrTrack pr_t[3];
static constexpr float PR_RADIUS_MM = 1000.0f;   // a seated person wanders about +-0.3 m on the radar
static constexpr uint32_t PR_EXIT_MS = 5000;     // walk-outs moved 0 s before the loss, seated fades 37 s or more
static constexpr uint32_t PR_FRESH_MS = 2000;    // seen this recently = still being followed
static constexpr uint32_t PR_SILENT_MS = 2000;   // radar frames come ~10 a second; a restart is a few seconds of nothing

// reboot snapshot, all 32-bit so there is no padding in the checksum
struct PrSave { uint32_t magic, used[3]; float x[3], y[3]; uint32_t age[3], gap[3], sum; };
static RTC_NOINIT_ATTR PrSave pr_save;
static constexpr uint32_t PR_MAGIC = 0x50523131;
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
    pr_save.used[k] = t.used; pr_save.x[k] = t.x; pr_save.y[k] = t.y;
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
    pr_restored++;
  }
}

// a free track, else a walked-out one, else the one lost longest; never one a target already took this frame
static int pr_victim(const bool *taken, uint32_t now) {
  for (int k = 0; k < 3; k++) if (!pr_t[k].used) return k;
  int best = -1; bool bout = false; uint32_t bage = 0;
  for (int k = 0; k < 3; k++) {
    if (taken[k]) continue;
    uint32_t age = now - pr_t[k].seen; bool out = pr_t[k].seen - pr_t[k].moved < PR_EXIT_MS;
    if (best < 0 || (out && !bout) || (out == bout && age > bage)) { best = k; bout = out; bage = age; }
  }
  return best;
}

// x, y in mm and s in cm/s for the 3 radar slots, NaN when empty; ymax in mm; live = the radar is sending
static void presence_tick(const float *x, const float *y, const float *s, bool live, float ymax, float smin,
                          uint32_t clear_ms, uint32_t hold_ms, uint32_t now) {
  bool ok[3], moving[3], done[3] = {false, false, false}, taken[3] = {false, false, false};
  for (int i = 0; i < 3; i++) {
    ok[i] = live && !std::isnan(x[i]) && !std::isnan(y[i]) && y[i] > 0.0f && y[i] <= ymax;
    moving[i] = !std::isnan(s[i]) && fabsf(s[i]) >= smin;
  }
  for (int pass = 0; pass < 2; pass++) {         // 0 = tracks being followed, 1 = lost tracks
    for (;;) {
      int bi = -1, bk = -1; float bd = PR_RADIUS_MM;
      for (int i = 0; i < 3; i++) {
        if (!ok[i] || done[i]) continue;
        for (int k = 0; k < 3; k++) {
          if (!pr_t[k].used || taken[k] || (now - pr_t[k].seen < PR_FRESH_MS) != (pass == 0)) continue;
          float d = hypotf(pr_t[k].x - x[i], pr_t[k].y - y[i]);
          if (d < bd) { bd = d; bi = i; bk = k; }
        }
      }
      if (bi < 0) break;
      done[bi] = true; taken[bk] = true;
      pr_t[bk].x = x[bi]; pr_t[bk].y = y[bi]; pr_t[bk].seen = now;
      if (moving[bi]) pr_t[bk].moved = now;
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
    taken[k] = true;
  }
  for (auto &t : pr_t) {
    if (!t.used) continue;
    t.limit = (t.seen - t.moved < PR_EXIT_MS) ? clear_ms : hold_ms;
    if (now - t.seen >= t.limit) t.used = false;
  }
  presence_save(now);
}

static bool presence_occupied() {
  for (const auto &t : pr_t) if (t.used) return true;
  return false;
}

// rectangle in mm, corners in any order; all four 0 = zone off
static bool presence_in_rect(float x1, float y1, float x2, float y2) {
  if (std::isnan(x1) || std::isnan(y1) || std::isnan(x2) || std::isnan(y2)) return false;
  if (x1 == 0 && y1 == 0 && x2 == 0 && y2 == 0) return false;
  float xa = fminf(x1, x2), xb = fmaxf(x1, x2), ya = fminf(y1, y2), yb = fmaxf(y1, y2);
  for (const auto &t : pr_t) if (t.used && t.x >= xa && t.x <= xb && t.y >= ya && t.y <= yb) return true;
  return false;
}
