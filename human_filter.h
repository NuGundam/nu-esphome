#pragma once
// human_filter.h v1.6 (2026-11-21) - "is it a person?" for an LD2410 radar, for common/human-filter.yaml.
//
// The LD2410 only reports energy per 0.75 m ring (gate): it cannot tell a person from a 3D printer, a car's cooling
// fan or a robot vacuum. This filter works on how the energy BEHAVES instead of fixed thresholds:
//
//   1. Background per ring, learned all the time (mean + spread of the move and the still energy). Anything that
//      stays in its ring - a printer, a fan, the unit's own SEN55 fan - becomes background within a few minutes and
//      that ring's threshold rises by itself (mean + 3 x spread). When the machine stops, the threshold falls back.
//   2. A person ARRIVES = confirmed, in one of two ways. Machines do not travel.
//        fast: MOVE energy above the background travels two rings (1.5 m) within a few seconds (walking);
//        slow: the strongest ring (move or still) steps to the next ring three times in the same direction, each
//              step within 20 s (v1.3: somebody strolling or shuffling shows up as STILL energy drifting from ring
//              to ring - the Family Room test of 2026-11-20 lost such a person for 2.5 minutes).
//      A strong burst that does not travel (move 15 or still 25 over the threshold for most of a second) also turns
//      presence on, but on probation.
//   3. Confirmed: NO ring learns, so somebody standing or sitting still is held by the energy above the background
//      around their ring (a lower bar than for arriving: mean + 2 x spread). Presence ends "Person clear delay"
//      after the last sign near that spot. A person who has not changed ring for "Still person hold" drops to probation.
//   4. Probation: the rings keep learning (3 min time constant), so a machine that just started is tuned out within
//      a couple of minutes, while a person only has to walk on to be confirmed.
//
//   5. v1.4 (a person sitting in the Family Room through a restart was learned as background within the minute):
//      - learn_ok / nobody: rooms without machines (hf_learn_gate) tie the filter to the radar's own Occupied:
//        learning only while Occupied is off (a person the radar sees is never learned as background, however
//        long they sit), and no detection or holding while it is off (presence ends "Person clear delay" after
//        the radar loses its target). Rooms with machines (garage) learn always and ignore Occupied.
//      - the learned background is kept in flash (save()/load(); the yaml stores it every 30 min while clear), so a
//        restart starts from the empty-room levels and is ready in 10 s.
//      - no stored background and learning not allowed at start (somebody is there): start from quiet-room levels
//        (move 5, still 6) instead of what is in view. relearn() = learn the room again for 60 s (button).
//
// Needs Engineering mode (per-ring energies). Called 10 times a second.
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>

struct HumanFilter {
  static const int N = 9;
  float mu_m[N], dv_m[N], mu_s[N], dv_s[N], fm[N], fs[N];
  bool init = false;
  uint32_t t0 = 0, last_ev = 0, anc_t = 0, anc2_t = 0, conf_t = 0, sl_t = 0;
  int state = 0;          // 0 clear, 1 probation, 2 confirmed, -1 learning (boot), -2 no data
  int hg = -1;            // ring the person is in
  int pk = -1, pk_n = 0, anc = -1, anc2 = -1;     // fast travel (move energy)
  int cp = -1, cp_n = 0, sl = -1, sl_dir = 0, sl_chain = 0;   // slow travel (strongest ring, move or still)
  uint16_t strong = 0;
  int learn_left = 0;     // seconds of boot learning left (0 = ready)
  bool learn_blocked = false;   // v1.6: the caller stopped learning this frame (somebody in view), for the page
  int learn_total = 60;   // v1.5: length of this learning run in seconds (10 with a stored background), for the page
  uint32_t skip_mask = 0; // rings never used (bit n = ring n)
  bool have_saved = false, loaded = false;   // v1.4
  struct Saved { uint32_t magic; float mu_m[N], dv_m[N], mu_s[N], dv_s[N]; };
  void save(Saved &o) const { o.magic = 0x48463134; for (int g = 0; g < N; g++) { o.mu_m[g] = mu_m[g]; o.dv_m[g] = dv_m[g]; o.mu_s[g] = mu_s[g]; o.dv_s[g] = dv_s[g]; } }
  void load(const Saved &o) { if (o.magic != 0x48463134) return; for (int g = 0; g < N; g++) { mu_m[g] = o.mu_m[g]; dv_m[g] = o.dv_m[g]; mu_s[g] = o.mu_s[g]; dv_s[g] = o.dv_s[g]; } have_saved = true; }
  void relearn() { init = false; have_saved = false; }

  float thr_m(int g) const { return mu_m[g] + std::fmax(3.0f * dv_m[g], 10.0f); }
  float thr_s(int g) const { return mu_s[g] + std::fmax(3.0f * dv_s[g], 12.0f); }
  float hold_m(int g) const { return mu_m[g] + std::fmax(2.0f * dv_m[g], 6.0f); }
  float hold_s(int g) const { return mu_s[g] + std::fmax(2.0f * dv_s[g], 8.0f); }

  void step(const float *m, const float *s, bool data_ok, bool learn_ok, bool nobody, float hold_min, float clear_s, uint32_t now) {
    if (!data_ok) { state = -2; hg = -1; return; }
    for (int g = 0; g < N; g++) if (std::isnan(m[g]) || std::isnan(s[g])) { state = -2; hg = -1; return; }
    if (!init || state == -2) {
      for (int g = 0; g < N; g++) {
        fm[g] = m[g]; fs[g] = s[g];
        if (have_saved) continue;                                  // start from the stored background
        if (learn_ok) { mu_m[g] = m[g]; mu_s[g] = s[g]; }          // start from what is in view
        else { mu_m[g] = 5.0f; mu_s[g] = 6.0f; }                   // somebody is there: quiet-room levels
        dv_m[g] = dv_s[g] = 3.0f;
      }
      loaded = have_saved;
      init = true; t0 = now; state = -1; hg = -1; pk = anc = anc2 = cp = sl = -1; pk_n = cp_n = sl_chain = sl_dir = 0; strong = 0;
    }
    const uint32_t boot_ms = loaded ? 10000 : 60000;   // with a stored background only the filters need to settle
    const bool boot = now - t0 < boot_ms;
    learn_total = (int) (boot_ms / 1000);
    learn_blocked = !learn_ok;
    learn_left = boot ? (int) ((boot_ms - (now - t0) + 999) / 1000) : 0;
    float em[N], es[N];
    int pm = -1, pc = -1; float best = 0.0f, bests = 0.0f, bestc = 0.0f;
    for (int g = 0; g < N; g++) {
      fm[g] += 0.35f * (m[g] - fm[g]);
      fs[g] += 0.35f * (s[g] - fs[g]);
      if (skip_mask & (1u << g)) { em[g] = es[g] = -100.0f; continue; }
      em[g] = fm[g] - thr_m(g);
      es[g] = fs[g] - thr_s(g);
      if (em[g] > best) { best = em[g]; pm = g; }
      if (es[g] > bests) bests = es[g];
      const float e = std::fmax(em[g], es[g]);
      if (e > bestc) { bestc = e; pc = g; }
    }
    // fast travel: the strongest moving ring settles (3 frames) two rings away within 6 s, or one ring further in the
    // same direction twice (a -> b -> c). One ring back and forth (a printer flickering between its rings) is not travel.
    bool travel = false; int tg = -1;
    if (pm == pk) pk_n++; else { pk = pm; pk_n = 1; }
    if (pm >= 0 && pk_n >= 3) {
      if (pk_n == 3 && anc >= 0 && anc != pm) {
        if (now - anc_t <= 6000) {
          const int d = pm - anc;
          if (std::abs(d) == 2) travel = true;
          else if (std::abs(d) == 1 && anc2 >= 0 && anc - anc2 == d && now - anc2_t <= 9000) travel = true;
          anc2 = anc; anc2_t = anc_t;
        } else {
          anc2 = -1;
        }
        if (travel) tg = pm;
      }
      anc = pm; anc_t = now;
    }
    // slow travel (v1.3): the strongest ring of all (move or still) settles (5 frames) on the NEXT ring three times in
    // a row in the same direction, each step within 20 s. Jumps and direction changes start the count again.
    if (pc == cp) cp_n++; else { cp = pc; cp_n = 1; }
    if (pc >= 0 && cp_n >= 5) {
      if (cp_n == 5 && sl >= 0 && sl != pc) {
        const int d = pc - sl;
        if (std::abs(d) == 1 && now - sl_t <= 20000) { sl_chain = (d == sl_dir) ? sl_chain + 1 : 1; sl_dir = d; }
        else { sl_chain = 0; sl_dir = 0; }
        if (sl_chain >= 3) { travel = true; tg = pc; sl_chain = 0; }
      }
      sl = pc; sl_t = now;
    }
    strong = (uint16_t) (((strong << 1) | ((best >= 15.0f || bests >= 25.0f) ? 1 : 0)) & 0x3FF);
    bool burst = __builtin_popcount(strong) >= 8;
    if (nobody) { travel = false; burst = false; }   // gated room: the radar itself has no target

    if (boot) {
      state = -1;
    } else {
      if (state < 0) state = 0;
      if (state == 0) {
        if (travel) { state = 2; hg = tg; last_ev = now; conf_t = now; }
        else if (burst && pc >= 0) { state = 1; hg = pc; last_ev = now; }
      } else {
        if (travel) { state = 2; hg = tg; last_ev = now; conf_t = now; }
        else if (!nobody && pc >= 0 && cp_n >= 3 && pc != hg && std::abs(pc - hg) <= 2) {   // follow the person
          hg = pc; last_ev = now;
          if (state == 2) conf_t = now;
        }
        for (int g = hg - 1; g <= hg + 1; g++)
          if (!nobody && g >= 0 && g < N && !(skip_mask & (1u << g)) && (fm[g] > hold_m(g) || fs[g] > hold_s(g))) last_ev = now;
        // a confirmed person who has not changed ring for "Still person hold" goes back on probation
        if (state == 2 && now - conf_t > (uint32_t) (std::fmax(hold_min, 1.0f) * 60000.0f)) state = 1;
        if (now - last_ev > (uint32_t) (std::fmax(clear_s, 5.0f) * 1000.0f)) { state = 0; hg = -1; }
      }
    }
    // learn the background (not while the caller forbids it, not during the short start with a stored background)
    const float dt = 0.1f;
    for (int g = 0; g < N && learn_ok && !(boot && loaded); g++) {
      float tau = boot ? 10.0f : 90.0f;
      if (state == 2) continue;        // nothing learns while a person is confirmed
      if (state == 1 && std::abs(g - hg) <= 1) tau = 180.0f;   // probation: tuned out within minutes
      const float a = dt / tau;
      mu_m[g] += a * (m[g] - mu_m[g]); dv_m[g] += a * (std::fabs(m[g] - mu_m[g]) - dv_m[g]);
      mu_s[g] += a * (s[g] - mu_s[g]); dv_s[g] += a * (std::fabs(s[g] - mu_s[g]) - dv_s[g]);
    }
  }

  bool human() const { return state > 0; }
  bool ready() const { return state >= 0; }   // learned the room, answers are valid

  std::string status() const {
    char b[48];
    if (state == -2) return "no data (Engineering mode off)";
    if (state == -1) { snprintf(b, sizeof(b), "learning, ready in %d s", ((learn_left + 4) / 5) * 5); return b; }   // 5 s steps
    if (state == 0) return "clear";
    snprintf(b, sizeof(b), "%s ring %d (%.1f m)", state == 2 ? "person" : "checking", hg, hg * 0.75f + 0.375f);
    return b;
  }

  // learned thresholds, move/still per ring: "0:10/12 1:100/12 ..." (100 = ring tuned out)
  std::string thresholds() const {
    std::string o; char b[20];
    if (!init) return "-";
    for (int g = 0; g < N; g++) {
      snprintf(b, sizeof(b), "%s%d:%d/%d", g ? " " : "", g, (int) std::fmin(100.0f, std::round(thr_m(g))), (int) std::fmin(100.0f, std::round(thr_s(g))));
      o += b;
    }
    return o;
  }
};

static inline HumanFilter &human_filter() { static HumanFilter h; return h; }

static inline void human_filter_relearn() { human_filter().relearn(); }   // v1.5: Relearn room button on the :8080 page

// for the :8080 tuning page (radar_web.h): {"st":2,"hg":4,"lrn":0,"txt":"person ring 4 (3.4 m)","tm":[..9],"ts":[..9]}
static inline void human_filter_json(char *out, size_t n) {
  auto &h = human_filter();
  int k = snprintf(out, n, "{\"st\":%d,\"hg\":%d,\"lrn\":%d,\"tot\":%d,\"blk\":%d,\"txt\":\"%s\",\"tm\":[", h.state, h.hg, h.state == -1 ? h.learn_left : 0, h.learn_total, h.learn_blocked ? 1 : 0, h.status().c_str());
  for (int g = 0; g < HumanFilter::N && k < (int) n; g++)
    k += snprintf(out + k, n - k, "%s%d", g ? "," : "", h.init ? (int) std::fmin(100.0f, std::round(h.thr_m(g))) : 0);
  if (k < (int) n) k += snprintf(out + k, n - k, "],\"ts\":[");
  for (int g = 0; g < HumanFilter::N && k < (int) n; g++)
    k += snprintf(out + k, n - k, "%s%d", g ? "," : "", h.init ? (int) std::fmin(100.0f, std::round(h.thr_s(g))) : 0);
  if (k < (int) n) snprintf(out + k, n - k, "]}");
}
