# nu-esphome

[![last commit](https://img.shields.io/github/last-commit/NuGundam/nu-esphome?label=last%20commit&color=6fb0ff)](https://github.com/NuGundam/nu-esphome/commits)
[![licence](https://img.shields.io/github/license/NuGundam/nu-esphome?label=licence&color=6fb0ff)](LICENSE)
[![esphome](https://img.shields.io/badge/ESPHome-2026.10-6fb0ff)](https://esphome.io)

**The ESPHome firmware behind one Home Assistant house: 21 devices for air quality, HVAC zoning, presence and more.**

Everything here is a real config from a working house, not an example. Most of these started
from stock firmware or a sensor on a breadboard and grew whatever the house turned out to need:
radar logic that copes with people sitting still, air quality scores the EPA would recognise,
and failsafes for when Home Assistant goes away.

---

## The devices

| Device | What it does | Hardware | Where |
|---|---|---|---|
| [`main-zone-controller`](#main-zone-controller) | Drives 15 HVAC dampers | ESP32, 4x PCF8574 | Attic |
| [`nu-esp32-s3-1`](#nu-esp32-s3-1-fresh-air-aqi) | Fresh-air intake air quality and duct fan | ESP32-S3, SEN5x, AC Infinity T8 | Attic |
| [`nu-aqi-1`, `2`, `4`, `5`, `6`](#nu-aqi-1-2-4-5-6) | Room air quality | ESP32-S3, SHT40, SGP41, SCD40 | Loft, Office 3, Bedroom 2, Master Bedroom, Office 2 |
| [`nu-aqi-3`](#nu-aqi-3) | Room air quality with a web dashboard | Arduino Nano ESP32, SEN66 | Office 1 |
| [`airgradient-loft`, `airgradient-kitchen`](#airgradient-loft-airgradient-kitchen) | AirGradient ONE on local firmware | AirGradient I-9PSL | Loft, Kitchen |
| [`nu-presence-1`, `2`](#nu-presence-1-2) | 60 GHz presence, breathing and light, with a web page | Seeed MR60BHA2 kit | Loft, not installed yet |
| [`nu-presence-24ghz-1`](#nu-presence-24ghz-1) | 24 GHz presence with its own screen and a tuning page | Waveshare ESP32-S3-LCD-1.47B, LD2410C | Family Room |
| [`nu-presence-24ghz-2`](#nu-presence-24ghz-2) | 24 GHz multi-person tracking with a room map | Apollo MTR-1, LD2450 | Upper Hallway |
| [`nu-trash-1`](#nu-trash-1) | Odor watch inside the kitchen trash cabinet | ESP32-S3 Super Mini, SGP41, BME688, AGS10 | Kitchen |
| [`nu-garage-1`](#nu-garage-1) | Garage air quality and 24 GHz presence | ESP32-S3 Super Mini, SEN55, LD2410C | Garage |
| [`nu-range-hood`](#nu-range-hood) | Three-speed kitchen hood | ESP32-S3, relays | Kitchen |
| [`nu-esp32-2432s028r`](#nu-esp32-2432s028r) | Wall touch panel for a light | ESP32-2432S028R display | Office 1 |
| [`nu-thread-1`](#nu-thread-1) | Soil moisture over Thread | ESP32-H2 | Dining Room |
| [`nu-wt-1`](#nu-wt-1) | Water pump on a relay | ESP32-C6 | Dining Room |
| [`nu-test-1`](#nu-test-1) | Bench test board | Arduino Nano ESP32, SHT4x | Closet |

---

## HVAC

### main-zone-controller

Opens and closes 15 motorised dampers, one per room plus fresh air, so Home Assistant can send
heating and cooling only where it's needed.

- Each damper is a Home Assistant `valve`. The dampers have no position feedback, so a timer
  tracks where each one should be, and any position can be requested.
- Relays run through two PCF8574 output expanders. Two more read the zone board's inputs.
- A 1-Wire sensor measures main duct temperature.
- **Failsafe:** if Home Assistant is gone for 30 minutes, the zone dampers open and fresh air
  closes, so the system keeps working on its own.
- The 15 valve blocks are generated. The Python generator is in a comment at the bottom of the file.

### nu-esp32-s3-1 (fresh-air-aqi)

Sits on the fresh-air intake. It measures what's coming in from outside and runs the fan that
pulls it.

- A Sensirion SEN5x reads PM, VOC, NOx, temperature and humidity behind the intake filter.
- Runs an AC Infinity CLOUDLINE PRO T8 duct fan through a Controller 69 Pro over Bluetooth, with
  no cloud. The protocol is in [`aci.h`](aci.h), ported from
  [hunterjm/ac-infinity-ble](https://github.com/hunterjm/ac-infinity-ble).
- Reads the fan controller's duct probe. If no data arrives for 60 seconds, the probe goes
  unavailable, so nothing acts on a frozen reading.
- **Failsafe:** if Home Assistant is gone for 30 minutes, the fan turns off. Otherwise it would hold
  its last speed and keep pulling in outdoor air with nothing watching humidity or air quality.

---

## Air quality

### nu-aqi-1, 2, 4, 5, 6

The room sensor. One shared package, [`common/nuaqi-common.yaml`](common/nuaqi-common.yaml), holds
all the logic, and each device file is a few lines of names and keys.

- SHT40 temperature and humidity, SGP41 VOC and NOx, SCD40 CO2.
- Two VOC sensors: "VOC Index" is smoothed for trends (one value per 100 s), "VOC Fast" is the
  same index before smoothing (read every 10 s), so a short event shows up quickly and you can
  see which room moved first.
- A single combined air quality level drives a colour LED, so you can see the room at a glance.
- The LED dims at night, based on sunset and sunrise.
- Temperature is corrected for the heat the board itself gives off, and the correction follows
  how hot the chip is running.
- Feels-like temperature, absolute humidity, and a CO2 recalibration button.

### nu-aqi-3

The bigger sibling, built around a Sensirion SEN66 with PM, CO2, VOC, NOx, temperature and humidity
in one sensor.

- The PM score is the real EPA NowCast: a 12-hour weighted average of PM2.5 and PM10 on the
  May 2024 breakpoints.
- Humidity is recalculated after the temperature correction, so absolute humidity stays right.
- Runs its own web dashboard on port 8080 ([`custom_web.h`](custom_web.h)) instead of the ESPHome
  `web_server`: every entity of the unit on one dark page that also fits a phone.
- The main loop builds the page's JSON every 2 s and the web task only hands it out, so no state
  is read across threads. The unit keeps 2 h of one-minute history, which gives the page
  sparklines, a 10-minute change and a 2 h low/high as soon as it opens.
- The page is 18 KB, is sent with an ETag (a reload costs a 304), polls every 4 s and stops while
  the tab is hidden.
- The page's Settings panel can change the two switches, the status light and the two
  temperature-offset numbers. It opens locked and locks again 2 min after the last change. The
  web task only queues a change (`POST /set`, which needs a custom header so another website
  cannot send one); the main loop performs it and values are clamped to each number's limits.
- The same panel has a Restart button: it needs Unlock and a second tap within 5 s. The unit
  answers the page first and restarts 1.5 s later; the page reconnects by itself.
- Tuned for heat: 80 MHz CPU and low Wi-Fi power, because a warm board reads the air wrong.
- Two VOC sensors: "VOC" is smoothed for trends, "VOC Fast" is the same index before smoothing,
  so a short event such as a spray shows up within seconds.

### airgradient-loft, airgradient-kitchen

AirGradient ONE units moved from stock firmware to ESPHome with no cloud posting. Shared logic is in
[`common/airgradient-one-common.yaml`](common/airgradient-one-common.yaml).

- Uses the same scoring as `nu-aqi-3`: the worst of VOC, CO2, NOx and NowCast PM.
- PM2.5 keeps the EPA 2021 correction the stock firmware uses.
- "VOC Fast" reports the VOC index every 10 s when it moves, for short events such as cooking;
  "VOC index" and "NOx index" still send one value a minute.
- The 11-LED bar works like the stock one: it starts with one LED at the far left and adds more
  as the air gets worse, moving through the AQI colours.
- The OLED level text follows the bar, so the words and the colour always agree. Both wait until
  the reading is clearly below a line before stepping down.
- The OLED stays, a blue LED sweeps during warm-up, and a test button walks the bar through every level.
- An HVAC page on the OLED shows what the house HVAC is doing (from Home Assistant): a fan that spins
  while the blower runs, the main run (heating, free cooling, cooling, duct purge, fresh air or fan) and
  how long it has lasted. While the HVAC runs, the far-right LED breathes in that run's colour.
- Pins and the EPA formula come from
  [MallocArray/airgradient_esphome](https://github.com/MallocArray/airgradient_esphome), copied in
  so a build never pulls remote code.

### nu-trash-1

A small unit inside the kitchen trash cabinet that watches for odor. Home Assistant compares it
with the kitchen sensor and logs odor events; the yaml only measures.

- SGP41 for VOC and NOx, with the same smoothed "VOC" and fast "VOC Fast" pair as the room units.
- AGS10 metal-oxide TVOC sensor on a second I2C bus at 15 kHz (its limit). It reports absolute ppb,
  so unlike the self-learning SGP41 index it shows how gas builds up over days.
- The BME688 runs as a gas scanner through a local component,
  [`components/bme688_scan`](components/bme688_scan), built on Bosch's open BME68x driver
  instead of the closed BSEC library. Once a minute the heater steps through ten temperatures
  (Bosch profile HP-354, 100 to 320 C), and "Scan" sends the ten resistances as one text value.
  Different gases change different steps, so the scan is a fingerprint that Home Assistant can
  learn odor types from. Package:
  [`common/nu-trash-bme688-scan.yaml`](common/nu-trash-bme688-scan.yaml).
- [`common/nu-trash-bme688.yaml`](common/nu-trash-bme688.yaml) is the BSEC alternative (IAQ
  values, no scan). Bosch's BSEC licence lets you share the yaml but not a compiled binary that
  contains it. [`common/nu-trash-bme680.yaml`](common/nu-trash-bme680.yaml) is for a plain BME680.
- The heater warms the chip about 10 C, so temperature needs the "T-Offset" number and the
  rescaled humidity is approximate.
- AQI texts for each channel and an AQI LED on the board's WS2812, like the NU-AQI units.

---

## Presence

Three kinds of radar, because each is good at something different.

### nu-presence-1, 2

Seeed MR60BHA2 kits (XIAO ESP32-C6, 60 GHz radar, BH1750 light sensor, one RGB LED). Shared logic is
in [`common/mr60-presence-common.yaml`](common/mr60-presence-common.yaml).

- `Occupied` turns on instantly. How it turns off depends on what the target was doing: a person
  walking out clears quickly, and a person who was still gets a longer hold.
- `Max range` fences the radar to its own room when it can see through a wall.
- `Breathing detected` is the strongest sign of a still person, within about 1.5 m.
- Detects a stuck radar and resets it through the module's reset line.
- `Dark` sensor with its own threshold, for light automations.
- Page on port 8080 ([`mr60_web.h`](mr60_web.h)) with live distance, vitals, trends and every setting, with presets and examples.
- Fixes the XIAO's RF switch at boot, which took Wi-Fi from -83 dBm to -39 dBm.

`nu-presence-2` is written but not flashed yet.

### nu-presence-24ghz-1

An HLK-LD2410C on a Waveshare ESP32-S3-LCD-1.47B (it started on an Arduino Nano ESP32). This
radar reports moving and still energy in 0.75 m distance gates, so it can see someone sitting on
a couch.

- The 1.47" screen has three pages: Presence (MOVING / STILL / CLEAR banner, target distance,
  zone chips, move and still energy bars), Gates (per-ring energies against their thresholds) and
  Status. The BOOT button or the `Display page` select switches pages; the screen can turn off
  when the room is clear.
- The board's RGB LED is the presence light: blue while moving, green while still only, with a
  brightness setting, an optional dim glow when clear, and an off switch.

- Per-gate sensitivity, two distance zones and a `Moving target` tuned separately from `Occupied`.
- Tuning page on port 8080 ([`radar_web.h`](radar_web.h)) with live gate energies and a room view.
  The page opens locked, so a stray tap or scroll on a phone can't drag a threshold; unlock it to
  edit, and it locks again two minutes after the last change.
- Wiring is at the top of the yaml.

### nu-presence-24ghz-2

An HLK-LD2450 on an Apollo MTR-1. This radar knows where up to three people are, but it has no
breathing detection and no sensitivity setting, so a seated person simply fades out.

[`presence_2450.h`](presence_2450.h) deals with that using one rule: **people don't vanish, to leave
they have to move.**

- A track starts only from a target that moved, so static ghosts are ignored.
- A track lost while moving means the person walked out, so it clears after `Clear delay`.
- A track lost while still means they faded out, so it's held for `Still hold`.
- Each person has their own track, so someone leaving doesn't clear someone sitting down.
- Tracks survive a restart or flash. A silent radar reads as an empty room instead of a stuck one.
- `Zone 1-3` rectangles, drawn on the page or typed into Home Assistant.
- Tuning page on port 8080 ([`radar_web_2450.h`](radar_web_2450.h)) with a live top-down map.
- `Web page`, `Still preset` and `Moving preset` show the page's status and the current presets in Home Assistant.

### nu-garage-1

A garage unit in one printed case: a Sensirion SEN55 for the air and an HLK-LD2410C radar for
presence, on an ESP32-S3 Super Mini. The SEN55 part follows `nu-esp32-s3-1`, the radar part
`nu-presence-24ghz-1` (same [`radar_web.h`](radar_web.h) tuning page).

- PM, VOC, NOx, temperature and humidity, with the same EPA PM AQI and status texts as the other
  air quality units. The SEN55 has no carbon monoxide sensor, so this is not a CO alarm.
- The ESP32 heats the case, so the SEN55 temperature carries a fixed offset measured against a
  separate garage sensor. Sensirion's compensation adjusts humidity with it, so absolute humidity
  and dew point are unaffected.
- `Moving target` holds for 5 seconds, so a walk through the garage is one clean event for lights.
- Engineering mode turns itself on after every boot, so the per-gate energies are always there.
- If the radar goes quiet for 10 seconds it gets an end-configuration command, and
  `Radar stall fixes` counts how often that happened. Keep the radar's Bluetooth off: with it on,
  this module stopped sending presence data.

---

## Around the house

### nu-range-hood

Replaces the controls on a three-speed kitchen hood.

- Three relays with an interlock, so two speeds can never be on at once.
- The physical speed switches keep working alongside the `Fan Speed` selector in Home Assistant.
- Logs its last four relay moves to a sensor, which is how odd clicking gets diagnosed.

### nu-esp32-2432s028r

A "Cheap Yellow Display" used as a wall touch panel for a light, built with LVGL.

- Resistive panels report phantom touches, so a touch on a sleeping screen only wakes it. The
  controls arm a moment after release, and they disarm when the screen dims.
- Presses shorter than "Min Touch Time" (default 100 ms, set from Home Assistant) are dropped as
  panel noise, and "Last Touch Event" shows how long each press lasted.
- The header comment explains how each guard came about.
- Change the `light.` entity in the yaml to your own.

### nu-thread-1

An ESP32-H2 soil moisture sensor on Thread instead of Wi-Fi. It also publishes radio counters such
as retries, CCA failures and partition changes, which is useful for spotting Wi-Fi overlap.

### nu-wt-1

An ESP32-C6 that runs a small water pump through a relay, exposed as a Home Assistant `valve`.

### nu-test-1

A bench board with an SHT4x and the Nano's RGB LED, used for trying things out.

---

## Using these

The folder layout matches an ESPHome config folder, so headers and `common/` have to stay next to the yaml.

```bash
cp secrets.example.yaml secrets.yaml
```

Fill in `secrets.yaml`, then build a device:

```bash
esphome run nu-aqi-1.yaml
```

Some things to know first:

- **Secrets:** every key and password is a `!secret` reference, listed in
  [`secrets.example.yaml`](secrets.example.yaml). All 22 configs validate on ESPHome 2026.11 with it filled in.
- **Room names:** rooms here are generic (Office 1, Bedroom 2), so they differ slightly from what
  runs in the house. Any `light.` or `sensor.` entity a config reads needs to match your own.
- **Home Assistant:** several devices depend on automations that live in Home Assistant and aren't
  in this repo. The damper controller, for example, only moves when told to.
- **Wi-Fi roaming:** every Wi-Fi device includes
  [`common/wifi-steer.yaml`](common/wifi-steer.yaml). ESPHome's own roaming checks three times
  after a connect and then stops, so a device that joined a far mesh node stays there. This keeps
  checking while the signal is under -60 dBm and moves only to a node at least 10 dB stronger.
- **Mains and HVAC:** the hood and damper builds switch real loads. Wire them safely and at your own risk.

## Credits

- [hunterjm/ac-infinity-ble](https://github.com/hunterjm/ac-infinity-ble): the AC Infinity Bluetooth protocol
- [MallocArray/airgradient_esphome](https://github.com/MallocArray/airgradient_esphome): AirGradient pins and the EPA formula
- [ApolloAutomation/MTR-1](https://github.com/ApolloAutomation/MTR-1): MTR-1 pin map
- Seeed Studio: the MR60BHA2 kit firmware these started from
- [boschsensortec/BME68x_SensorAPI](https://github.com/boschsensortec/BME68x_SensorAPI) (BSD-3, pulled at build time): the driver under `bme688_scan`

## Licence

MIT. See [LICENSE](LICENSE).
