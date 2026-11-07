#pragma once
// NU-AQI helper functions (v5.0) - shared by nuaqi-common, nu-aqi-3 and airgradient-one-common
// Place this file next to nu-aqi-3.yaml, alongside custom_web.h.

#include <cmath>
#include <string>
#include <esp_wifi.h>

// Reads the actual maximum WiFi TX power from the radio driver.
// The driver stores it in 0.25 dBm steps; returns dBm, or NAN if
// WiFi is not started yet.
static inline float nuaqi_wifi_tx_power_dbm() {
  int8_t p = 0;
  if (esp_wifi_get_max_tx_power(&p) != ESP_OK)
    return NAN;
  return p * 0.25f;
}

// Sets the WiFi max TX power at runtime (used by the adaptive power loop).
// The driver quantizes to 0.25 dBm steps and maps to its discrete levels;
// safe to call repeatedly, silently ignored if WiFi is not started yet.
static inline void nuaqi_wifi_set_tx_power_dbm(float dbm) {
  esp_wifi_set_max_tx_power((int8_t) lroundf(dbm * 4.0f));
}

// Maps a 2.4 GHz BSSID to the access point name the TP-Link app shows
// (read off the units on 2026-11-07). Unknown radios return the BSSID itself.
static inline std::string nuaqi_ap_name(const std::string &bssid) {
  if (bssid == "AA:BB:CC:DD:EE:01") return "Loft";
  if (bssid == "AA:BB:CC:DD:EE:02") return "Study";
  if (bssid == "AA:BB:CC:DD:EE:03") return "Family Room";
  return bssid;
}
