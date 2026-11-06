#pragma once
// NU-AQI-3 helper functions (v4.9)
// Place this file next to nu-aqi-3.yaml, alongside custom_web.h.

#include <cmath>
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