#pragma once
// wifi_diag.h v1.0 (2026-11-07) - helpers for common/wifi-diag.yaml.
// Place next to the device yamls. Shared by every Wi-Fi ESP32 device.
#include <cmath>
#include <string>
#include <esp_wifi.h>

// Maximum TX power the radio driver is set to, in dBm (0.25 dBm steps); NAN before Wi-Fi starts.
static inline float wd_tx_power_dbm() {
  int8_t p = 0;
  if (esp_wifi_get_max_tx_power(&p) != ESP_OK)
    return NAN;
  return p * 0.25f;
}

// Maps a 2.4 GHz BSSID to the access point name the router app shows.
// Unknown radios return the BSSID itself.
static inline std::string wd_ap_name(const std::string &bssid) {
  if (bssid == "AA:BB:CC:DD:EE:01") return "Loft";
  if (bssid == "AA:BB:CC:DD:EE:02") return "Study";
  if (bssid == "AA:BB:CC:DD:EE:03") return "Family Room";
  return bssid;
}
