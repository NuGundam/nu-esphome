#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <bme68x/bme68x.h>

namespace esphome {
namespace bme688_scan {

static const uint8_t MAX_STEPS = 10;

class BME688ScanComponent : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_step(uint8_t i, uint16_t temp_c, uint16_t mult) {
    if (i < MAX_STEPS) {
      this->temp_prof_[i] = temp_c;
      this->dur_prof_[i] = mult;
    }
  }
  void set_profile_len(uint8_t n) { this->profile_len_ = n > MAX_STEPS ? MAX_STEPS : n; }
  void set_step_base_ms(uint32_t ms) { this->step_base_ms_ = ms; }
  void set_publish_interval_ms(uint32_t ms) { this->publish_interval_ms_ = ms; }
  void set_scan_interval_ms(uint32_t ms) { this->scan_interval_ms_ = ms; }
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_sensor_ = s; }
  void set_humidity_sensor(sensor::Sensor *s) { this->humidity_sensor_ = s; }
  void set_pressure_sensor(sensor::Sensor *s) { this->pressure_sensor_ = s; }
  void set_gas_resistance_sensor(sensor::Sensor *s) { this->gas_resistance_sensor_ = s; }
  void set_scan_text_sensor(text_sensor::TextSensor *s) { this->scan_text_sensor_ = s; }

 protected:
  bool start_scan_();
  void finish_cycle_();
  void publish_scan_();

  static int8_t read_cb(uint8_t reg, uint8_t *data, uint32_t len, void *intf);
  static int8_t write_cb(uint8_t reg, const uint8_t *data, uint32_t len, void *intf);
  static void delay_cb(uint32_t period_us, void *intf);

  struct bme68x_dev dev_ {};
  struct bme68x_heatr_conf heatr_ {};
  uint16_t temp_prof_[MAX_STEPS]{};
  uint16_t dur_prof_[MAX_STEPS]{};
  uint8_t profile_len_{10};
  uint32_t step_base_ms_{140};
  uint32_t publish_interval_ms_{60000};
  uint32_t scan_interval_ms_{30000};   // start of one cycle to the start of the next; 0 = scan non-stop
  uint32_t cycle_start_{0};
  bool sleeping_{false};
  bool climate_pending_{true};   // publish temperature / humidity / pressure from the next field (start of a cycle)

  uint32_t poll_ms_{70};
  uint32_t last_poll_{0};
  uint32_t last_publish_{0};
  uint32_t last_cycle_{0};
  uint8_t fails_{0};

  float cur_[MAX_STEPS]{};      // this cycle: resistance per step (ohms)
  uint8_t prev_idx_{0};         // heater step of the previous field (a smaller one = the profile wrapped)
  uint16_t seen_{0};            // bit per step that gave a valid, heater-stable value this cycle
  float log_sum_[MAX_STEPS]{};  // since the last publish: sum of ln(ohms) per step
  uint16_t cycles_{0};          // complete cycles since the last publish
  uint32_t cycles_total_{0};

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *pressure_sensor_{nullptr};
  sensor::Sensor *gas_resistance_sensor_{nullptr};
  text_sensor::TextSensor *scan_text_sensor_{nullptr};
};

}  // namespace bme688_scan
}  // namespace esphome
