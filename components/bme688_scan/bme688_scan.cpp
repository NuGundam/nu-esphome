#include "bme688_scan.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace esphome {
namespace bme688_scan {

static const char *const TAG = "bme688_scan";

int8_t BME688ScanComponent::read_cb(uint8_t reg, uint8_t *data, uint32_t len, void *intf) {
  return static_cast<BME688ScanComponent *>(intf)->read_bytes(reg, data, len) ? 0 : -1;
}
int8_t BME688ScanComponent::write_cb(uint8_t reg, const uint8_t *data, uint32_t len, void *intf) {
  return static_cast<BME688ScanComponent *>(intf)->write_bytes(reg, data, len) ? 0 : -1;
}
void BME688ScanComponent::delay_cb(uint32_t period_us, void *intf) { delayMicroseconds(period_us); }

void BME688ScanComponent::setup() {
  this->dev_.intf_ptr = (void *) this;
  this->dev_.intf = BME68X_I2C_INTF;
  this->dev_.read = BME688ScanComponent::read_cb;
  this->dev_.write = BME688ScanComponent::write_cb;
  this->dev_.delay_us = BME688ScanComponent::delay_cb;
  this->dev_.amb_temp = 25;

  int8_t rs = bme68x_init(&this->dev_);
  if (rs != BME68X_OK) {
    ESP_LOGE(TAG, "bme68x_init failed: %d", rs);
    this->mark_failed();
    return;
  }
  if (!this->start_scan_()) {
    this->mark_failed();
    return;
  }
  this->last_publish_ = millis();
  this->last_cycle_ = millis();
}

bool BME688ScanComponent::start_scan_() {
  struct bme68x_conf conf {};
  int8_t rs = bme68x_get_conf(&conf, &this->dev_);
  conf.filter = BME68X_FILTER_OFF;
  conf.odr = BME68X_ODR_NONE;
  conf.os_hum = BME68X_OS_1X;
  conf.os_pres = BME68X_OS_16X;
  conf.os_temp = BME68X_OS_2X;
  rs = bme68x_set_conf(&conf, &this->dev_);
  if (rs != BME68X_OK) {
    ESP_LOGE(TAG, "bme68x_set_conf failed: %d", rs);
    return false;
  }
  // one time base = TPHG conversion time + shared heater time
  uint32_t meas_ms = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &this->dev_) / 1000;
  uint32_t shared = this->step_base_ms_ > meas_ms + 5 ? this->step_base_ms_ - meas_ms : 5;
  this->heatr_.enable = BME68X_ENABLE;
  this->heatr_.heatr_temp_prof = this->temp_prof_;
  this->heatr_.heatr_dur_prof = this->dur_prof_;
  this->heatr_.profile_len = this->profile_len_;
  this->heatr_.shared_heatr_dur = (uint16_t) shared;
  rs = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &this->heatr_, &this->dev_);
  if (rs != BME68X_OK) {
    ESP_LOGE(TAG, "bme68x_set_heatr_conf failed: %d", rs);
    return false;
  }
  rs = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &this->dev_);
  if (rs != BME68X_OK) {
    ESP_LOGE(TAG, "bme68x_set_op_mode failed: %d", rs);
    return false;
  }
  this->poll_ms_ = this->step_base_ms_ / 2;
  this->seen_ = 0;
  this->prev_idx_ = 0;
  this->sleeping_ = false;
  this->climate_pending_ = true;
  this->cycle_start_ = millis();
  return true;
}

void BME688ScanComponent::loop() {
  const uint32_t now = millis();
  if (now - this->last_poll_ < this->poll_ms_)
    return;
  this->last_poll_ = now;

  // resting between two scan cycles (scan_interval): wake the chip when the next cycle is due
  if (this->sleeping_) {
    if (now - this->cycle_start_ < this->scan_interval_ms_)
      return;
    if (bme68x_set_op_mode(BME68X_PARALLEL_MODE, &this->dev_) != BME68X_OK) {
      this->start_scan_();
    }
    this->sleeping_ = false;
    this->climate_pending_ = true;
    this->cycle_start_ = now;
    this->seen_ = 0;
    this->prev_idx_ = 0;
    return;
  }

  struct bme68x_data data[3];
  uint8_t n = 0;
  int8_t rs = bme68x_get_data(BME68X_PARALLEL_MODE, data, &n, &this->dev_);
  if (rs < BME68X_OK) {
    if (++this->fails_ == 20) {
      ESP_LOGW(TAG, "bme68x_get_data keeps failing (%d)", rs);
      this->status_set_warning();
    }
  } else {
    this->fails_ = 0;
    for (uint8_t i = 0; i < n; i++) {
      const struct bme68x_data &d = data[i];
      if (!(d.status & BME68X_NEW_DATA_MSK))
        continue;
      const uint8_t idx = d.gas_index;
      if (idx >= this->profile_len_)
        continue;
      // Climate values come from the FIRST field of a cycle: the chip has rested since the last cycle, so its own
      // heater has not warmed it yet (taken at the last 320 C step they read ~10 C high and the humidity far off).
      // idx 0 only: right after a wake-up the chip can still hand out the last (hot) field of the previous cycle
      if (this->climate_pending_ && idx == 0) {
        this->climate_pending_ = false;
        if (this->temperature_sensor_ != nullptr)
          this->temperature_sensor_->publish_state(d.temperature);
        if (this->humidity_sensor_ != nullptr)
          this->humidity_sensor_->publish_state(d.humidity);
        if (this->pressure_sensor_ != nullptr)
          this->pressure_sensor_->publish_state(d.pressure * 0.01f);
      }
      // the profile wrapped around without a valid last step: drop that cycle
      if (idx < this->prev_idx_) {
        this->seen_ = 0;
        this->cycle_start_ = now;
      }
      this->prev_idx_ = idx;
      // Each step repeats its field every time base; only the last one carries a new gas value (GASM_VALID).
      // HEAT_STAB is not required: the short 100 C step after 320 C often reports "not stable" and is still the
      // same point of the profile every cycle.
      if ((d.status & BME68X_GASM_VALID_MSK) && d.gas_resistance > 0.0f) {
        this->cur_[idx] = d.gas_resistance;
        this->seen_ |= (uint16_t) (1u << idx);
        if (idx == this->profile_len_ - 1) {
          // the valid last step ends the cycle; with a scan_interval the chip then rests (heater off) until the
          // next cycle is due, which keeps its self-heating small
          this->finish_cycle_();
          this->prev_idx_ = 0;
          if (this->scan_interval_ms_ > 0) {
            bme68x_set_op_mode(BME68X_SLEEP_MODE, &this->dev_);
            this->sleeping_ = true;
            break;
          }
          this->cycle_start_ = now;
          this->climate_pending_ = true;
        }
      }
    }
  }

  // no complete cycle for a long time: put the chip back into scan mode
  // (int32_t): last_cycle_ can be a few ms newer than `now`, which was read at the top of loop()
  if ((int32_t) (now - this->last_cycle_) > 120000) {
    ESP_LOGW(TAG, "No complete scan cycle for 2 min, restarting the scan");
    this->last_cycle_ = now;
    if (this->start_scan_())
      this->status_clear_warning();
    else
      this->status_set_warning();
  }
  if (now - this->last_publish_ >= this->publish_interval_ms_) {
    this->last_publish_ = now;
    this->publish_scan_();
  }
}

void BME688ScanComponent::finish_cycle_() {
  const uint16_t all = (uint16_t) ((1u << this->profile_len_) - 1);
  if (this->seen_ == all) {
    for (uint8_t i = 0; i < this->profile_len_; i++)
      this->log_sum_[i] += logf(this->cur_[i]);
    this->cycles_++;
    this->cycles_total_++;
    this->last_cycle_ = millis();
    this->status_clear_warning();
    if (this->gas_resistance_sensor_ != nullptr)
      this->gas_resistance_sensor_->publish_state(this->cur_[this->profile_len_ - 1]);
  } else {
    ESP_LOGD(TAG, "Incomplete scan cycle (steps seen 0x%03x)", this->seen_);
  }
  this->seen_ = 0;
}

void BME688ScanComponent::publish_scan_() {
  if (this->cycles_ == 0)
    return;
  // geometric mean of each step over the cycles since the last publish, kilo-ohms
  std::string out;
  char buf[16];
  for (uint8_t i = 0; i < this->profile_len_; i++) {
    const float kohm = expf(this->log_sum_[i] / this->cycles_) * 0.001f;
    snprintf(buf, sizeof(buf), kohm >= 100.0f ? "%.0f" : kohm >= 10.0f ? "%.1f" : "%.2f", kohm);
    if (i)
      out += ',';
    out += buf;
    this->log_sum_[i] = 0.0f;
  }
  ESP_LOGD(TAG, "Scan (%u cycles): %s", this->cycles_, out.c_str());
  this->cycles_ = 0;
  if (this->scan_text_sensor_ != nullptr)
    this->scan_text_sensor_->publish_state(out);
}

void BME688ScanComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "BME688 gas scan:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Chip id 0x%02x, variant 0x%02x", this->dev_.chip_id, (unsigned) this->dev_.variant_id);
  uint32_t total = 0;
  for (uint8_t i = 0; i < this->profile_len_; i++) {
    ESP_LOGCONFIG(TAG, "  Step %u: %u C x %u", i, this->temp_prof_[i], this->dur_prof_[i]);
    total += this->dur_prof_[i];
  }
  ESP_LOGCONFIG(TAG, "  Time base %u ms, cycle %.1f s, one cycle every %u s (0 = non-stop), publish every %u s",
                (unsigned) this->step_base_ms_, total * this->step_base_ms_ / 1000.0f,
                (unsigned) (this->scan_interval_ms_ / 1000), (unsigned) (this->publish_interval_ms_ / 1000));
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup failed");
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
  LOG_SENSOR("  ", "Gas Resistance", this->gas_resistance_sensor_);
  LOG_TEXT_SENSOR("  ", "Scan", this->scan_text_sensor_);
}

}  // namespace bme688_scan
}  // namespace esphome
