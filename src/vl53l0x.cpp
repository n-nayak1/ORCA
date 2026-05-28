#include "vl53l0x.hpp"

VL53L0X::VL53L0X() {}

bool VL53L0X::begin(uint8_t addr, Adafruit_VL53L0X::VL53L0X_Sense_config_t config,
                    int tare_samples, int tare_min_valid,
                    uint16_t continuous_period_ms) {
  if (!lox_.begin(addr, false, &Wire, config)) {
    return false;
  }

  data_.raw_distance_mm = 0.0f;
  data_.height_mm = 0.0f;
  data_.valid = false;
  data_.t_us = 0;
  data_.range_status = 0xFF;

  return autoTare(tare_samples, tare_min_valid, continuous_period_ms);
}

bool VL53L0X::autoTare(int samples, int min_valid, uint16_t continuous_period_ms) {
  float sum_mm = 0.0f;
  int valid_count = 0;

  for (int i = 0; i < samples; ++i) {
    VL53L0X_Error err = lox_.rangingTest(&measure_, false);
    if (err == VL53L0X_ERROR_NONE && measure_.RangeStatus == 0) {
      sum_mm += static_cast<float>(measure_.RangeMilliMeter);
      valid_count++;
    }
    delay(15);
  }

  if (valid_count < min_valid) {
    return false;
  }

  tare_offset_mm_ = sum_mm / static_cast<float>(valid_count);
  if (tare_offset_mm_ < 0.0f) tare_offset_mm_ = 0.0f;

  if (!lox_.startRangeContinuous(continuous_period_ms)) {
    return false;
  }

  return true;
}

bool VL53L0X::update() {
  if (!lox_.isRangeComplete()) {
    return true;
  }

  uint16_t range = lox_.readRangeResult();
  uint8_t status = lox_.readRangeStatus();

  data_.raw_distance_mm = static_cast<float>(range);
  data_.range_status = status;
  data_.valid = (status == 0 && range != 0xFFFF);
  data_.t_us = micros();

  data_.height_mm = data_.raw_distance_mm - tare_offset_mm_;
  if (data_.height_mm < 0.0f) data_.height_mm = 0.0f;

  return true;
}
