#pragma once

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

struct vl53l0x_data_t {
  float raw_distance_mm;
  float height_mm;
  bool valid;
  uint32_t t_us;
  uint8_t range_status;
};

class VL53L0X {
public:
  explicit VL53L0X();

  bool begin(uint8_t addr = VL53L0X_I2C_ADDR,
             Adafruit_VL53L0X::VL53L0X_Sense_config_t config = Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT,
             int tare_samples = 30,
             int tare_min_valid = 10,
             uint16_t continuous_period_ms = 200);
  bool update();

  const vl53l0x_data_t& data() const { return data_; }

private:
  Adafruit_VL53L0X lox_;
  VL53L0X_RangingMeasurementData_t measure_;
  vl53l0x_data_t data_{};
  float tare_offset_mm_ = 0.0f;

  bool autoTare(int samples, int min_valid, uint16_t continuous_period_ms);
};
