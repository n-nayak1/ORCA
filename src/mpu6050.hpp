#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "imu_types.hpp"

class MPU6050 {
public:
  explicit MPU6050(uint8_t addr = 0x68);

  bool begin(uint8_t sda_pin = 21, uint8_t scl_pin = 22, uint32_t freq_hz = 100000, bool begin_i2c = true);

  bool update();                 // updates internal imu6_t
  const imu6_t& data() const { return data_; }

private:
  uint8_t addr_;
  imu6_t data_{};

  static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
  static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

  bool write_byte(uint8_t reg, uint8_t val);
  bool read_bytes(uint8_t start_reg, uint8_t* buf, size_t len);
  static int16_t be16(const uint8_t* p);
};