#pragma once
#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include "imu_types.hpp"

class ICM20948 {
public:
  ICM20948() = default;

  // begin_i2c=true will call Wire.begin(sda,scl) + Wire.setClock(freq)
  // If you're sharing the I2C bus and already did Wire.begin, pass begin_i2c=false.
  bool begin(uint8_t sda_pin = 5, uint8_t scl_pin = 4, uint32_t freq_hz = 400000, bool begin_i2c = true);

  // Optional configuration helpers
  void set_accel_range(icm20948_accel_range_t range);
  void set_gyro_range(icm20948_gyro_range_t range);
  void set_mag_rate(ak09916_data_rate_t rate);

  // Poll sensor and update latest imu9_t
  bool update();

  const imu9_t& data() const { return data_; }

private:
  Adafruit_ICM20948 icm_;
  imu9_t data_{};

  // Unit conversions from Adafruit events:
  // accel: m/s^2 -> g
  // gyro: rad/s  -> deg/s
  static constexpr float G_SI = 9.80665f;
  static constexpr float RAD2DEG = 57.29577951308232f;
};
