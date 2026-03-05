#include "icm20948.hpp"

bool ICM20948::begin(uint8_t sda_pin, uint8_t scl_pin, uint32_t freq_hz, bool begin_i2c) {
  if (begin_i2c) {
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(freq_hz);
  }

  // Default Adafruit begin_I2C() uses Wire and the default address.
  // Some versions allow begin_I2C(addr, &wire). If yours does, we can add addr support.
  if (!icm_.begin_I2C(0x69, &Wire) && !icm_.begin_I2C(0x68, &Wire)) {
    Serial.println("ICM20948 not detected");
    return false;
  }

  // Reasonable defaults for a quad (tune later):
  // +-8g accel, 2000 dps gyro, mag 50 Hz
  icm_.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  icm_.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm_.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);

  delay(10);
  return true;
}

void ICM20948::set_accel_range(icm20948_accel_range_t range) {
  icm_.setAccelRange(range);
}

void ICM20948::set_gyro_range(icm20948_gyro_range_t range) {
  icm_.setGyroRange(range);
}

void ICM20948::set_mag_rate(ak09916_data_rate_t rate) {
  icm_.setMagDataRate(rate);
}

bool ICM20948::update() {
  sensors_event_t accel, gyro, mag, temp;
  icm_.getEvent(&accel, &gyro, &temp, &mag);

  data_.t_us = micros();

  // accel in m/s^2 -> g
  data_.ax = accel.acceleration.x / G_SI;
  data_.ay = accel.acceleration.y / G_SI;
  data_.az = accel.acceleration.z / G_SI;

  // gyro in rad/s -> deg/s
  data_.gx = gyro.gyro.x * RAD2DEG;
  data_.gy = gyro.gyro.y * RAD2DEG;
  data_.gz = gyro.gyro.z * RAD2DEG;

  // mag already in uT per Adafruit demo
  data_.mx = mag.magnetic.x;
  data_.my = mag.magnetic.y;
  data_.mz = mag.magnetic.z;

  return true;
}