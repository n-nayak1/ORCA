#include "mpu6050.hpp"

MPU6050::MPU6050(uint8_t addr) : addr_(addr) {}

bool MPU6050::begin(uint8_t sda_pin, uint8_t scl_pin, uint32_t freq_hz, bool begin_i2c) {
  if (begin_i2c) {
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(freq_hz);
  }
  if (!write_byte(REG_PWR_MGMT_1, 0x00)) return false; // wake
  delay(10);
  return true;
}

bool MPU6050::write_byte(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool MPU6050::read_bytes(uint8_t start_reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(addr_);
  Wire.write(start_reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t n = Wire.requestFrom((int)addr_, (int)len, (int)true);
  if (n != len) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

int16_t MPU6050::be16(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] << 8 | (uint16_t)p[1]);
}

bool MPU6050::update() {
  uint8_t b[14];
  if (!read_bytes(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;

  int16_t ax = be16(&b[0]);
  int16_t ay = be16(&b[2]);
  int16_t az = be16(&b[4]);
  int16_t gx = be16(&b[8]);
  int16_t gy = be16(&b[10]);
  int16_t gz = be16(&b[12]);

  data_.t_us = micros();

  // same scaling you used (assumes ±2g, ±250 dps)
  data_.ax = ax / 16384.0f;
  data_.ay = ay / 16384.0f;
  data_.az = az / 16384.0f;

  data_.gx = gx / 131.0f;
  data_.gy = gy / 131.0f;
  data_.gz = gz / 131.0f;

  return true;
}