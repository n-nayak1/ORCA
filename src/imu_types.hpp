#pragma once
#include <Arduino.h>

struct imu6_t {
  float ax, ay, az;   // g
  float gx, gy, gz;   // deg/s
  uint32_t t_us;      // timestamp (micros)
};

struct imu9_t {
  float ax, ay, az;   // g
  float gx, gy, gz;   // deg/s
  float mx, my, mz;   // uT (or raw / normalized—pick one convention)
  uint32_t t_us;      // timestamp (micros)
};