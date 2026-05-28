#pragma once
#include <Arduino.h>

class Motor {
public:
  // Defaults match your earlier constants (50 Hz, 16-bit, 1.0–2.0 ms mapped duty)
  Motor(uint8_t pin, uint8_t channel,
        uint32_t freq_hz = 50,
        uint8_t  res_bits = 16,
        uint32_t duty_min = 3277,
        uint32_t duty_max = 6553);

  // Sets up LEDC for this channel, attaches the pin, and arms ESC at duty_min
  bool begin(uint32_t arm_delay_ms = 2000);

  // Percent in [0..100]
  void setPercent(float percent);

  // Direct duty write (0..2^res-1) - use carefully
  void setDuty(uint32_t duty);

  // Emergency stop -> writes duty_min (typical ESC “off”)
  void stop();

  // Accessors
  uint8_t  pin() const { return pin_; }
  uint8_t  channel() const { return ch_; }
  bool     armed() const { return armed_; }

  uint32_t dutyMin() const { return duty_min_; }
  uint32_t dutyMax() const { return duty_max_; }

  // Optional: update endpoints (e.g., cap max throttle)
  void setDutyRange(uint32_t duty_min, uint32_t duty_max);

  void setPercentLog(float percent, float strength = 5.0f);

private:
  uint8_t  pin_;
  uint8_t  ch_;
  uint32_t freq_hz_;
  uint8_t  res_bits_;

  uint32_t duty_min_;
  uint32_t duty_max_;

  bool armed_ = false;

  static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }
};
