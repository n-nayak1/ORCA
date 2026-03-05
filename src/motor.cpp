#include "motor.hpp"

Motor::Motor(uint8_t pin, uint8_t channel,
             uint32_t freq_hz,
             uint8_t res_bits,
             uint32_t duty_min,
             uint32_t duty_max)
: pin_(pin),
  ch_(channel),
  freq_hz_(freq_hz),
  res_bits_(res_bits),
  duty_min_(duty_min),
  duty_max_(duty_max) {}

bool Motor::begin(uint32_t arm_delay_ms) {
  if (ch_ >= 16) return false; // ESP32 has 16 LEDC channels

  // Setup this channel
  ledcSetup(ch_, freq_hz_, res_bits_);
  ledcAttachPin(pin_, ch_);

  // Arm ESC at minimum
  ledcWrite(ch_, duty_min_);
  delay(arm_delay_ms);

  armed_ = true;
  return true;
}

void Motor::setDutyRange(uint32_t duty_min, uint32_t duty_max) {
  duty_min_ = duty_min;
  duty_max_ = duty_max;
}

void Motor::setPercent(float percent) {
  percent = clampf(percent, 0.0f, 100.0f);

  const uint32_t span = (duty_max_ > duty_min_) ? (duty_max_ - duty_min_) : 0;
  const uint32_t duty = duty_min_ + (uint32_t)((float)span * (percent / 100.0f));

  ledcWrite(ch_, duty);
}

void Motor::setDuty(uint32_t duty) {
  ledcWrite(ch_, duty);
}

void Motor::stop() {
  ledcWrite(ch_, duty_min_);
}