#include "z_control.hpp"

#include <math.h>

void ZControl::reset() {
  measured_mm_ = 0.0f;
  filtered_mm_ = 0.0f;
  error_mm_ = 0.0f;
  trim_pct_ = 0.0f;
  integral_ = 0.0f;
  previous_error_ = 0.0f;
  valid_ = false;
}

void ZControl::setParams(float kp, float ki, float kd,
                         float deadband_mm,
                         float integral_limit,
                         float trim_limit) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  deadband_mm_ = (deadband_mm < 0.0f) ? 0.0f : deadband_mm;
  integral_limit_ = (integral_limit < 0.0f) ? 0.0f : integral_limit;
  trim_limit_ = (trim_limit < 0.0f) ? 0.0f : trim_limit;
}

void ZControl::setHoverThrottle(float hover_throttle_pct) {
  hover_throttle_pct_ = clamp(hover_throttle_pct, 0.0f, 100.0f);
}

void ZControl::setSetpointMm(float setpoint_mm) {
  setpoint_mm_ = clamp(setpoint_mm, ALT_MIN_MM, ALT_MAX_MM);
}

bool ZControl::update(float measured_mm, bool measurement_valid, float dt) {
  if (dt <= 0.0f) {
    trim_pct_ = 0.0f;
    valid_ = false;
    return false;
  }

  if (!measurement_valid) {
    trim_pct_ = 0.0f;
    previous_error_ = 0.0f;
    integral_ *= 0.98f;
    valid_ = false;
    return false;
  }

  measured_mm_ = clamp(measured_mm, ALT_MIN_MM, ALT_MAX_MM);
  filtered_mm_ = measured_mm_;

  error_mm_ = setpoint_mm_ - filtered_mm_;
  if (fabsf(error_mm_) < deadband_mm_) error_mm_ = 0.0f;

  integral_ += error_mm_ * dt;
  integral_ = clamp(integral_, -integral_limit_, integral_limit_);

  const float derivative = (error_mm_ - previous_error_) / dt;
  previous_error_ = error_mm_;

  trim_pct_ = (kp_ * error_mm_) +
              (ki_ * integral_) +
              (kd_ * derivative);
  trim_pct_ = clamp(trim_pct_, -trim_limit_, trim_limit_);

  valid_ = true;
  return true;
}

float ZControl::commandThrottle() const {
  return clamp(hover_throttle_pct_ + trim_pct_, 0.0f, 100.0f);
}

float ZControl::clamp(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}
