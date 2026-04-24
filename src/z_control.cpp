#include "z_control.hpp"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void ZControl::reset() {
  measured_mm_ = 0.0f;
  filtered_mm_ = 0.0f;
  error_mm_ = 0.0f;
  trim_pct_ = 0.0f;
  integral_ = 0.0f;
  previous_error_ = 0.0f;
  has_filter_state_ = false;
  valid_ = false;
}

void ZControl::setParams(float kp, float ki, float kd,
                         float lpf_cutoff_hz,
                         float deadband_mm,
                         float integral_limit,
                         float trim_limit) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  lpf_cutoff_hz_ = (lpf_cutoff_hz < 0.1f) ? 0.1f : lpf_cutoff_hz;
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

  if (!has_filter_state_) {
    filtered_mm_ = measured_mm_;
    has_filter_state_ = true;
  } else {
    const float tau = 1.0f / (2.0f * (float)M_PI * lpf_cutoff_hz_);
    const float alpha = tau / (tau + dt);
    filtered_mm_ = alpha * filtered_mm_ + (1.0f - alpha) * measured_mm_;
  }

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
