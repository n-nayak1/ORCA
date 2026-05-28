#pragma once

#include <Arduino.h>


class ZControl {
public:
  static constexpr float ALT_MIN_MM = 0.0f;
  static constexpr float ALT_MAX_MM = 1800.0f;

  ZControl() = default;

  void reset();

  void setParams(float kp, float ki, float kd,
                 float deadband_mm,
                 float integral_limit,
                 float trim_limit);

  void setHoverThrottle(float hover_throttle_pct);
  void setSetpointMm(float setpoint_mm);

  bool update(float measured_mm, bool measurement_valid, float dt);

  float commandThrottle() const;

  float setpointMm() const { return setpoint_mm_; }
  float measuredMm() const { return measured_mm_; }
  float filteredMm() const { return filtered_mm_; }
  float errorMm() const { return error_mm_; }
  float trimPct() const { return trim_pct_; }
  float hoverThrottlePct() const { return hover_throttle_pct_; }
  bool valid() const { return valid_; }

private:
  float kp_ = 1.0e-5f;
  float ki_ = 0.000f;
  float kd_ = 0.000f;

  float deadband_mm_ = 400.0f;
  float integral_limit_ = 2000.0f;
  float trim_limit_ = 20.0f;
  float hover_throttle_pct_ = 5.0f;

  float setpoint_mm_ = 0.0f;
  float measured_mm_ = 0.0f;
  float filtered_mm_ = 0.0f;
  float error_mm_ = 0.0f;
  float trim_pct_ = 0.0f;
  float integral_ = 0.0f;
  float previous_error_ = 0.0f;
  bool valid_ = false;

  static float clamp(float val, float min, float max);
};
