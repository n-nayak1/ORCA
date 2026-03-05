#pragma once
#include <Arduino.h>

#include "imu_types.hpp"

extern "C" {
#include <Fusion.h>
}

struct euler_t {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
};

struct quat_t {
  float w, x, y, z;
};

class Filter {
public:
  void begin(float expected_sample_rate_hz = 100.0f,
             FusionConvention convention = FusionConventionNwu);

  void setGain(float gain);
  void setRejection(float accelRejectionDeg, float magRejectionDeg);
  void setGyroRangeDps(float gyroRangeDps);
  void reset();

  bool update6(const imu6_t& s);
  bool update9(const imu9_t& s);

  euler_t euler6() const { return euler6_; }
  euler_t euler9() const { return euler9_; }

  quat_t quat6() const { return quat6_; }
  quat_t quat9() const { return quat9_; }

private:
  static float dtSeconds(uint32_t t_us, uint32_t& last_us);

  static euler_t toEulerDeg(const FusionQuaternion& q);
  static quat_t  toQuat(const FusionQuaternion& q);

  void applySettings_();

  FusionAhrs ahrs6_{};
  FusionAhrs ahrs9_{};

  FusionAhrsSettings settings_{};

  uint32_t last6_us_ = 0;
  uint32_t last9_us_ = 0;

  euler_t euler6_{};
  euler_t euler9_{};

  quat_t quat6_{1,0,0,0};
  quat_t quat9_{1,0,0,0};
};