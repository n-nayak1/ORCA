#include "filter.hpp"
#include <math.h>

void Filter::begin(float expected_sample_rate_hz, FusionConvention convention) {
  FusionAhrsInitialise(&ahrs6_);
  FusionAhrsInitialise(&ahrs9_);

  // Reasonable defaults
  settings_.convention = convention;
  settings_.gain = 0.5f;                 // typical starting point
  settings_.gyroscopeRange = 2000.0f;    // set to your IMU range if you want
  settings_.accelerationRejection = 10.0f;
  settings_.magneticRejection = 10.0f;

  // Fusion uses a "recoveryTriggerPeriod" in *samples*
  // ~5 seconds is a common choice; if you dislike rejection behavior, set to 0.
  const float seconds = 5.0f;
  const uint16_t samples = (uint16_t) lroundf(seconds * expected_sample_rate_hz);
  settings_.recoveryTriggerPeriod = samples;

  applySettings_();
  reset(); // clears timestamps + outputs
}

void Filter::setGain(float gain) {
  settings_.gain = gain;
  applySettings_();
}

void Filter::setRejection(float accelRejectionDeg, float magRejectionDeg) {
  settings_.accelerationRejection = accelRejectionDeg;
  settings_.magneticRejection = magRejectionDeg;
  applySettings_();
}

void Filter::setGyroRangeDps(float gyroRangeDps) {
  settings_.gyroscopeRange = gyroRangeDps;
  applySettings_();
}

void Filter::reset() {
  FusionAhrsInitialise(&ahrs6_);
  FusionAhrsInitialise(&ahrs9_);
  applySettings_();

  last6_us_ = 0;
  last9_us_ = 0;

  euler6_ = {};
  euler9_ = {};
  quat6_ = {1, 0, 0, 0};
  quat9_ = {1, 0, 0, 0};
}


void Filter::applySettings_() {
  FusionAhrsSetSettings(&ahrs6_, &settings_);
  FusionAhrsSetSettings(&ahrs9_, &settings_);
}

float Filter::dtSeconds(uint32_t t_us, uint32_t& last_us) {
  if (last_us == 0) {
    last_us = t_us;
    return 0.0f;
  }
  // uint32 wrap-safe subtraction is fine in unsigned arithmetic
  const uint32_t dt_us = (uint32_t)(t_us - last_us);
  last_us = t_us;
  return (float)dt_us * 1e-6f;
}

euler_t Filter::toEulerDeg(const FusionQuaternion& q) {
  const FusionEuler e = FusionQuaternionToEuler(q);
  return euler_t{
    .roll_deg  = e.angle.roll,
    .pitch_deg = e.angle.pitch,
    .yaw_deg   = e.angle.yaw
  };
}

quat_t Filter::toQuat(const FusionQuaternion& q) {
  return quat_t{ q.element.w, q.element.x, q.element.y, q.element.z };
}

bool Filter::update6(const imu6_t& s) {
  const float dt = dtSeconds(s.t_us, last6_us_);
  if (dt <= 0.0f) return false; // need at least 2 samples to have dt

  const FusionVector gyro = { .axis = { s.gx, s.gy, s.gz } }; // deg/s
  const FusionVector acc  = { .axis = { s.ax, s.ay, s.az } }; // g

  // 6DoF "Madgwick-style" update (no magnetometer)
  FusionAhrsUpdateNoMagnetometer(&ahrs6_, gyro, acc, dt);

  const FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs6_);
  quat6_  = toQuat(q);
  euler6_ = toEulerDeg(q);
  return true;
}

bool Filter::update9(const imu9_t& s) {
  const float dt = dtSeconds(s.t_us, last9_us_);
  if (dt <= 0.0f) return false;

  const FusionVector gyro = { .axis = { s.gx, s.gy, s.gz } }; // deg/s
  const FusionVector acc  = { .axis = { s.ax, s.ay, s.az } }; // g
  const FusionVector mag  = { .axis = { s.mx, s.my, s.mz } }; // calibrated (uT ok)

  // Full 9DoF update
  FusionAhrsUpdate(&ahrs9_, gyro, acc, mag, dt);

  const FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs9_);
  quat9_  = toQuat(q);
  euler9_ = toEulerDeg(q);
  return true;
}