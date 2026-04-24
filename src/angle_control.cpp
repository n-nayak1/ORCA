#include "angle_control.hpp"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

AngleController::AngleController() {
  pitch_pid_ = {0.06f, 0.000f, 0.065f, 0.0f, 0.0f, 40.0f, 0.0f};
  roll_pid_ = {0.07f, 0.000f, 0.065f, 0.0f, 0.0f, 40.0f, 0.0f};
  yaw_pid_ = {0.005f, 0.000f, 0.000f, 0.0f, 0.0f, 40.0f, 0.0f};
  resetIntegrals();
}

float AngleController::runPid(AnglePidAxis& pid, float error, float dt) {
  if (dt <= 0.0f) return 0.0f;

  if (fabsf(error) < 0.5f) error = 0.0f;

  pid.integral += error * dt;
  pid.integral = clamp(pid.integral, -pid.integral_limit, pid.integral_limit);

  const float d_raw = (error - pid.previous_error) / dt;
  pid.previous_error = error;

  const float cutoff_hz = 15.0f;
  const float tau = 1.0f / (2.0f * (float)M_PI * cutoff_hz);
  const float alpha = tau / (tau + dt);
  pid.d_filtered = alpha * pid.d_filtered + (1.0f - alpha) * d_raw;

  return (pid.kp * error) +
         (pid.ki * pid.integral) +
         (pid.kd * pid.d_filtered);
}

float AngleController::wrapAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void AngleController::compute(euler_t target, euler_t current, float throttle, float dt) {
  const float target_p = clamp(target.pitch_deg, -5.0f, 5.0f);
  const float target_r = clamp(target.roll_deg, -5.0f, 5.0f);

  const float pitch_error = target_p - current.pitch_deg;
  const float roll_error = target_r - current.roll_deg;
  const float yaw_error = wrapAngle180(target.yaw_deg - current.yaw_deg);

  const float p_adj = runPid(pitch_pid_, pitch_error, dt);
  const float r_adj = runPid(roll_pid_, roll_error, dt);
  const float y_adj = runPid(yaw_pid_, yaw_error, dt);
  (void)y_adj;

  motor_outputs_[0] = throttle - p_adj - r_adj;
  motor_outputs_[1] = throttle + p_adj - r_adj;
  motor_outputs_[2] = throttle - p_adj + r_adj;
  motor_outputs_[3] = throttle + p_adj + r_adj;

  for (int i = 0; i < 4; i++) {
    motor_outputs_[i] = clamp(motor_outputs_[i], 0.0f, 100.0f);
  }
}

void AngleController::updateGains(float pP, float pI, float pD,
                                  float rP, float rI, float rD,
                                  float yP, float yI, float yD) {
  pitch_pid_.kp = pP;
  pitch_pid_.ki = pI;
  pitch_pid_.kd = pD;

  roll_pid_.kp = rP;
  roll_pid_.ki = rI;
  roll_pid_.kd = rD;

  yaw_pid_.kp = yP;
  yaw_pid_.ki = yI;
  yaw_pid_.kd = yD;

  resetIntegrals();
}

void AngleController::resetIntegrals() {
  AnglePidAxis* axes[3] = {&pitch_pid_, &roll_pid_, &yaw_pid_};
  for (int i = 0; i < 3; i++) {
    axes[i]->integral = 0.0f;
    axes[i]->previous_error = 0.0f;
    axes[i]->d_filtered = 0.0f;
  }

  for (int i = 0; i < 4; i++) {
    motor_outputs_[i] = 0.0f;
  }
}

float AngleController::clamp(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}
