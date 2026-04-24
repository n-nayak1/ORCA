#pragma once

#include "filter.hpp"

struct AnglePidAxis {
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float integral_limit;
  float d_filtered;
};

class AngleController {
public:
  AngleController();

  void compute(euler_t target, euler_t current, float throttle, float dt);

  void updateGains(float pP, float pI, float pD,
                   float rP, float rI, float rD,
                   float yP, float yI, float yD);

  void resetIntegrals();

  float getMotorOutput(int index) const { return motor_outputs_[index]; }

private:
  float motor_outputs_[4];
  AnglePidAxis pitch_pid_;
  AnglePidAxis roll_pid_;
  AnglePidAxis yaw_pid_;

  float runPid(AnglePidAxis& pid, float error, float dt);
  static float clamp(float val, float min, float max);
  static float wrapAngle180(float angle);
};
