#include "control.hpp"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Controller::Controller() {
    pitchPID = {0.06f, 0.000f, 0.065f, 0.0f, 0.0f, 40.0f, 0.0f};
    rollPID  = {0.07f, 0.000f, 0.065f, 0.0f, 0.0f, 40.0f, 0.0f};
    yawPID   = {0.005f, 0.000f, 0.000f, 0.0f, 0.0f, 40.0f, 0.0f};

    resetIntegrals();
}

float Controller::runPID(PIDAxis &pid, float error, float dt) {
    if (dt <= 0.0f) return 0.0f;

    if (fabs(error) < 0.5f) error = 0.0f;

    // Integral
    pid.integral += error * dt;
    pid.integral = clamp(pid.integral, -pid.integral_limit, pid.integral_limit);

    // Raw derivative
    float d_raw = (error - pid.previous_error) / dt;
    pid.previous_error = error;

    // 1st-order low-pass filter on derivative
    // Start with cutoff around 15 Hz for 100 Hz loop
    const float cutoff_hz = 15.0f;
    const float tau = 1.0f / (2.0f * (float)M_PI * cutoff_hz);
    const float alpha = tau / (tau + dt);

    pid.d_filtered = alpha * pid.d_filtered + (1.0f - alpha) * d_raw;

    return (pid.Kp * error) +
           (pid.Ki * pid.integral) +
           (pid.Kd * pid.d_filtered);
}

float Controller::wrapAngle180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void Controller::compute(euler_t target, euler_t current, float throttle, float dt) {
    float target_p = clamp(target.pitch_deg, -5.0f, 5.0f);
    float target_r = clamp(target.roll_deg,  -5.0f, 5.0f);

    float pitch_error = target_p - current.pitch_deg;
    float roll_error  = target_r - current.roll_deg;
    float yaw_error   = wrapAngle180(target.yaw_deg - current.yaw_deg);

    float p_adj = runPID(pitchPID, pitch_error, dt);
    float r_adj = runPID(rollPID,  roll_error,  dt);
    float y_adj = runPID(yawPID,   yaw_error,   dt);

    motor_outputs[0] = throttle - p_adj - r_adj; // FL
    motor_outputs[1] = throttle + p_adj - r_adj; // FR
    motor_outputs[2] = throttle - p_adj + r_adj; // RL
    motor_outputs[3] = throttle + p_adj + r_adj; // RR

    // Uncomment after verifying yaw motor directions
    // motor_outputs[0] += y_adj;
    // motor_outputs[1] -= y_adj;
    // motor_outputs[2] -= y_adj;
    // motor_outputs[3] += y_adj;

    for (int i = 0; i < 4; i++) {
        motor_outputs[i] = clamp(motor_outputs[i], 0.0f, 100.0f);
    }
}

void Controller::updateGains(float pP, float pI, float pD,
                             float rP, float rI, float rD,
                             float yP, float yI, float yD) {
    pitchPID.Kp = pP; pitchPID.Ki = pI; pitchPID.Kd = pD;
    rollPID.Kp  = rP; rollPID.Ki  = rI; rollPID.Kd  = rD;
    yawPID.Kp   = yP; yawPID.Ki   = yI; yawPID.Kd   = yD;
    resetIntegrals();
}

void Controller::resetIntegrals() {
    PIDAxis* axes[3] = {&pitchPID, &rollPID, &yawPID};
    for (int i = 0; i < 3; i++) {
        axes[i]->integral = 0.0f;
        axes[i]->previous_error = 0.0f;
        axes[i]->d_filtered = 0.0f;
    }

    for (int i = 0; i < 4; i++) {
        motor_outputs[i] = 0.0f;
    }
}

float Controller::clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}
