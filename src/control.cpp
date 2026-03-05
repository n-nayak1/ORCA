#include "control.hpp"
#include <math.h>

Controller::Controller() {
    // Initial scaled gains for 0-100 output range
    pitchPID = {0.15f, 0.005f, 0.04f, 0.0f, 0.0f, 40.0f};
    rollPID  = {0.15f, 0.005f, 0.04f, 0.0f, 0.0f, 40.0f};
    yawPID   = {0.20f, 0.001f, 0.0f, 0.0f, 0.0f, 40.0f};
    
    resetIntegrals();
}

float Controller::runPID(PIDAxis &pid, float error, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Deadband: Ignore tiny errors to stop motor jitter
    if (fabs(error) < 0.5f) error = 0.0f;

    pid.integral += error * dt;
    pid.integral = clamp(pid.integral, -pid.integral_limit, pid.integral_limit);

    float derivative = (error - pid.previous_error) / dt;
    pid.previous_error = error;

    return (pid.Kp * error) + (pid.Ki * pid.integral) + (pid.Kd * derivative);
}

void Controller::compute(euler_t target, euler_t current, float throttle, float dt) {
    // Target clamping: prevent extreme angle requests
    float target_p = clamp(target.pitch_deg, -5.0f, 5.0f);
    float target_r = clamp(target.roll_deg, -5.0f, 5.0f);

    float p_adj = runPID(pitchPID, target_p - current.pitch_deg, dt);
    float r_adj = runPID(rollPID,  target_r - current.roll_deg,  dt);
    float y_adj = runPID(yawPID,   target.yaw_deg - current.yaw_deg, dt);

    // Motor Mixing (X-Config)
    // motor_outputs[0] = throttle + p_adj + r_adj + y_adj; // FL
    // motor_outputs[1] = throttle + p_adj - r_adj - y_adj; // FR
    // motor_outputs[2] = throttle - p_adj + r_adj - y_adj; // RL
    // motor_outputs[3] = throttle - p_adj - r_adj + y_adj; // RR

    motor_outputs[0] = throttle - p_adj - r_adj; // + y_adj; // FL
    motor_outputs[1] = throttle + p_adj - r_adj; // - y_adj; // FR
    motor_outputs[2] = throttle - p_adj + r_adj; // - y_adj; // RL
    motor_outputs[3] = throttle + p_adj + r_adj; // + y_adj; // RR

    // Clamp final output to 0-100% for the Motor class
    for(int i=0; i<4; i++) {
        motor_outputs[i] = clamp(motor_outputs[i], 0.0f, 100.0f);
    }
}

void Controller::updateGains(float pP, float pI, float pD, 
                             float rP, float rI, float rD, 
                             float yP, float yI, float yD) {
    pitchPID.Kp = pP; pitchPID.Ki = pI; pitchPID.Kd = pD;
    rollPID.Kp  = rP; rollPID.Ki  = rI; rollPID.Kd  = rD;
    yawPID.Kp   = yP; yawPID.Ki   = yI; yawPID.Kd   = yD;
    resetIntegrals(); // Reset to prevent jumps when gains change
}

void Controller::resetIntegrals() {
    PIDAxis* axes[3] = {&pitchPID, &rollPID, &yawPID};
    for(int i=0; i<3; i++) {
        axes[i]->integral = 0.0f;
        axes[i]->previous_error = 0.0f;
    }
    for(int i=0; i<4; i++) motor_outputs[i] = 0.0f;
}

float Controller::clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

