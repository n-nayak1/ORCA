#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <array>
#include "filter.hpp" // For euler_t

struct PIDAxis {
    float Kp, Ki, Kd;
    float integral;
    float previous_error;
    float integral_limit;
    float d_filtered;
};

class Controller {
public:
    Controller();

    void compute(euler_t target, euler_t current, float throttle, float dt);

    void updateGains(float pP, float pI, float pD,
                     float rP, float rI, float rD,
                     float yP, float yI, float yD);

    void resetIntegrals();

    float getMotorOutput(int index) const { return motor_outputs[index]; }

    float getKp(int axis) const {
        if(axis == 0) return pitchPID.Kp;
        if(axis == 1) return rollPID.Kp;
        return yawPID.Kp;
    }

    float getKi(int axis) const {
        if(axis == 0) return pitchPID.Ki;
        if(axis == 1) return rollPID.Ki;
        return yawPID.Ki;
    }

    float getKd(int axis) const {
        if(axis == 0) return pitchPID.Kd;
        if(axis == 1) return rollPID.Kd;
        return yawPID.Kd;
    }

private:
    float motor_outputs[4]; // FL, FR, RL, RR
    PIDAxis pitchPID, rollPID, yawPID;

    float runPID(PIDAxis &pid, float error, float dt);
    float clamp(float val, float min, float max);
    float wrapAngle180(float angle);
};

#endif