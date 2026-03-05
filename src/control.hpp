#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "filter.hpp" // For euler_t

struct PIDAxis {
    float Kp, Ki, Kd;
    float integral;
    float previous_error;
    float integral_limit; 
};

class Controller {
public:
    Controller();

    // Main control loop: inputs 0-100 throttle, outputs 0-100 motor power
    void compute(euler_t target, euler_t current, float throttle, float dt);

    // Live tuning for the web dashboard
    void updateGains(float pP, float pI, float pD, 
                     float rP, float rI, float rD, 
                     float yP, float yI, float yD);

    // Safety: Clears stored error to prevent jumps during takeoff
    void resetIntegrals();

    float getMotorOutput(int index) const { return motor_outputs[index]; }

    // Getters for live monitoring
    float getKp(int axis) { 
        if(axis == 0) return pitchPID.Kp; 
        if(axis == 1) return rollPID.Kp; 
        return yawPID.Kp; 
    }
    float getKi(int axis) { 
        if(axis == 0) return pitchPID.Ki; 
        if(axis == 1) return rollPID.Ki; 
        return yawPID.Ki; 
    }
    float getKd(int axis) { 
        if(axis == 0) return pitchPID.Kd; 
        if(axis == 1) return rollPID.Kd; 
        return yawPID.Kd; 
    }

private:
    float motor_outputs[4]; // FL, FR, RL, RR
    PIDAxis pitchPID, rollPID, yawPID;

    float runPID(PIDAxis &pid, float error, float dt);
    float clamp(float val, float min, float max);
};

#endif