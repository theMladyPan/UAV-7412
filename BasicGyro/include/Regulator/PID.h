#pragma once

#include "Regulator/Regulator.h"
#include <PID_v1.h>

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    uint16_t sample_time;
} pid_params_t;


class PIDRegulator : public Regulator {
private:
    PID* _pid_roll;
    PID* _pid_pitch;
    PID* _pid_yaw;

public: 
    PIDRegulator();    

    void setup(pid_params_t &params);

    void update();

};