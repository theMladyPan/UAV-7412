#pragma once

#include <stdint.h>

typedef struct {
    double kp;  // proportional gain
    double ki;  // integral gain
    double kd;  // derivative gain
    double sampling_period;  // seconds
} pid_params_t;


class PID {
private:
    pid_params_t _params;
    double _last_error;
    double _min;
    double _max;
    double _integral;

public: 
    PID();

    PID(pid_params_t params);

    void set_limits(double min, double max);

    double correction(double feedback, double setpoint);
};