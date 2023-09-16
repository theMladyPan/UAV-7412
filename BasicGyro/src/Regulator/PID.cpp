#include "Regulator/PID.h"


PIDRegulator::PIDRegulator() {
    _pid_roll = new PID(&_feedback->x(), &_correction->x(), &_setpoint->x(), 0, 0, 0, DIRECT);
    _pid_pitch = new PID(&_feedback->y(), &_correction->y(), &_setpoint->y(), 0, 0, 0, DIRECT);
    _pid_yaw = new PID(&_feedback->z(), &_correction->z(), &_setpoint->z(), 0, 0, 0, DIRECT);
}

void PIDRegulator::setup(pid_params_t &params) {
    _pid_roll->SetTunings(params.Kp, params.Ki, params.Kd);
    _pid_roll->SetOutputLimits(-90, 90);
    _pid_roll->SetSampleTime(params.sample_time);

    _pid_pitch->SetTunings(params.Kp, params.Ki, params.Kd);
    _pid_pitch->SetOutputLimits(-90, 90);
    _pid_pitch->SetSampleTime(params.sample_time);

    _pid_yaw->SetTunings(params.Kp, params.Ki, params.Kd);
    _pid_yaw->SetOutputLimits(-90, 90);
    _pid_yaw->SetSampleTime(params.sample_time);
}

void PIDRegulator::update() {
    _pid_roll->Compute();
    _pid_pitch->Compute();
    _pid_yaw->Compute();
}