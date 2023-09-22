#include "Regulator/PIDRegulator.h"


PIDRegulator::PIDRegulator(
    Eigen::Vector3d* feedback, 
    Eigen::Vector3d* setpoint, 
    Eigen::Vector3d* correction
) : RegulatorBase(feedback, setpoint, correction) { };


void PIDRegulator::update() {
    _correction->x() = _pid_roll.correction(_feedback->x(), _setpoint->x());
    _correction->y() = _pid_pitch.correction(_feedback->y(), _setpoint->y());
    _correction->z() = _pid_yaw.correction(_feedback->z(), _setpoint->z());
}


void PIDRegulator::setup(void* pid_params, float min, float max) {
    pid_params_t* params = static_cast<pid_params_t*>(pid_params);
    _pid_roll = PID(*params);
    _pid_pitch = PID(*params);
    _pid_yaw = PID(*params);
    _pid_roll.set_limits(min, max);
    _pid_pitch.set_limits(min, max);
    _pid_yaw.set_limits(min, max);
}