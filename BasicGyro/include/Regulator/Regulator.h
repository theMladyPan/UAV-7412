#pragma once

#include <ArduinoEigenDense.h>

class Regulator {
protected: 
    Eigen::Vector3d* _feedback;
    Eigen::Vector3d* _setpoint;
    Eigen::Vector3d* _correction;

public:
    virtual void update() = 0;
    void set_feedback(Eigen::Vector3d* feedback) { _feedback = feedback; }
    void set_setpoint(Eigen::Vector3d* setpoint) { _setpoint = setpoint; }
    void set_correction(Eigen::Vector3d* correction) { _correction = correction; }
};
