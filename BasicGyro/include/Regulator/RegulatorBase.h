#pragma once

#include <ArduinoEigenDense.h>

class RegulatorBase {
protected: 
    Eigen::Vector3d* _feedback;
    Eigen::Vector3d* _setpoint;
    Eigen::Vector3d* _correction;

public:
    RegulatorBase() {};
    RegulatorBase(Eigen::Vector3d* feedback, Eigen::Vector3d* setpoint, Eigen::Vector3d* correction) {
        _feedback = feedback;
        _setpoint = setpoint;
        _correction = correction;
    }

    virtual void update() = 0;
    void set_feedback(Eigen::Vector3d* feedback) { _feedback = feedback; }
    void set_setpoint(Eigen::Vector3d* setpoint) { _setpoint = setpoint; }
    void set_correction(Eigen::Vector3d* correction) { _correction = correction; }
};
