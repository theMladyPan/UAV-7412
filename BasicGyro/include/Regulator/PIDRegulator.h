#pragma once

#include <ArduinoEigenDense.h>
#include "Regulator/RegulatorBase.h"
#include "Regulator/PID.h"

class PIDRegulator : public RegulatorBase {
private:
    PID _pid_roll;
    PID _pid_pitch;
    PID _pid_yaw;
public:
    PIDRegulator(
        Eigen::Vector3d* feedback, 
        Eigen::Vector3d* setpoint, 
        Eigen::Vector3d* correction
    );

    void update();

    void setup(void* pid_params, float min, float max);
};