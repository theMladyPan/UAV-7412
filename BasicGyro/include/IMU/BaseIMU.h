#ifndef BASE_IMU_H
#define BASE_IMU_H

#include <ArduinoEigenDense.h>
#include <esp_log.h>


/**
 * @brief Base class for IMU
 * 
 * @details This class is used to define interface for IMU classes. 
 */
class BaseIMU { 
protected:
    Eigen::Matrix3d _base_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d _gyro_null_val;
    float _acc_gain = 1;
public:
    virtual void get_rotations_dps(Eigen::Vector3d &rotations) = 0;
    virtual void get_accelerations_g(Eigen::Vector3d &accelerations) = 0;
    virtual void calibrate(uint n_samples) = 0;
};

#endif // BASE_IMU_H