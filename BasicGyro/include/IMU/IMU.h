#ifndef IMU_H
#define IMU_H

#include <BMI160Gen.h>
#include <ArduinoEigenDense.h>
#include "BaseIMU.h"
#include "utils.h"

class IMU: public BMI160GenClass, public BaseIMU {
private:
    /**
     * @brief Read raw gyro values from device and convert to degrees per second
     * 
     * @param gRaw 
     * @return double 
     */
    double convert_raw_gyro(int gRaw);

    /**
     * @brief Read raw accelerometer values from device and convert to g
     * 
     * @param aRaw 
     * @return double 
     */
    double convert_raw_accel(int aRaw);
    
public:
    /**
     * @brief Construct a new IMU object
     * 
     */
    IMU();

    /**
     * @brief Get the rotations in degrees per second
     * 
     * @param rotations vector (roll, pitch yaw)
     */
    void get_rotations_dps(Eigen::Vector3d &rotations);

    /**
     * @brief Get the accelerations g vector (x, y, z)
     * 
     * @param accelerations vector (x, y, z)
     */
    void get_accelerations_g(Eigen::Vector3d &accelerations);

    void calibrate(uint n_samples);
};

#endif // IMU_H