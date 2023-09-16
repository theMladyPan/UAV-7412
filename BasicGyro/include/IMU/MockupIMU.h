#ifndef MOCKUP_IMU_H
#define MOCKUP_IMU_H


#include <ArduinoEigenDense.h>
#include <esp_log.h>
#include "BaseIMU.h"


float rnd(float min = -1, float max = 1);


class MockupIMU: public BaseIMU {
private:
    Eigen::Vector3f _rotations;
    Eigen::Vector3f _accelerations;
public:
    MockupIMU(): _rotations(0, 0, 0), _accelerations(0, 0, 0) {}
    
    // Gets the current rotations in degrees per second (dps).
    // The rotations are stored in the passed parameter.
    // The rotations are in the format (roll, pitch, yaw).
    void get_rotations_dps(Eigen::Vector3f &rotations);

    // Gets the current accelerations in g's.
    // The accelerations are stored in the passed parameter.
    // The accelerations are in the format (x, y, z).
    void get_accelerations_g(Eigen::Vector3f &accelerations);

    // Calibrate the IMU by taking the average of the first n_samples readings.
    void calibrate(uint n_samples);
};


#endif // MOCKUP_IMU_H