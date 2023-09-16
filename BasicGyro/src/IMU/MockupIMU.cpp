#include "IMU/MockupIMU.h"

float rnd(float min, float max) {
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

void MockupIMU::get_rotations_dps(Eigen::Vector3f &rotations) {
    // get some random values in range (-10, 10) and add to the base values,
    rotations[0] = _rotations[0] + rnd();
    rotations[1] = _rotations[1] + rnd();
    rotations[2] = _rotations[2] + rnd();
    // then update the base values
    std::copy(rotations.begin(), rotations.end(), _rotations.begin());
}

void MockupIMU::get_accelerations_g(Eigen::Vector3f &accelerations) {
    // do the same for accelerations
    accelerations[0] = _accelerations[0] + rnd() / 10.0;
    accelerations[1] = _accelerations[1] + rnd() / 10.0;
    accelerations[2] = _accelerations[2] + rnd() / 10.0;
    std::copy(accelerations.begin(), accelerations.end(), _accelerations.begin());
}

void MockupIMU::calibrate(uint n_samples) {
    // TODO
}

