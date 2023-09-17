#ifndef UTILS_H
#define UTILS_H

#include <ArduinoEigenDense.h>


// Calculates the rotation matrix R that rotates v into w. 
//
// The rotation matrix is the orthogonal matrix that satisfies
// the equation Rv = w.
void calculateRotationMatrix(
        const Eigen::Vector3d& v, 
        const Eigen::Vector3d& w, 
        Eigen::Matrix3d& R);


/**
 * @brief Maps an input value within a given range to an output value within another range.
 * 
 * @param x The value to map.
 * @param in_min The minimum value of the input range.
 * @param in_max The maximum value of the input range.
 * @param out_min The minimum value of the output range.
 * @param out_max The maximum value of the output range.
 * @return float The mapped value, or -1.0 if the input range is invalid (i.e., min == max).
 */
float map(float x, float in_min, float in_max, float out_min, float out_max);

#endif // UTILS_H