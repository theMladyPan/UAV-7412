#ifndef UTILS_H
#define UTILS_H

#include <ArduinoEigenDense.h>


// Calculates the rotation matrix R that rotates v into w. 
//
// The rotation matrix is the orthogonal matrix that satisfies
// the equation Rv = w.
void calculateRotationMatrix(
        const Eigen::Vector3f& v, 
        const Eigen::Vector3f& w, 
        Eigen::Matrix3f& R);


#endif // UTILS_H