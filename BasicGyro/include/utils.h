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


#endif // UTILS_H