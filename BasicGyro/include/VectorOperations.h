#ifndef VECTOR_OPERATIONS_H
#define VECTOR_OPERATIONS_H

#include <ArduinoEigenDense.h>
#include <cmath>


class VectorOperations {
public:
    /**
     * @brief Get the vector length from vector
     * 
     * @param vec - 3 dimensional vector
     * @return float - vector length
     */
    static float get_vector_length(Eigen::Vector3f &vec);

    /**
     * @brief Convert vector to euler angles
     * 
     * @param vec - 3 dimensional vector
     * @param roll - roll angle in radians
     * @param pitch - pitch angle in radians
     */
    static void to_euler_angles(const Eigen::Vector3f& vec, float& roll, float& pitch);
};

#endif // VECTOR_OPERATIONS_H