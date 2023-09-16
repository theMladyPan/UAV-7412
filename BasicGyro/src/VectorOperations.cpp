#include "VectorOperations.h"


float VectorOperations::get_vector_length(Eigen::Vector3f &vec) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

void VectorOperations::to_euler_angles(const Eigen::Vector3f& vec, float& roll, float& pitch) {
    roll = std::atan2(vec[1], vec[2]);
    pitch = std::atan2(-vec[0], std::sqrt(vec[1] * vec[1] + vec[2] * vec[2]));
    // yaw = std::atan2(vec[1], vec[0]);  // Or set to 0 or another suitable value
}