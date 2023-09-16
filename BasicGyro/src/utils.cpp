#include "utils.h"

void calculateRotationMatrix(
        const Eigen::Vector3d& v, 
        const Eigen::Vector3d& w, 
        Eigen::Matrix3d& R) {
    Eigen::Vector3d u = v.cross(w);
    u = u.normalized();
    
    float cos_theta = v.normalized().dot(w.normalized());
    float sin_theta = std::sqrt(1 - cos_theta * cos_theta);

    R = Eigen::Matrix3d::Identity() * cos_theta +
        (1 - cos_theta) * u * u.transpose() +
        sin_theta * (Eigen::Matrix3d() << 0, -u(2), u(1),
                                        u(2), 0, -u(0),
                                        -u(1), u(0), 0).finished();
}