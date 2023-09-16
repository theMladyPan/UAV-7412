#include "utils.h"

void calculateRotationMatrix(
        const Eigen::Vector3f& v, 
        const Eigen::Vector3f& w, 
        Eigen::Matrix3f& R) {
    Eigen::Vector3f u = v.cross(w);
    u = u.normalized();
    
    float cos_theta = v.normalized().dot(w.normalized());
    float sin_theta = std::sqrt(1 - cos_theta * cos_theta);

    R = Eigen::Matrix3f::Identity() * cos_theta +
        (1 - cos_theta) * u * u.transpose() +
        sin_theta * (Eigen::Matrix3f() << 0, -u(2), u(1),
                                        u(2), 0, -u(0),
                                        -u(1), u(0), 0).finished();
}