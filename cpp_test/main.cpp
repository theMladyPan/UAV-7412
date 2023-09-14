#include <Eigen/Dense>
#include <iostream>

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


int main() {
    Eigen::Vector3f v(1.0, 2.0, 3.0);
    Eigen::Vector3f w(0.0, 0.0, 1.0);

    Eigen::Matrix3f R;
    calculateRotationMatrix(v, w, R);

    std::cout << "Rotation matrix:\n" << R << std::endl;

    Eigen::Vector3f u = R * v;

    std::cout << "One rotation:\n" << u << std::endl;

    u = R * u;

    std::cout << "Two rotations:\n" << u << std::endl;

    Eigen::Vector3f v1(1.0, 0, 0);
    calculateRotationMatrix(v1, w, R);
    std::cout << "Rotation matrix:\n" << R << std::endl;
    u = R * R * v1;
    std::cout << "Test - u:\n" << u << std::endl;

    return 0;
}