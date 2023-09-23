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


float map(float x, float in_min, float in_max, float out_min, float out_max) {
    float run = in_max - in_min;
    if(run < 0.0f){
        ESP_LOGE("map", "Invalid input range: min > max");
        return -INFINITY;
    }
    float rise = out_max - out_min;
    float delta = x - in_min;
    return (delta * rise) / run + out_min;
}
