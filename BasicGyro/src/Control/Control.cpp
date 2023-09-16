#include "Control/Control.h"


void Control::get_desired_orientation(
        Eigen::Vector3d &desired_orientation) {
    std::copy(
        _desired_orientation.begin(), 
        _desired_orientation.end(), 
        desired_orientation.begin()
    );
}

void Control::get_desired_throttle(float &desired_throttle) {
    desired_throttle = _desired_throttle;
}