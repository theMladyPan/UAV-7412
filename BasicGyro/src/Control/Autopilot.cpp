#include "Control/Autopilot.h"


void Autopilot::update() {
    _desired_orientation[0] = 0;
    _desired_orientation[1] = 0;
    _desired_orientation[2] = 0;
    _desired_throttle = 0;
}