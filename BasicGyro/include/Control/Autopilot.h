#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <ArduinoEigenDense.h>
#include "Control/Control.h"


class Autopilot : public Control {
private:
    Eigen::Vector3f _desired_orientation;
public:
    Autopilot() { }

    void update();
};

#endif // AUTOPILOT_H