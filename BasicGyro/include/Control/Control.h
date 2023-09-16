#ifndef CONTROL_H
#define CONTROL_H

#include <ArduinoEigenDense.h>

class Control {
protected:
    /**
     * @brief Desired orientation in degrees, roll, pitch, yaw
     */
    Eigen::Vector3f _desired_orientation;
    float _desired_throttle;
public:
    void get_desired_orientation(
            Eigen::Vector3f &desired_orientation);

    void get_desired_throttle(float &desired_throttle);

    virtual void update() = 0;
};


#endif // CONTROL_H