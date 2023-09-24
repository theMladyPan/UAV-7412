#ifndef ISERVO_H
#define ISERVO_H

#include <stdint.h>

#include "ESP32Servo.h"


/**
 * @brief Servo class 
 */
class IServo {
public:
    IServo() { };

    /**
     * @brief Construct a new IServo object
     * 
     * @param pin attached to servo
     */
    IServo(uint8_t pin);

    /**
     * @brief Set the angle in degrees
     * 
     * @param angle -90°, 90° -> 500, 2500us
     */
    void set_angle(float angle);

    /**
     * @brief Set the angle in degrees
     * 
     * @param angle -90°, 90° -> 500, 2500us
     * @param angle_min minimum angle in degrees
     * @param angle_max maximum angle in degrees     * 
     */
    void set_angle(float angle, float angle_min, float angle_max);

    void set_percent(float percent);

private:
    Servo ServoImpl;
};

#endif // ISERVO_H