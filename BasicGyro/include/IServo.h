#ifndef ISERVO_H
#define ISERVO_H

#include <stdint.h>
#include "ESP32_ISR_Servo.hpp"  // This must be included first and only in main.cpp

static ESP32_ISR_Servo ServoImpl;

/**
 * @brief Servo class that uses ESP32_ISR_Servo library * 
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
     * @param angle -90°, 90°
     */
    void set_angle(int angle);

private:
    int8_t _servoIndex;
};

#endif // ISERVO_H