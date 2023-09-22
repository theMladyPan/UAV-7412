#ifndef REMOTE_H
#define REMOTE_H

#include <ArduinoEigenDense.h>
#include <IBusBM.h>
#include <Control/Control.h>


#define PULSE_MIN 500
#define PULSE_MAX 2500
#define PULSE_NEUTRAL 1500


class Remote : public Control {
private:
    float _svc;
    float _vrb;

    IBusBM _ibus;
    /**
     * @brief Convert pulse width to angle
     * 
     * @param pulse_width from 500 to 2500us
     * @return float angle in degrees from -90 to 90
     */
    float pulse_width_to_angle(uint16_t pulse_width);

    /**
     * @brief Convert pulse width to throttle
     * 
     * @param pulse_width from 500 to 2500us
     * @return float throttle from 0 to 100%
     */
    float pulse_width_to_throttle(uint16_t pulse_width);

    /**
     * @brief Failsafe for pulse width
     * 
     * @param value pulse width
     * @param failsafe_val value to return if value is out of range, default is PULSE_NEUTRAL
     * @param min minimum value
     * @param max maximum value
     * @return float in range min to max, or failsafe_val if value is out of range
     */
    float failsafe(float value, float failsafe_val, float min, float max);

public:
    Remote();

    void update();

    // SVC:1 = 0, SVC:2 = 50, SVC:3 = 100
    float get_switch();

    // VRB = 0 to 100
    float get_potentiometer();
};


#endif // REMOTE_H