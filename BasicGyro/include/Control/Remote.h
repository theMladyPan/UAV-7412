#ifndef REMOTE_H
#define REMOTE_H

#include <ArduinoEigenDense.h>
#include <IBusBM.h>
#include <Control/Control.h>


class Remote : public Control {
private:
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
public:
    Remote();

    void update();
};


#endif // REMOTE_H