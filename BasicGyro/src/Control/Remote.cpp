#include "Control/Remote.h"


Remote::Remote(){
    _ibus.begin(Serial2,1);
}

void Remote::update() {
    _desired_orientation[0] = pulse_width_to_angle(_ibus.readChannel(0));  // roll
    _desired_orientation[1] = pulse_width_to_angle(_ibus.readChannel(1));  // pitch
    _desired_orientation[2] = pulse_width_to_angle(_ibus.readChannel(3));  // yaw
    _desired_throttle = pulse_width_to_throttle(_ibus.readChannel(2));
}

float Remote::pulse_width_to_angle(uint16_t pulse_width) {
    // 500us = -90°
    // 1500us = 0°
    // 2500us = 90°
    // TODO: measure if this is correct
    return -90 + ((pulse_width - 600) * 180 / 1000);
}

float Remote::pulse_width_to_throttle(uint16_t pulse_width) {
    // 500us = 0
    // 1500us = 0.5
    // 2500us = 1
    // TODO: measure if this is correct
    return (pulse_width - 500) / 2000.0;
}