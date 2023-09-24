#include "IServo.h"
#include "esp_log.h"
#include "utils.h"


IServo::IServo(uint8_t pin) {
    ServoImpl.attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

void IServo::set_angle(float angle) {
    int angle_us = static_cast<int>(mapf(angle, -90, 90, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    ESP_LOGD("IServo", "Setting servo %d to %dus", _servoIndex, angle_us);
    ServoImpl.write(angle_us);
}

void IServo::set_angle(float angle, float angle_min, float angle_max) {
    if(angle < angle_min) {
        angle = angle_min;
    } else if(angle > angle_max) {
        angle = angle_max;
    }
    int angle_us = static_cast<int>(mapf(angle, -90, 90, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    ESP_LOGD("IServo", "Setting servo %d to %dus", _servoIndex, angle_us);
    ServoImpl.write(angle_us);
}

void IServo::set_percent(float percent) {
    int percent_us = static_cast<int>(mapf(percent, 0, 100, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    ESP_LOGD("IServo", "Setting servo %d to %dus", _servoIndex, percent_us);
    ServoImpl.write(percent_us);
}