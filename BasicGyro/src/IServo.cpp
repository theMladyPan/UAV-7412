#include "IServo.h"
#include "esp_log.h"


IServo::IServo(uint8_t pin) {
    _servoIndex = ServoImpl.setupServo(pin);
}

void IServo::set_angle(float angle) {
    long angle_us = static_cast<long>(map(angle, -90, 90, 500, 2500));
    ESP_LOGD("IServo", "Setting servo %d pulse width to %dus", _servoIndex, angle_us);
    ServoImpl.setPosition(_servoIndex, angle_us);
}

void IServo::set_percent(float percent) {
    long percent_us = static_cast<long>(map(percent, 0, 100, 500, 2500));
    ESP_LOGD("IServo", "Setting servo %d pulse width to %dus", _servoIndex, percent_us);
    ServoImpl.setPosition(_servoIndex, percent_us);
}