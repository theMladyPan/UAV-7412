#include "IServo.h"
#include "esp_log.h"


IServo::IServo(uint8_t pin) {
    _servoIndex = ServoImpl.setupServo(pin);
}

void IServo::set_angle(int angle) {
    ESP_LOGD("IServo", "Setting servo %d angle to %d", _servoIndex, angle);
    ServoImpl.setPosition(_servoIndex, angle + 90);
}