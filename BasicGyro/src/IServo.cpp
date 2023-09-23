#include "IServo.h"
#include "esp_log.h"

ESP32_ISR_Servo ServoImpl;


bool IRAM_ATTR ESP32_ISR_Servo_Handler(void * timerNo)
{
  ServoImpl.run();

  return true;
}

IServo::IServo(uint8_t pin) {
    _servoIndex = ServoImpl.setupServo(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

void IServo::set_angle(float angle) {
    long angle_deg = static_cast<long>(map(angle, -90, 90, 0, 180));
    ESP_LOGD("IServo", "Setting servo %d to %d", _servoIndex, angle_deg);
    ServoImpl.setPosition(_servoIndex, angle_deg);
}

void IServo::set_percent(float percent) {
    long percent_deg = static_cast<long>(map(percent, 0, 100, 0, 180));
    ESP_LOGD("IServo", "Setting servo %d to %d", _servoIndex, percent_deg);
    ServoImpl.setPosition(_servoIndex, percent_deg);
}