#include <Arduino.h>
#include <ESP32_ISR_Servo.h>
#include <chrono>
#include "esp_log.h"
#include <iostream>
#include "Aircraft.h"
#include "Control/Control.h"
#include "Control/Autopilot.h"
#include "Control/Remote.h"
#include "IMU/IMU.h"
#include "IMU/MockupIMU.h"
#include "Regulator/PID.h"


#ifdef TFT_DISPLAY
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(135, 240);
#endif // TFT_DISPLAY

#define PIN_SERVO_TAILERON_L 33
#define PIN_SERVO_TAILERON_R 26
#define PIN_SERVO_RUDDER 25
#define PIN_THROTTLE 32

#define LOOP_FREQ_HZ 1
#define LOOP_PERIOD_US (1000000 / LOOP_FREQ_HZ)


void setup() {
    Serial.begin(1000000);
    Wire.begin(I2C_SDA, I2C_SCL, 1000000); // join i2c bus (address optional for master
    #ifdef TFT_DISPLAY
    tft.init();
    tft.setTextFont(1);
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Hello", 0, 50, 4);
    #endif // TFT_DISPLAY
}

void loop() {
    // MockupIMU *Imu = new MockupIMU();
    IMU *Imu = new IMU();

    // Calibrate the IMU as a part of the startup procedure
    Imu->calibrate(100);

    PIDRegulator *regulator = new PIDRegulator();
    pid_params_t pid_params = {
        .Kp = 1,
        .Ki = 10,
        .Kd = 0,
        .sample_time = 1  // ms
    };
    
    regulator->setup(pid_params);

    // Setup the aircraft
    aircraft_param_t aircraft_params;
    aircraft_params.invert_taileron_left = true;
    aircraft_params.loop_period_us = LOOP_PERIOD_US;

    Aircraft aircraft(
        Imu,
        regulator,
        aircraft_params
    );

    aircraft.setup(
        PIN_SERVO_TAILERON_L,
        PIN_SERVO_TAILERON_R,
        PIN_SERVO_RUDDER,
        PIN_THROTTLE
    );

    // Control *controller = new Autopilot();
    Control *controller = new Remote();
    
    delay(1e3);

    aircraft.pre_flight_check();
    delay(1e3);

    while(1) {
        auto start = std::chrono::high_resolution_clock::now();

        aircraft.update();
        aircraft.calculate_corrections(controller);
        aircraft.steer();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        int dt = duration.count();
        if (dt < LOOP_PERIOD_US) {
            delayMicroseconds(LOOP_PERIOD_US - dt);
        }
    }
}
