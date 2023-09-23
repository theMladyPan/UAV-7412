#include <Arduino.h>
#include <chrono>
#include "esp_log.h"
#include <iostream>
#include "Aircraft.h"
#include "Control/Control.h"
#include "Control/Autopilot.h"
#include "Control/Remote.h"
#include "IMU/IMU.h"
#include "IMU/MockupIMU.h"
#include "Regulator/PIDRegulator.h"
#include "IServo.h"


#ifdef TFT_DISPLAY
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(135, 240);
#endif // TFT_DISPLAY

#define PIN_SERVO_TAILERON_L 33
#define PIN_SERVO_TAILERON_R 25
#define PIN_THROTTLE 26
#define PIN_SERVO_RUDDER 27

#define LOOP_FREQ_HZ 200.0
#define LOOP_PERIOD_US (1e6 / LOOP_FREQ_HZ)

#define SERVO_ANGLE_MIN -45
#define SERVO_ANGLE_MAX 45

#ifdef I2C_SDA
#undef I2C_SDA
#define I2C_SDA 19
#endif // I2C_SDA


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
    ESP_LOGI("main", "Calibrating IMU");
    Imu->calibrate(100);

    pid_params_t pid_params = {
        .kp = 1,
        .ki = 1,
        .kd = 0,
        .sampling_period = LOOP_PERIOD_US / 1e6  // 1s
    };

    // Setup the aircraft
    aircraft_param_t aircraft_params;
    aircraft_params.invert_taileron_right = true;
    aircraft_params.control_rudder = true;
    aircraft_params.control_throttle = true;
    aircraft_params.loop_period_us = LOOP_PERIOD_US;
    aircraft_params.angle_min = SERVO_ANGLE_MIN;
    aircraft_params.angle_max = SERVO_ANGLE_MAX;

    ESP_LOGI("main", "Creating aircraft");
    Aircraft<PIDRegulator> aircraft(
        Imu,
        aircraft_params
    );
    aircraft.setup_regulator(&pid_params);

    ESP_LOGI("main", "Setting up aircraft");
    
    aircraft.setup(
        PIN_SERVO_TAILERON_L,
        PIN_SERVO_TAILERON_R,
        PIN_SERVO_RUDDER,
        PIN_THROTTLE
    );

    // Control *controller = new Autopilot();
    ESP_LOGI("main", "Creating controller");
    Remote *controller = new Remote();    
    ESP_LOGI("main", "Initiating pre-flight check");
    aircraft.pre_flight_check();  // Turn on after testing

    uint64_t loopn = 0;
    ESP_LOGI("main", "Entering main loop");
    while(1) {
        auto start = std::chrono::high_resolution_clock::now();

        aircraft.update();
        aircraft.calculate_corrections(controller);
        aircraft.steer();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        int dt = duration.count();

        if(loopn++ % 100 == 0) {
            aircraft.print_status();
            std::cout << "Loop duration: " << dt << " us" << std::endl << std::endl;
        }
            
        if (dt < LOOP_PERIOD_US) {
            delayMicroseconds(LOOP_PERIOD_US - dt);
        }
    }
}
