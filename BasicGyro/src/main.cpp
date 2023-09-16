#include <Arduino.h>
#include <ESP32_ISR_Servo.h>
#include <PID_v1.h>
#include <chrono>
#include "esp_log.h"
#include <iostream>
#include "Aircraft.h"
#include "Control/Control.h"
#include "Control/Autopilot.h"
#include "Control/Remote.h"
#include "IMU/IMU.h"
#include "IMU/MockupIMU.h"


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

double Kp=1, Ki=10, Kd=0;

//Define Variables we'll be connecting to
double angle_x_set, angle_x, corr_x;
PID pid_roll(&angle_x, &corr_x, &angle_x_set, Kp, Ki, Kd, DIRECT);
double angle_y_set, angle_y, corr_y;
PID pid_pitch(&angle_y, &corr_y, &angle_y_set, Kp, Ki, Kd, DIRECT);
double angle_z_set, angle_z, corr_z;
PID pid_yaw(&angle_z, &corr_z, &angle_z_set, Kp, Ki, Kd, DIRECT);


void setup_PIDs() {
    //turn the PID on
    pid_roll.SetMode(AUTOMATIC);
    pid_roll.SetSampleTime(1);
    pid_roll.SetOutputLimits(-90, 90);
    pid_pitch.SetMode(AUTOMATIC);
    pid_pitch.SetSampleTime(1);
    pid_pitch.SetOutputLimits(-90, 90);
    pid_yaw.SetMode(AUTOMATIC);
    pid_yaw.SetSampleTime(1);
    pid_yaw.SetOutputLimits(-90, 90);
}


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
    #ifdef MOCKUP
    MockupIMU *Imu = new MockupIMU();
    #else
    IMU *Imu = new IMU();    
    #endif // MOCKUP

    // Calibrate the IMU as a part of the startup procedure
    Imu->calibrate(100);

    // Setup the aircraft
    aircraft_param_t params;
    params.invert_taileron_left = true;

    Aircraft aircraft(
        Imu,
        params
    );

    aircraft.setup(
        PIN_SERVO_TAILERON_L,
        PIN_SERVO_TAILERON_R,
        PIN_SERVO_RUDDER,
        PIN_THROTTLE
    );

    #ifdef AUTOPILOT
    Control *controller = new Autopilot();
    #else
    Control *controller = new Remote();
    #endif // AUTOPILOT
    
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
