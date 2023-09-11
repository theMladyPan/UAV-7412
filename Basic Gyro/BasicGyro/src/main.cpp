#include <Arduino.h>
#include <BMI160Gen.h>
#include <PID_v1.h>
#include <math.h>
#include <vector>
#include <chrono>
#include "ESP32_ISR_Servo.h"
#include "esp_log.h"
#include <IBusBM.h>

#ifdef TFT_DISPLAY
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(135, 240);
#endif // TFT_DISPLAY


#define PIN_SERVO_TAILERON_L 33
#define PIN_SERVO_TAILERON_R 26
#define PIN_SERVO_RUDDER 25
#define PIN_THROTTLE 32

#define MIN_MICROS 500
#define MAX_MICROS 2500

double Kp=1, Ki=10, Kd=0;

//Define Variables we'll be connecting to
double angle_x_set, angle_x, corr_x;
PID pid_roll(&angle_x, &corr_x, &angle_x_set, Kp, Ki, Kd, DIRECT);
double angle_y_set, angle_y, corr_y;
PID pid_pitch(&angle_y, &corr_y, &angle_y_set, Kp, Ki, Kd, DIRECT);
double angle_z_set, angle_z, corr_z;
PID pid_yaw(&angle_z, &corr_z, &angle_z_set, Kp, Ki, Kd, DIRECT);

IBusBM IBus;    // IBus object

float convertRawGyro(int gRaw) {
    float g = (gRaw * 1000.0) / 32768.0;
    return g;
}

float convertRawAccel(int aRaw) {
    float a = (aRaw * 2.0) / 32768.0;
    return a;
}

void setupPids() {
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

class IServo {
public:
    IServo() { }
    IServo(uint8_t pin) {

	    _servoIndex = ESP32_ISR_Servos.setupServo(pin, MIN_MICROS, MAX_MICROS);
        
    }
    void setAngle(int angle) {
        ESP32_ISR_Servos.setPosition(_servoIndex, angle + 90);
    }

private:
    int8_t _servoIndex;
};

void axesToVector(std::array<float, 3> &vec, float ax, float ay, float az) {
    vec[0] = ax;
    vec[1] = ay;
    vec[2] = az;
}

float vectorLength(std::array<float, 3> &vec) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

void readGyro(std::array<float, 3> &rotations) {
    int gxRaw, gyRaw, gzRaw;         // raw gyro values
    BMI160.readGyro(gxRaw, gyRaw, gzRaw);
    rotations[0] = convertRawGyro(gxRaw);
    rotations[1] = convertRawGyro(gyRaw);
    rotations[2] = convertRawGyro(gzRaw);
}

void readAccelerometer(std::array<float, 3> &accelerations) {
    int axRaw, ayRaw, azRaw;         // raw gyro values
    BMI160.readAccelerometer(axRaw, ayRaw, azRaw);
    accelerations[0] = convertRawAccel(axRaw);
    accelerations[1] = convertRawAccel(ayRaw);
    accelerations[2] = convertRawAccel(azRaw);
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

	ESP32_ISR_Servos.useTimer(0);
    IBus.begin(Serial2,1);

    ESP_LOGI("main", "Starting BMI160");

    ESP_LOGD("main", "Initializing IMU");
    BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x68, 0);
    ESP_LOGD("main", "Getting device ID");
    uint8_t dev_id = BMI160.getDeviceID();

    ESP_LOGI("main", "Device ID: %d", dev_id);

    // Set the accelerometer range to 250 degrees/second
    ESP_LOGD("main", "Setting accelerometer range ...");
    BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_1000);
    ESP_LOGD("main", "Setting accelerometer range done.");
    BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
}

void loop() {
    std::array<float, 3> accelerations;
    std::array<float, 3> rotations;

    uint64_t iter = 0;

    IServo servoTaileronL(PIN_SERVO_TAILERON_L);
    IServo servoTaileronR(PIN_SERVO_TAILERON_R);
    IServo servoRudder(PIN_SERVO_RUDDER);
    IServo throttle(PIN_THROTTLE);

    while(1) {
        auto start = std::chrono::high_resolution_clock::now();
        // read raw gyro measurements from device
        readGyro(rotations);
        // read raw accelerometer measurements from device
        readAccelerometer(accelerations);
        auto val = IBus.readChannel(0);

        float len = vectorLength(accelerations);
        // convert to unit vector
        for (auto &v : accelerations) {
            v /= len;
        }
        len = vectorLength(accelerations);  // should be 1

        // ESP_LOGI("main", "BMI160", "Accel: %f, %f, %f", ax, ay, az);
        /*
        angle_x_set = 0;
        if(ay >= 1.0) ay = 1.0;
        if(ay <= -1.0) ay = -1.0;
        float asiny = asin(ay);
        angle_x = asin(ay) * -180 / M_PI;
        pid_roll.Compute();

        float rotationX = gx * 0.001;  // 1000˚/s -> 1˚/ms

        servoTaileronL.setAngle(-angle_x + rotationX*50);
        */

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        int dt = duration.count();
        if (dt < 1000) {
            delayMicroseconds(1000 - dt);
        }
        if (iter++ % 100 == 0) {
            // ESP_LOGI("main", "Angle: " << angle_x << ", " << corr_x << ", dt: " << dt << "s");
            ESP_LOGI("main", "Angle: %f, %f, dt: %d", angle_x, corr_x, dt);
            // print vector length
            ESP_LOGI("main", "Vector length: %f", len);
        }
    }
}
