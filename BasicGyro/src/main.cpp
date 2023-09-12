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

#define MOCKUP

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

float rnd(float min = -1, float max = 1) {
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
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

class VectorOperations {
public:
    static void axesToVector(std::array<float, 3> &vec, float ax, float ay, float az) {
        vec[0] = ax;
        vec[1] = ay;
        vec[2] = az;
    }

    static float vectorLength(std::array<float, 3> &vec) {
        return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    }
};

class BaseIMU { 
public:
    virtual void getRotations(std::array<float, 3> &rotations) = 0;
    virtual void getAccelerations(std::array<float, 3> &accelerations) = 0;
};

class IMU: public BMI160GenClass, public BaseIMU {
public:
    IMU() { 
        ESP_LOGD("IMU", "Initializing IMU");
        this->begin(BMI160GenClass::I2C_MODE, Wire, 0x68, 0);
        ESP_LOGD("IMU", "Getting device ID");
        uint8_t dev_id = this->getDeviceID();

        ESP_LOGI("IMU", "Device ID: %d", dev_id);

        // Set the accelerometer range to 250 degrees/second
        ESP_LOGD("IMU", "Setting accelerometer range ...");
        this->setFullScaleGyroRange(BMI160_GYRO_RANGE_1000);
        ESP_LOGD("IMU", "Setting accelerometer range done.");
        this->setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
    }
    void getRotations(std::array<float, 3> &rotations) {
        int gxRaw, gyRaw, gzRaw;         // raw gyro values
        this->readGyro(gxRaw, gyRaw, gzRaw);
        rotations[0] = convertRawGyro(gxRaw);
        rotations[1] = convertRawGyro(gyRaw);
        rotations[2] = convertRawGyro(gzRaw);
    }

    void getAccelerations(std::array<float, 3> &accelerations) {
        int axRaw, ayRaw, azRaw;         // raw gyro values
        this->readAccelerometer(axRaw, ayRaw, azRaw);
        accelerations[0] = convertRawAccel(axRaw);
        accelerations[1] = convertRawAccel(ayRaw);
        accelerations[2] = convertRawAccel(azRaw);
    }
};

class MockupIMU: public BaseIMU {
private:
    std::array<float, 3> _rotations = {0, 0, 0};
    std::array<float, 3> _accelerations = {0, 0, 0};
public:
    void getRotations(std::array<float, 3> &rotations) {
        // get some random values in range (-10, 10) and add to the base values,
        rotations[0] = _rotations[0] + rnd();
        rotations[1] = _rotations[1] + rnd();
        rotations[2] = _rotations[2] + rnd();
        // then update the base values
        std::copy(rotations.begin(), rotations.end(), _rotations.begin());
        
    }

    void getAccelerations(std::array<float, 3> &accelerations) {
        // do the same for accelerations
        accelerations[0] = _accelerations[0] + rnd() / 10.0;
        accelerations[1] = _accelerations[1] + rnd() / 10.0;
        accelerations[2] = _accelerations[2] + rnd() / 10.0;
        std::copy(accelerations.begin(), accelerations.end(), _accelerations.begin());
    }
};


class Orientation {
private:
    std::array<float, 3> _orientation;
public:
    Orientation() { 
        _orientation[0] = 0;
        _orientation[1] = 0;
        _orientation[2] = 0;
    } 
    void setRoll(float roll) {
        _orientation[0] = roll;
    }   
    void setPitch(float pitch) {
        _orientation[1] = pitch;
    }
    void setYaw(float yaw) {
        _orientation[2] = yaw;
    }
    float getRoll() {
        return _orientation[0];
    }
    float getPitch() {
        return _orientation[1];
    }
    float getYaw() {
        return _orientation[2];
    }
};

class Aircraft {
private:
    std::array<float, 3> _accelerometer;
    std::array<float, 3> _gyro;
    std::array<float, 3> _accelerations;
    Orientation _orientation;
    BaseIMU *_imu;
public:
    Aircraft(BaseIMU *imu) { 
        _imu = imu;
    }
    void calculate(std::array<float, 3> _desiredOrientation) {
        // read raw gyro measurements from device
        _imu->getRotations(_gyro);
        // read raw accelerometer measurements from device
        _imu->getAccelerations(_accelerometer);

        float len = VectorOperations::vectorLength(_accelerometer);
        // convert to unit vector
        for (auto &v : _accelerometer) {
            v /= len;
        }
        len = VectorOperations::vectorLength(_accelerometer);  // should be 1
    }
    
};


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
}

void loop() {
    std::array<float, 3> accelerations;
    std::array<float, 3> rotations;
    Orientation O;
    std::array<float, 3> desiredOrientation;

    #ifdef MOCKUP
    MockupIMU Imu;
    #else
    IMU Imu;    
    #endif // MOCKUP
    Aircraft aircraft(*Imu);

    uint64_t iter = 0;

    IServo servoTaileronL(PIN_SERVO_TAILERON_L);
    IServo servoTaileronR(PIN_SERVO_TAILERON_R);
    IServo servoRudder(PIN_SERVO_RUDDER);
    IServo throttle(PIN_THROTTLE);

    while(1) {
        auto start = std::chrono::high_resolution_clock::now();
        desiredOrientation[0] = IBus.readChannel(0);
        desiredOrientation[1] = IBus.readChannel(1);
        desiredOrientation[2] = IBus.readChannel(3);
        float desiredThrottle = IBus.readChannel(2); 

        aircraft.calculate(desiredOrientation);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        int dt = duration.count();
        if (dt < 1000) {
            delayMicroseconds(1000 - dt);
        }
        if (iter++ % 100 == 0) {
            // ESP_LOGI("main", "Angle: " << angle_x << ", " << corr_x << ", dt: " << dt << "s");
            ESP_LOGI("main", "Angle: %f°, %f°, dt: %dus", angle_x, corr_x, dt);
            ESP_LOGD("main", "Accelerations: %f, %f, %f", accelerations[0], accelerations[1], accelerations[2]);
        }
    }
}
