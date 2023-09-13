#include <Arduino.h>
#include <BMI160Gen.h>
#include <PID_v1.h>
#include <math.h>
#include <vector>
#include <chrono>
#include "ESP32_ISR_Servo.h"
#include "esp_log.h"
#include <IBusBM.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

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

#define LOOP_FREQ_HZ 10
#define LOOP_PERIOD_US (1000000 / LOOP_FREQ_HZ)

double Kp=1, Ki=10, Kd=0;

//Define Variables we'll be connecting to
double angle_x_set, angle_x, corr_x;
PID pid_roll(&angle_x, &corr_x, &angle_x_set, Kp, Ki, Kd, DIRECT);
double angle_y_set, angle_y, corr_y;
PID pid_pitch(&angle_y, &corr_y, &angle_y_set, Kp, Ki, Kd, DIRECT);
double angle_z_set, angle_z, corr_z;
PID pid_yaw(&angle_z, &corr_z, &angle_z_set, Kp, Ki, Kd, DIRECT);

IBusBM IBus;    // IBus object

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

float rnd(float min = -1, float max = 1) {
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

class IServo {
public:
    IServo() { }
    IServo(uint8_t pin) {
	    _servoIndex = ESP32_ISR_Servos.setupServo(pin, MIN_MICROS, MAX_MICROS);
    }
    void set_angle(int angle) {
        ESP32_ISR_Servos.setPosition(_servoIndex, angle + 90);
    }

private:
    int8_t _servoIndex;
};


class VectorOperations {
public:
    static void axes_to_vector(std::array<float, 3> &vec, float ax, float ay, float az) {
        vec[0] = ax;
        vec[1] = ay;
        vec[2] = az;
    }

    static float get_vector_length(std::array<float, 3> &vec) {
        return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    }
};


class BaseIMU { 
protected:
    Eigen::Matrix3f _base_rotation;
    std::array<float, 3> _gyro_null_val;
public:
    virtual void get_rotations(std::array<float, 3> &rotations) = 0;
    virtual void get_accelerations(std::array<float, 3> &accelerations) = 0;
    virtual void calibrate(uint n_samples) = 0;
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

    /**
     * @brief Read raw gyro values from device and convert to degrees per second
     * 
     * @param gRaw 
     * @return float 
     */
    float convert_raw_gyro(int gRaw) {
        float g = (gRaw * 1000.0) / 32768.0;
        return g;
    }

    /**
     * @brief Read raw accelerometer values from device and convert to g
     * 
     * @param aRaw 
     * @return float 
     */
    float convert_raw_accel(int aRaw) {
        float a = (aRaw * 2.0) / 32768.0;
        return a;
    }

    void get_rotations(std::array<float, 3> &rotations) {
        int gxRaw, gyRaw, gzRaw;         // raw gyro values
        this->readGyro(gxRaw, gyRaw, gzRaw);
        rotations[0] = convert_raw_gyro(gxRaw) - _gyro_null_val[0]; 
        rotations[1] = convert_raw_gyro(gyRaw) - _gyro_null_val[1]; 
        rotations[2] = convert_raw_gyro(gzRaw) - _gyro_null_val[2]; 
    }

    void get_accelerations(std::array<float, 3> &accelerations) {
        int axRaw, ayRaw, azRaw;         // raw gyro values
        this->readAccelerometer(axRaw, ayRaw, azRaw);
        accelerations[0] = convert_raw_accel(axRaw);
        accelerations[1] = convert_raw_accel(ayRaw);
        accelerations[2] = convert_raw_accel(azRaw);
    }

    void calibrate(uint n_samples) {
        // TODO
    }
};

class MockupIMU: public BaseIMU {
private:
    std::array<float, 3> _rotations = {0, 0, 0};
    std::array<float, 3> _accelerations = {0, 0, 0};
public:
    void get_rotations(std::array<float, 3> &rotations) {
        // get some random values in range (-10, 10) and add to the base values,
        rotations[0] = _rotations[0] + rnd();
        rotations[1] = _rotations[1] + rnd();
        rotations[2] = _rotations[2] + rnd();
        // then update the base values
        std::copy(rotations.begin(), rotations.end(), _rotations.begin());
        
    }

    void get_accelerations(std::array<float, 3> &accelerations) {
        // do the same for accelerations
        accelerations[0] = _accelerations[0] + rnd() / 10.0;
        accelerations[1] = _accelerations[1] + rnd() / 10.0;
        accelerations[2] = _accelerations[2] + rnd() / 10.0;
        std::copy(accelerations.begin(), accelerations.end(), _accelerations.begin());
    }

    void calibrate(uint n_samples) {
        // TODO
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
    void set_roll(float roll) {
        _orientation[0] = roll;
    }   
    void set_pitch(float pitch) {
        _orientation[1] = pitch;
    }
    void set_yaw(float yaw) {
        _orientation[2] = yaw;
    }
    float get_roll() {
        return _orientation[0];
    }
    float get_pitch() {
        return _orientation[1];
    }
    float get_yaw() {
        return _orientation[2];
    }
};


class Control {
protected:
    std::array<float, 3> _desired_orientation;
    float _desired_throttle;
public:
    void get_desired_orientation(
            std::array<float, 3> &desired_orientation, 
            float desired_throttle) {
        std::copy(
            _desired_orientation.begin(), 
            _desired_orientation.end(), 
            desired_orientation.begin()
        );
    }

    void get_desired_throttle(float &desired_throttle) {
        desired_throttle = _desired_throttle;
    }

    virtual void update() = 0;
};


class RemoteControl : public Control {
private:
    IBusBM _ibus;
public:
    RemoteControl() {
        _ibus.begin(Serial2,1);
    }

    void update() {
        _desired_orientation[0] = _ibus.readChannel(0);
        _desired_orientation[1] = _ibus.readChannel(1);
        _desired_orientation[2] = _ibus.readChannel(2);
        _desired_throttle = _ibus.readChannel(3);
    }
};


class Autopilot : public Control {
private:
    std::array<float, 3> _desired_orientation;
public:
    Autopilot() { }

    void update() {
        _desired_orientation[0] = 0;
        _desired_orientation[1] = 0;
        _desired_orientation[2] = 0;
        _desired_throttle = 0;
    }
};


class Aircraft {
private:
    std::array<float, 3> _gyro_vals;
    std::array<float, 3> _acc_vals;
    std::array<float, 3> _desired_orientation;
    float _desired_throttle;

    BaseIMU *_imu;
    IServo _servo_taileron_l;
    IServo _servo_taileron_r;
    IServo _servo_rudder;
    IServo _throttle;
    
    float _throttle_value;
    float _taileron_left_value;
    float _taileron_right_value;
    float _rudder_value;

    bool _invert_taileron_left;
    bool _invert_taileron_right;
    bool _control_rudder;
    bool _control_throttle;

    // gains to correct values from control
    float _throttle_gain = 1;
    float _roll_gain = 1;
    float _pitch_gain = 1;
    float _yaw_gain = 1;

    // calculated values for steering:
    float _roll_corr;
    float _pitch_corr;
    float _yaw_corr;
    float _throttle_corr;

    // properties
    float _mass;
    float _wing_area;
    float _max_thrust;
    float _lift_coefficient;

    void compensate_thrust() {
        // assume linear thrust curve for now
        float current_thrust = _throttle_value * _max_thrust;
        // compensate for thrust:
        _acc_vals[0] -= current_thrust;
    }
    
public:
    Aircraft(
            BaseIMU *imu, 
            bool control_rudder = true, 
            bool control_throttle = true, 
            bool invert_taileron_left = false, 
            bool invert_taileron_right = false) : 
        _control_rudder(control_rudder), 
        _control_throttle(control_throttle),
        _invert_taileron_left(invert_taileron_left),
        _invert_taileron_right(invert_taileron_right) { 
        _imu = imu;
        _servo_taileron_l = IServo(PIN_SERVO_TAILERON_L);
        _servo_taileron_r = IServo(PIN_SERVO_TAILERON_R);
        _servo_rudder = IServo(PIN_SERVO_RUDDER);
        _throttle = IServo(PIN_THROTTLE);
    }

    void setup(
            float throttle_gain, 
            float roll_gain, 
            float pitch_gain, 
            float yaw_gain) {
        _throttle_gain = throttle_gain;
        _roll_gain = roll_gain;
        _pitch_gain = pitch_gain;
        _yaw_gain = yaw_gain;
    }

    void properties(
            float mass,
            float wing_area,
            float thrust,
            float lift_coefficient) {
        _mass = mass;
        _wing_area = wing_area;
        _max_thrust = thrust;
        _lift_coefficient = lift_coefficient;
    }

    void calculate_corrections(Control *controller) {
        // calculate corrections read data from IMU first
        read_imu();
        compensate_thrust();

        // calculate corrections
        controller->get_desired_orientation(_desired_orientation, _desired_throttle);
        calculate_roll_correction(_desired_orientation[0]);
        calculate_pitch_correction(_desired_orientation[1]);
        calculate_yaw_correction(_desired_orientation[2]);
    }

    void calculate_roll_correction(float desired_roll) {
        _roll_corr = 0;  // TODO Sem treba aj PID
    }

    void calculate_pitch_correction(float desired_pitch) {
        _pitch_corr = 0;  // TODO Sem treba aj PID
    }

    void calculate_yaw_correction(float desired_yaw) {
        _yaw_corr = (desired_yaw * _yaw_gain) - _gyro_vals[2];
    }

    void steer() {
        // steer the aircraft
        set_tailerons();
        set_rudder();
        set_throttle();
    }

    void read_imu() {
        // read raw gyro measurements from device
        _imu->get_rotations(_gyro_vals);
        // read raw accelerometer measurements from device
        _imu->get_accelerations(_acc_vals);
        ESP_LOGD("Aircraft", "Gyro: %f, %f, %f", _gyro_vals[0], _gyro_vals[1], _gyro_vals[2]);
        ESP_LOGD("Aircraft", "Acc: %f, %f, %f", _acc_vals[0], _acc_vals[1], _acc_vals[2]);
    }

    /**
     * @brief Set the rudder angle
     * 
     * @param angle in degrees in range -90, 90
     */
    void set_taileron_left(float angle) {
        // set angle of left taileron
        // left is probably inverted, so invert angle
        if (_invert_taileron_left) {
            angle = -angle;
        }
        // TODO covert angle from range -90, 90 if necessary
        _servo_taileron_l.set_angle(angle);
        ESP_LOGD("Aircraft", "Set taileron left to %f", angle);
    }

    /**
     * @brief Set the rudder angle
     * 
     * @param angle in degrees in range -90, 90
     */
    void set_taileron_right(float angle) {
        // set angle of right taileron
        // TODO covert angle from range -90, 90 if necessary
        _servo_taileron_l.set_angle(angle);
        ESP_LOGD("Aircraft", "Set taileron right to %f", angle);
    }

    /**
     * @brief Set the rudder angle
     * 
     * @param throttle in range 0, 100
     */
    void set_throttle() {
        // set throttle
        // TODO covert throttle to range 0, 100 if necessary
        _throttle.set_angle(_throttle_corr);
        ESP_LOGD("Aircraft", "Set throttle to %f", _throttle_corr);
    }

    void set_tailerons() {
        _taileron_left_value = _roll_corr;
        _taileron_right_value = -_roll_corr;
        _taileron_left_value += _pitch_corr;
        _taileron_right_value += _pitch_corr;

        set_taileron_left(_taileron_left_value);
        set_taileron_right(_taileron_right_value);
    }

    void set_rudder() {
        // set rudder angle
        // TODO convert angle from range -90, 90 if necessary
        _rudder_value = _yaw_corr;
        _servo_rudder.set_angle(_rudder_value);
        ESP_LOGD("Aircraft", "Set rudder to %f", _rudder_value);
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

    ESP_LOGI("main", "Starting BMI160");
}

void loop() {
    #ifdef MOCKUP
    MockupIMU *Imu = new MockupIMU();
    #else
    IMU *Imu = new IMU();    
    #endif // MOCKUP

    Aircraft aircraft(Imu);
    aircraft.setup(1, 1, 1, 1);
    aircraft.properties(1.3, 0.3, 15, 0.87);

    #ifdef AUTOPILOT
    Control *controller = new Autopilot();
    #else
    Control *controller = new RemoteControl();
    #endif // AUTOPILOT

    uint64_t iter = 0;

    while(1) {
        auto start = std::chrono::high_resolution_clock::now();

        aircraft.calculate_corrections(controller);
        aircraft.steer();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        int dt = duration.count();
        if (dt < LOOP_PERIOD_US) {
            delayMicroseconds(LOOP_PERIOD_US - dt);
        }
        if (iter++ % 100 == 0) {
            // print here some statistics
        }
    }
}
