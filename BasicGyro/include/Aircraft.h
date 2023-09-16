#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#include <ArduinoEigenDense.h>
#include "IServo.h"
#include "IMU/BaseIMU.h"
#include "Control/Control.h"
#include "VectorOperations.h"

typedef struct
{
    // control flags
    bool invert_taileron_left = false;
    bool invert_taileron_right = false;
    bool invert_rudder = false;
    
    bool control_rudder = false;
    bool control_throttle = false;
} aircraft_param_t;


class Aircraft {
private:
    Eigen::Vector3f _gyro_vals;
    Eigen::Vector3f _acc_vals;
    Eigen::Vector3f _desired_orientation;
    float _desired_throttle;
    aircraft_param_t _params;

    BaseIMU *_imu;
    IServo _servo_taileron_l;
    IServo _servo_taileron_r;
    IServo _servo_rudder;
    IServo _throttle;
    
    // curent values for steering:
    float _throttle_value;  // 0-100%
    float _taileron_left_value;  // -90, 90 deg
    float _taileron_right_value;  // -90, 90 deg
    float _rudder_value;   // -90, 90 deg


    // calculated values for steering:
    float _roll_corr;
    float _pitch_corr;
    float _yaw_corr;
    float _throttle_corr;

    void compensate_thrust() {
        // calculate length of acceleration vector
        float vgt = VectorOperations::get_vector_length(_acc_vals);
        // thrust is always in X direction
        float xt = sqrt(vgt * vgt - 1);
    }
    
public:
    Aircraft(
            BaseIMU *imu, 
            aircraft_param_t &params) { 
        _imu = imu;
        _params = params;
    }

    void setup(
            int pin_servo_taileron_l,
            int pin_servo_taileron_r,
            int pin_servo_rudder,
            int pin_throttle) {
	    ServoImpl.useTimer(0);
        _servo_taileron_l = IServo(pin_servo_taileron_l);
        _servo_taileron_r = IServo(pin_servo_taileron_r);
        _servo_rudder = IServo(pin_servo_rudder);
        _throttle = IServo(pin_throttle);
    }


    void calculate_corrections(Control *controller) {

        // calculate corrections
        controller->get_desired_orientation(_desired_orientation);
        controller->get_desired_throttle(_desired_throttle);

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
        _yaw_corr = desired_yaw - _gyro_vals[2];
    }

    void steer() {
        // steer the aircraft
        set_tailerons();
        set_rudder();
        set_throttle();
    }

    void update() {
        // read raw gyro measurements from device
        _imu->get_rotations_dps(_gyro_vals);
        // read raw accelerometer measurements from device
        _imu->get_accelerations_g(_acc_vals);
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
        if (_params.invert_taileron_left) {
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
        if (_params.invert_taileron_right) {
            angle = -angle;
        }
        _servo_taileron_l.set_angle(angle);
        ESP_LOGD("Aircraft", "Set taileron right to %f", angle);
    }

    /**
     * @brief Set the throttle value
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

    /**
     * @brief Performs pre-flight checks for the aircraft's control surfaces and throttle.
     *
     * The pre_flight_check function conducts a series of control surface and throttle
     * tests to ensure the aircraft is ready for operation. These checks are sequenced
     * with delays to allow for visual and instrumental verification.
     *
     * - Tailerons are set to simulate roll maneuvers (both CW and CCW).
     * - Tailerons are also adjusted for pitch up and pitch down.
     * - Rudder is set for a left and right turn.
     * - Throttle is momentarily set to 20% before returning to 0%.
     *
     * A log entry is generated to indicate the completion of the pre-flight checks.
     *
     * @note Make sure that all sub-systems are initialized before calling this function.
     * @note The control settings and gains should be properly configured prior to running this check.
     *
     * Example usage:
     * @code
     * pre_flight_check();
     * @endcode
     */
    void pre_flight_check() {
        ESP_LOGI("Aircraft", "Starting pre-flight check sequence");
        // roll ccw
        set_taileron_left(45);
        set_taileron_right(-45);
        delay(1000);
        // roll cw
        set_taileron_left(-45);
        set_taileron_right(45);
        delay(1000);
        // zero roll
        set_taileron_left(90);
        set_taileron_right(90);
        
        // pitch up
        set_taileron_left(45);
        set_taileron_right(45);
        delay(1000);

        // pitch down
        set_taileron_left(-45);
        set_taileron_right(-45);
        delay(1000);

        // zero pitch
        set_taileron_left(0);
        set_taileron_right(0);
        
        // rudder left
        _yaw_corr = -45;
        set_rudder();
        delay(1000);
        // rudder right
        _yaw_corr = 45;
        set_rudder();
        delay(1000);
        // zero rudder
        _yaw_corr = 0;
        set_rudder();
        // set throttle to 20%
        _throttle_corr = 20;
        set_throttle();
        delay(200);
        // set throttle to 0%
        _throttle_corr = 0;
        set_throttle();
        ESP_LOGI("Aircraft", "Pre-flight check done");
    }

}; 

#endif // AIRCRAFT_H