#include "Aircraft.h"
#include "VectorOperations.h"
#include <iostream>
#include <sstream>


template <class T>
void Aircraft<T>::convert_acc_to_orientation() {
    // convert _acc_vals to roll, pitch
    _current_orientation[0] = asin(_acc_vals[1]) * 180 / M_PI;
    _current_orientation[1] = -asin(_acc_vals[0]) * 180 / M_PI;
    // yaw can be only obtained from GPS, aproximate from gyro:
    _current_orientation[2] += _gyro_vals[2] * _params.loop_period_us / 1e6;
}


template <class T>
Aircraft<T>::Aircraft(
        BaseIMU *imu, 
        aircraft_param_t &params) { 
    _imu = imu;
    _params = params;
    ESP_LOGD("Aircraft", "Aircraft created, initializing regulator");
    _regulator->set_setpoint(&_desired_orientation);
    _regulator->set_correction(&_corrections);
    _regulator->set_feedback(&_current_orientation);
}


template <class T>
void Aircraft<T>::setup(
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


template <class T>
void Aircraft<T>::calculate_corrections(Control *controller) {
    // calculate corrections
    controller->update();
    controller->get_desired_orientation(_desired_orientation);
    controller->get_desired_throttle(_desired_throttle);

    calculate_roll_correction(_desired_orientation[0]);
    calculate_pitch_correction(_desired_orientation[1]);
    if (_params.control_rudder) {
        calculate_yaw_correction(_desired_orientation[2]);
    }
    else {
        _corrections[2] = _desired_orientation[2];
    }
    if(_params.control_throttle) {
        calculate_throttle_correction(_desired_throttle);
    }
    else {
        _throttle_corr = _desired_throttle;
    }

    _regulator->update();
}


template <class T>
void Aircraft<T>::calculate_roll_correction(float desired_roll) {
    // Handled by regulator
    // _corrections[0] = desired_roll - _current_orientation[0];
}


template <class T>
void Aircraft<T>::calculate_pitch_correction(float desired_pitch) {
    // Handled by regulator
    // _corrections[1] = desired_pitch - _current_orientation[1];
}


template <class T>
void Aircraft<T>::calculate_yaw_correction(float desired_yaw) {
    _corrections[2] = desired_yaw - _gyro_vals[2];
}


template <class T>
void Aircraft<T>::calculate_throttle_correction(float desired_throttle) {
    _throttle_corr = desired_throttle;
    // TODO: maybe add some correction here
}


template <class T>
void Aircraft<T>::steer() {
    // steer the aircraft
    set_tailerons();
    set_rudder();
    set_throttle();
}


template <class T>
void Aircraft<T>::update() {
    // read raw gyro measurements from device
    _imu->get_rotations_dps(_gyro_vals);
    // read raw accelerometer measurements from device
    _imu->get_accelerations_g(_acc_vals);
    ESP_LOGD("Aircraft", "Gyro: %f, %f, %f", _gyro_vals[0], _gyro_vals[1], _gyro_vals[2]);
    ESP_LOGD("Aircraft", "Acc: %f, %f, %f", _acc_vals[0], _acc_vals[1], _acc_vals[2]);

    // convert _acc_vals to roll, pitch, yaw
    convert_acc_to_orientation();
}


template <class T>
void Aircraft<T>::set_taileron_left(float angle) {
    // set angle of left taileron
    // left is probably inverted, so invert angle
    if (_params.invert_taileron_left) {
        angle = -angle;
    }
    //clamp angle to -45, 45
    if (angle > 45) {
        angle = 45;
    } else if (angle < -45) {
        angle = -45;
    }
    ESP_LOGD("Aircraft", "Setting taileron left to %f", angle);
    _servo_taileron_l.set_angle(angle);
}


template <class T>
void Aircraft<T>::set_taileron_right(float angle) {
    // set angle of right taileron
    if (_params.invert_taileron_right) {
        angle = -angle;
    }
    //clamp angle to -45, 45
    if (angle > 45) {
        angle = 45;
    } else if (angle < -45) {
        angle = -45;
    }
    ESP_LOGD("Aircraft", "Setting taileron right to %f", angle);
    _servo_taileron_r.set_angle(angle);
}


template <class T>
void Aircraft<T>::set_throttle() {
    // set throttle
    ESP_LOGD("Aircraft", "Setting throttle to %f", _throttle_corr);
    _throttle.set_percent(_throttle_corr);
}


template <class T>
void Aircraft<T>::set_tailerons() {
    // we are using V-tail configuration, so we need to set both tailerons
    _taileron_left_value = _corrections[0];
    _taileron_right_value = -_corrections[0];
    _taileron_left_value += _corrections[1];
    _taileron_right_value += _corrections[1];

    set_taileron_left(_taileron_left_value);
    set_taileron_right(_taileron_right_value);
}


template <class T>
void Aircraft<T>::set_rudder() {
    // set rudder angle
    // TODO convert angle from range -90, 90 if necessary
    _rudder_value = _corrections[2];
    ESP_LOGD("Aircraft", "Setting rudder to %f", _rudder_value);
    _servo_rudder.set_angle(_rudder_value);
}


template <class T>
void Aircraft<T>::pre_flight_check() {
    ESP_LOGI("Aircraft", "Starting pre-flight check sequence");
    
    // roll ccw
    ESP_LOGI("Aircraft", "Rolling CCW");
    set_taileron_left(45);
    set_taileron_right(-45);
    delay(1000);

    // roll cw
    ESP_LOGI("Aircraft", "Rolling CW");
    set_taileron_left(-45);
    set_taileron_right(45);
    delay(1000);

    // zero roll
    ESP_LOGI("Aircraft", "Zeroing roll");
    set_taileron_left(0);
    set_taileron_right(0);
    delay(1000);

    // pitch up
    ESP_LOGI("Aircraft", "Pitching up");
    set_taileron_left(45);
    set_taileron_right(45);
    delay(1000);

    // pitch down
    ESP_LOGI("Aircraft", "Pitching down");
    set_taileron_left(-45);
    set_taileron_right(-45);
    delay(1000);

    // zero pitch
    ESP_LOGI("Aircraft", "Zeroing pitch");
    set_taileron_left(0);
    set_taileron_right(0);
    delay(1000);
    
    // rudder left
    ESP_LOGI("Aircraft", "Ruddering left");
    _corrections[2] = -45;
    set_rudder();
    delay(1000);

    // rudder right
    ESP_LOGI("Aircraft", "Ruddering right");
    _corrections[2] = 45;
    set_rudder();
    delay(1000);

    // zero rudder
    ESP_LOGI("Aircraft", "Zeroing rudder");
    _corrections[2] = 0;
    set_rudder();

    // set throttle to 20%
    ESP_LOGI("Aircraft", "Setting throttle to 20%");
    _throttle_corr = 20;
    set_throttle();
    delay(500);

    // set throttle to 0%
    ESP_LOGI("Aircraft", "Setting throttle to 0%");
    _throttle_corr = 0;
    set_throttle();

    ESP_LOGI("Aircraft", "Pre-flight check done");
}


template <class T>
void Aircraft<T>::setup_regulator(void *params) {
    _regulator = new T(&_current_orientation, &_desired_orientation, &_corrections);
    _regulator->setup(params, _params.angle_min, _params.angle_max);
}


template <class T>
void Aircraft<T>::print_status() {

    std::cout << "Current orientation: " << _current_orientation.transpose() << std::endl;
    std::cout << "Desired orientation: " << _desired_orientation.transpose() << std::endl;
    std::cout << "Desired throttle: " << _desired_throttle << std::endl;
    std::cout << "Corrections: " << _corrections.transpose() << std::endl;
    std::cout << "Throttle correction: " << _throttle_corr << std::endl;

}