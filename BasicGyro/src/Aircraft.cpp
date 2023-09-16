#include "Aircraft.h"

#include "VectorOperations.h"

void Aircraft::convert_acc_to_orientation() {
    // convert _acc_vals to roll, pitch
    _current_orientation[0] = asin(_acc_vals[1]) * 180 / M_PI;
    _current_orientation[1] = asin(_acc_vals[0]) * 180 / M_PI;
    // yaw can be only obtained from GPS, aproximate from gyro:
    _current_orientation[2] += _gyro_vals[2] * _params.loop_period_us / 1000000;
}

Aircraft::Aircraft(
        BaseIMU *imu, 
        Regulator *regulator,
        aircraft_param_t &params) { 
    _imu = imu;
    _params = params;
    _regulator = regulator;
    _regulator->set_setpoint(&_desired_orientation);
    _regulator->set_correction(&_corrections);
    _regulator->set_feedback(&_gyro_vals);
}

void Aircraft::setup(
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

void Aircraft::calculate_corrections(Control *controller) {
    // calculate corrections
    controller->get_desired_orientation(_desired_orientation);
    controller->get_desired_throttle(_desired_throttle);

    // update regulator with new measurements
    _regulator->update();

    calculate_roll_correction(_desired_orientation[0]);
    calculate_pitch_correction(_desired_orientation[1]);
    calculate_yaw_correction(_desired_orientation[2]);
}

void Aircraft::calculate_roll_correction(float desired_roll) {
    // Handled by regulator
}

void Aircraft::calculate_pitch_correction(float desired_pitch) {
    // Handled by regulator
}

void Aircraft::calculate_yaw_correction(float desired_yaw) {
    _corrections[2] = desired_yaw - _gyro_vals[2];
}

void Aircraft::steer() {
    // steer the aircraft
    set_tailerons();
    set_rudder();
    set_throttle();
}

void Aircraft::update() {
    // read raw gyro measurements from device
    _imu->get_rotations_dps(_gyro_vals);
    // read raw accelerometer measurements from device
    _imu->get_accelerations_g(_acc_vals);
    ESP_LOGD("Aircraft", "Gyro: %f, %f, %f", _gyro_vals[0], _gyro_vals[1], _gyro_vals[2]);
    ESP_LOGD("Aircraft", "Acc: %f, %f, %f", _acc_vals[0], _acc_vals[1], _acc_vals[2]);

    // convert _acc_vals to roll, pitch, yaw
    convert_acc_to_orientation();
}

void Aircraft::set_taileron_left(float angle) {
    // set angle of left taileron
    // left is probably inverted, so invert angle
    if (_params.invert_taileron_left) {
        angle = -angle;
    }
    // TODO covert angle from range -90, 90 if necessary
    _servo_taileron_l.set_angle(angle);
    ESP_LOGD("Aircraft", "Set taileron left to %f", angle);
}

void Aircraft::set_taileron_right(float angle) {
    // set angle of right taileron
    // TODO covert angle from range -90, 90 if necessary
    if (_params.invert_taileron_right) {
        angle = -angle;
    }
    _servo_taileron_l.set_angle(angle);
    ESP_LOGD("Aircraft", "Set taileron right to %f", angle);
}

void Aircraft::set_throttle() {
    // set throttle
    // TODO covert throttle to range 0, 100 if necessary
    _throttle.set_angle(_throttle_corr);
    ESP_LOGD("Aircraft", "Set throttle to %f", _throttle_corr);
}

void Aircraft::set_tailerons() {
    // we are using V-tail configuration, so we need to set both tailerons
    _taileron_left_value = _corrections[0];
    _taileron_right_value = -_corrections[0];
    _taileron_left_value += _corrections[1];
    _taileron_right_value += _corrections[1];

    set_taileron_left(_taileron_left_value);
    set_taileron_right(_taileron_right_value);
}

void Aircraft::set_rudder() {
    // set rudder angle
    // TODO convert angle from range -90, 90 if necessary
    _rudder_value = _corrections[2];
    _servo_rudder.set_angle(_rudder_value);
    ESP_LOGD("Aircraft", "Set rudder to %f", _rudder_value);
}

void Aircraft::pre_flight_check() {
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
    _corrections[2] = -45;
    set_rudder();
    delay(1000);
    // rudder right
    _corrections[2] = 45;
    set_rudder();
    delay(1000);
    // zero rudder
    _corrections[2] = 0;
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
