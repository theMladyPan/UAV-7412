#include "IMU/IMU.h"
#include "utils.h"

double IMU::convert_raw_gyro(int gRaw) {
    double g = (gRaw * 1000.0) / 32768.0;
    return g;
}

double IMU::convert_raw_accel(int aRaw) {
    double a = (aRaw * 2.0) / 32768.0;
    return a;
}

IMU::IMU() { 
    ESP_LOGD("IMU", "Initializing IMU");
    this->begin(BMI160GenClass::I2C_MODE, Wire, 0x68, 0);
    ESP_LOGD("IMU", "Getting device ID");
    uint8_t dev_id = this->getDeviceID();

    ESP_LOGI("IMU", "Device ID: %d", dev_id);

    // Set the accelerometer range to 250 degrees/second
    ESP_LOGD("IMU", "Setting gyro range ...");
    this->setFullScaleGyroRange(BMI160_GYRO_RANGE_1000);
    ESP_LOGD("IMU", "Setting gyro range done.");
    this->setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
    ESP_LOGD("IMU", "Setting accelerometer range done.");
    this->setGyroDLPFMode(BMI160_DLPF_MODE_OSR4);
    ESP_LOGD("IMU", "Setting gyro DLPF mode done.");
    this->setAccelDLPFMode(BMI160_DLPF_MODE_OSR4);
    ESP_LOGD("IMU", "Setting accelerometer DLPF mode done.");
}

void IMU::get_rotations_dps(Eigen::Vector3d &rotations) {
    int gxRaw, gyRaw, gzRaw;         // raw gyro values
    this->readGyro(gxRaw, gyRaw, gzRaw);
    rotations[0] = convert_raw_gyro(gxRaw) - _gyro_null_val[0]; 
    rotations[1] = convert_raw_gyro(gyRaw) - _gyro_null_val[1]; 
    rotations[2] = convert_raw_gyro(gzRaw) - _gyro_null_val[2]; 
}

void IMU::get_accelerations_g(Eigen::Vector3d &accelerations) {
    int axRaw, ayRaw, azRaw;         // raw gyro values
    this->readAccelerometer(axRaw, ayRaw, azRaw);
    accelerations[0] = convert_raw_accel(axRaw) * _acc_gain;
    accelerations[1] = convert_raw_accel(ayRaw) * _acc_gain;
    accelerations[2] = convert_raw_accel(azRaw) * _acc_gain;
    accelerations = _base_rotation * accelerations;
}

void IMU::calibrate(uint n_samples) {
    ESP_LOGI("IMU", "Calibrating Gyro ...");
    std::array<int, 3> gyro_vals;
    std::array<int64_t, 3> gyro_sums = {0, 0, 0};
    for(uint i = 0; i < n_samples; i++) {
        readGyro(gyro_vals[0], gyro_vals[1], gyro_vals[2]);
        gyro_sums[0] += gyro_vals[0];
        gyro_sums[1] += gyro_vals[1];
        gyro_sums[2] += gyro_vals[2];
        delay(1);
    }
    _gyro_null_val[0] = convert_raw_gyro(gyro_sums[0] / n_samples);
    _gyro_null_val[1] = convert_raw_gyro(gyro_sums[1] / n_samples);
    _gyro_null_val[2] = convert_raw_gyro(gyro_sums[2] / n_samples);
    
    ESP_LOGI("IMU", "Calibrating Gyro done.");
    ESP_LOGD("IMU", "Gyro null values: %f, %f, %f", _gyro_null_val[0], _gyro_null_val[1], _gyro_null_val[2]);

    ESP_LOGI("IMU", "Calibrating Accelerometer ...");
    std::array<int, 3> acc_vals;
    std::array<int64_t, 3> acc_sums = {0, 0, 0};
    for(uint i = 0; i < n_samples; i++) {
        readAccelerometer(acc_vals[0], acc_vals[1], acc_vals[2]);
        acc_sums[0] += acc_vals[0];
        acc_sums[1] += acc_vals[1];
        acc_sums[2] += acc_vals[2];
        delay(1);
    }
    for(uint i = 0; i < 3; i++) {
        acc_sums[i] /= n_samples;
    }
    Eigen::Vector3d acc_vec (
        convert_raw_accel(acc_sums[0]), 
        convert_raw_accel(acc_sums[1]), 
        convert_raw_accel(acc_sums[2])
    );
    ESP_LOGI("IMU", "Calibrating Accelerometer done.");
    ESP_LOGD("IMU", "Accelerometer null values: %f, %f, %f", acc_vec[0], acc_vec[1], acc_vec[2]);
    /*
    // maybe we dont need the gain
    VectorOperations::get_vector_length(acc_vec);
    _acc_gain = 1 / VectorOperations::get_vector_length(acc_vec);
    ESP_LOGD("IMU", "Accelerometer gain: %f", _acc_gain);
    */

    // calculate rotation matrix to shift accelerometer values to base frame
    Eigen::Vector3d base_vec(0, 0, 1);  // base vector is z-axis
    Eigen::Matrix3d R;
    calculateRotationMatrix(acc_vec, base_vec, R);
    ESP_LOGD("IMU", "Rotation matrix:\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f", 
        R(0, 0), R(0, 1), R(0, 2),
        R(1, 0), R(1, 1), R(1, 2),
        R(2, 0), R(2, 1), R(2, 2));

    _base_rotation = R;
    get_accelerations_g(acc_vec);
    ESP_LOGD("IMU", "Accelerometer after calibration: %f, %f, %f", acc_vec[0], acc_vec[1], acc_vec[2]);
}