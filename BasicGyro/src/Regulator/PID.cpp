#include "Regulator/PID.h"
#include "esp_log.h"


PID::PID() {
    _params.kp = 1;
    _params.ki = 0;
    _params.kd = 0;
    _params.sampling_period = 1;
    _last_error = 0;
    _integral = 0;
}

PID::PID(pid_params_t params) {
    _params = params;
    _last_error = 0;
}

double PID::correction(double feedback, double setpoint) {
    double error = setpoint - feedback;
    _integral += error * _params.sampling_period;
    double p = _params.kp * error;
    double i = _params.ki * _integral;
    double d = _params.kd * (error - _last_error) / _params.sampling_period;
    _last_error = error;
    double correction = p + i + d;
    ESP_LOGD("PID", "P: %f, I: %f, D: %f", p, i, d);

    double overshoot = correction - _max;
    double undershoot = correction - _min;
    if (overshoot > 0) {
        _integral -= overshoot;
        correction = _max;
    } else if (undershoot < 0) {
        _integral -= undershoot;
        correction = _min;
    }

    ESP_LOGD("PID", "Correction: %f", correction);
    return correction;
}

void PID::set_limits(double min, double max) {
    _min = min;
    _max = max;
}

