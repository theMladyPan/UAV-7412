#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#include <ArduinoEigenDense.h>
#include "IServo.h"
#include "IMU/BaseIMU.h"
#include "Control/Control.h"
#include "Regulator/RegulatorBase.h"

typedef struct
{
    // control flags
    bool invert_taileron_left = false;
    bool invert_taileron_right = false;
    bool invert_rudder = false;
    
    bool control_rudder = false;
    bool control_throttle = false;
    uint32_t loop_period_us;
    float angle_min;
    float angle_max;

    float max_thrust;  // Newtons
    float mass;  // kg
} aircraft_param_t;


template <typename T>
class Aircraft {
private:
    Eigen::Vector3d _gyro_vals;
    Eigen::Vector3d _acc_vals;
    Eigen::Vector3d _desired_orientation;  // roll, pitch, yaw
    Eigen::Vector3d _current_orientation;  // roll, pitch, yaw
    float _desired_throttle;
    aircraft_param_t _params;

    BaseIMU *_imu;
    T* _regulator;
    IServo* _servo_taileron_l;
    IServo* _servo_taileron_r;
    IServo* _servo_rudder;
    IServo* _throttle;
    
    // curent values for steering:
    float _throttle_value;  // 0-100%
    float _taileron_left_value;  // -90, 90 deg
    float _taileron_right_value;  // -90, 90 deg
    float _rudder_value;   // -90, 90 deg


    // calculated values for steering:
    Eigen::Vector3d _corrections;  // roll, pitch, yaw
    float _throttle_corr;

    void convert_acc_to_orientation();
    
public:
    Aircraft(
        BaseIMU *imu, 
        aircraft_param_t &params) ;

    void setup(
        int pin_servo_taileron_l,
        int pin_servo_taileron_r,
        int pin_servo_rudder,
        int pin_throttle);


    void calculate_corrections(Control *controller);

    void calculate_roll_correction(float desired_roll);

    void calculate_pitch_correction(float desired_pitch);

    void calculate_yaw_correction(float desired_yaw);

    void calculate_throttle_correction(float desired_throttle);

    void steer();

    void update();

    /**
     * @brief Set the rudder angle
     * 
     * @param angle in degrees in range -90, 90
     */
    void set_taileron_left(float angle);

    /**
     * @brief Set the rudder angle
     * 
     * @param angle in degrees in range -90, 90
     */
    void set_taileron_right(float angle);

    /**
     * @brief Set the throttle value
     */
    void set_throttle();

    void set_tailerons();

    void set_rudder();

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
    void pre_flight_check();

    void setup_regulator(void* params);

    void print_status();
}; 


#include "Aircraft.tpp"

#endif // AIRCRAFT_H