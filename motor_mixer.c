// motor_mixer.c
// turn the three PID outputs (roll, pitch, yaw) into four motor commands

// motor layout (X config):
// M1 rear-left  (CCW)   M2 front-right (CCW)  - same diagonal
// M3 rear-right (CW)    M4 front-left  (CW)   - other diagonal
//
// roll+ (right side up):    M2, M3 up; M1, M4 down
// pitch+ (nose up):         M1, M3 up; M2, M4 down
// yaw+ (CCW from above):    M1, M2 up; M3, M4 down
//
// the motor command represents omega^2 (speed squared), not RPM directly
// so thrust = k * command is linear under this convention

#include "pid_motor_mixer.h"
#include <math.h>

void motor_mixer(output* spinMotors, estimator* estimator_angle, pid* pid_outputs) {

    // physical constants
    const float drone_mass         = 4.0f;
    const float gravity            = 9.81f;
    const float air_density        = 1.211f;
    const float thrust_coefficient = 3e-3f;
    const float motor_radius       = 3.3f;
    const float pi                 = 3.14159265358979f;
    const float area               = pi * motor_radius * motor_radius;
    const float k                  = thrust_coefficient * air_density * area; // thrust per unit command

    // tilt compensation - need more total thrust when tilted to stay airborne
    float ctilt = cosf(estimator_angle->cur_roll) * cosf(estimator_angle->cur_pitch);
    if (ctilt < 0.5f) ctilt = 0.5f; // clamp - don't ask for infinite thrust at extreme tilts

    float totalThrust = (drone_mass * gravity) / ctilt; // total needed in newtons
    float hoverThrust = totalThrust / 4.0f; // per motor

    // mix the PID commands - signs follow the layout above
    float motorOne_thrust   = hoverThrust - pid_outputs->roll_pid + pid_outputs->pitch_pid + pid_outputs->yaw_pid;
    float motorTwo_thrust   = hoverThrust + pid_outputs->roll_pid - pid_outputs->pitch_pid + pid_outputs->yaw_pid;
    float motorThree_thrust = hoverThrust + pid_outputs->roll_pid + pid_outputs->pitch_pid - pid_outputs->yaw_pid;
    float motorFour_thrust  = hoverThrust - pid_outputs->roll_pid - pid_outputs->pitch_pid - pid_outputs->yaw_pid;

    // motors can't push down - clamp at zero
    if (motorOne_thrust   < 0.0f) motorOne_thrust   = 0.0f;
    if (motorTwo_thrust   < 0.0f) motorTwo_thrust   = 0.0f;
    if (motorThree_thrust < 0.0f) motorThree_thrust = 0.0f;
    if (motorFour_thrust  < 0.0f) motorFour_thrust  = 0.0f;

    // convert thrust to motor command (= thrust / k)
    spinMotors->spinMotorOne   = motorOne_thrust   / k;
    spinMotors->spinMotorTwo   = motorTwo_thrust   / k;
    spinMotors->spinMotorThree = motorThree_thrust / k;
    spinMotors->spinMotorFour  = motorFour_thrust  / k;
}