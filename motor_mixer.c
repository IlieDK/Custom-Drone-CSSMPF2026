//
// Created by Gabriel Kihara on 01/05/2026.
//
// Frame convention (X-config) — must match simulation.py:
//   M1: rear-left   (CCW)   ┐
//   M2: front-right (CCW)   ┘ same diagonal — both CCW
//   M3: rear-right  (CW)    ┐
//   M4: front-left  (CW)    ┘ other diagonal — both CW
//
// Roll+  (right side up):  M2, M3 up;  M1, M4 down
// Pitch+ (nose up):        M1, M3 up;  M2, M4 down
// Yaw+   (CCW from above): M1, M2 up;  M3, M4 down  ← CCW motors increase
//

#include "pid_motor_mixer.h"
#include <math.h>

void motor_mixer(output* spinMotors, estimator* estimator_angle, pid* pid_outputs) {

    const float drone_mass         = 4.0f;
    const float gravity            = 9.81f;
    const float air_density        = 1.211f;
    const float thrust_coefficient = 3e-3f;
    const float motor_radius       = 3.3f;
    const float pi                 = 3.14159265358979f;
    const float area               = pi * motor_radius * motor_radius;
    const float k                  = thrust_coefficient * air_density * area;

    /* Tilt-compensated hover thrust:
     *   when the drone is tilted, the body z-thrust must be larger than
     *   m*g by a factor of 1 / (cos(roll)*cos(pitch)) to keep the world-
     *   frame vertical component equal to weight.
     *
     *   Clamp the divisor so an extreme tilt doesn't ask for infinite
     *   thrust (and so cos < 0, i.e. drone upside down, doesn't flip the
     *   sign).
     */
    float ctilt = cosf(estimator_angle->cur_roll) * cosf(estimator_angle->cur_pitch);
    if (ctilt < 0.5f) ctilt = 0.5f;   /* clamp to ~60° combined tilt */

    float totalThrust = (drone_mass * gravity) / ctilt;
    float hoverThrust = totalThrust / 4.0f;

    /* Apply the differential PID commands. Each motor receives the same
       hover baseline plus signed contributions from the three attitude
       PIDs, with signs determined by the X-config geometry above. */
    float motorOne_thrust   = hoverThrust - pid_outputs->roll_pid + pid_outputs->pitch_pid + pid_outputs->yaw_pid;
    float motorTwo_thrust   = hoverThrust + pid_outputs->roll_pid - pid_outputs->pitch_pid + pid_outputs->yaw_pid;
    float motorThree_thrust = hoverThrust + pid_outputs->roll_pid + pid_outputs->pitch_pid - pid_outputs->yaw_pid;
    float motorFour_thrust  = hoverThrust - pid_outputs->roll_pid - pid_outputs->pitch_pid - pid_outputs->yaw_pid;

    /* Per-motor thrust must be non-negative (motors can't push the drone
       down). Saturate at zero rather than letting negative values wrap
       through the divisor. */
    if (motorOne_thrust   < 0.0f) motorOne_thrust   = 0.0f;
    if (motorTwo_thrust   < 0.0f) motorTwo_thrust   = 0.0f;
    if (motorThree_thrust < 0.0f) motorThree_thrust = 0.0f;
    if (motorFour_thrust  < 0.0f) motorFour_thrust  = 0.0f;

    spinMotors->spinMotorOne   = motorOne_thrust   / k;
    spinMotors->spinMotorTwo   = motorTwo_thrust   / k;
    spinMotors->spinMotorThree = motorThree_thrust / k;
    spinMotors->spinMotorFour  = motorFour_thrust  / k;
}