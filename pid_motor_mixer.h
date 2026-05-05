//
// Created by Gabriel Kihara on 01/05/2026.
//

#ifndef PROJECT_PID_MOTOR_H
#define PROJECT_PID_MOTOR_H

// Motor mixer output
typedef struct {
    float spinMotorOne;
    float spinMotorTwo;
    float spinMotorThree;
    float spinMotorFour;
} output;

// Estimator inputs and outputs — single struct flowing through the whole pipeline
typedef struct {
    // Current angles on each axis
    float cur_roll, cur_pitch, cur_yaw;
    // Angle setpoints
    float roll_setpoint, pitch_setpoint, yaw_setpoint;

    // Current acceleration on each axis (body frame, gravity not removed)
    float cur_accX, cur_accY, cur_accZ;
    // Acceleration setpoints
    float accX_setpoint, accY_setpoint, accZ_setpoint;

    // Timestep — shared across estimator and PIDs (single source of truth)
    float dt;
} estimator;

// PID output values
typedef struct {
    float roll_pid, pitch_pid, yaw_pid;
    float accX_pid, accY_pid, accZ_pid;
} pid;

// Per-controller state
typedef struct {
    float prev_integral;     // anti-windup integral memory
    float prev_measurement;  // for derivative-on-measurement
    float d_filtered;        // low-pass filtered derivative
} state;

// PID gains
typedef struct {
    float p_gain;
    float i_gain;
    float d_gain;
} gain;

void roll_PID (gain* roll_gains,  estimator* roll_input,  pid* roll_output,  state* roll_state);
void pitch_PID(gain* pitch_gains, estimator* pitch_input, pid* pitch_output, state* pitch_state);
void yaw_PID  (gain* yaw_gains,   estimator* yaw_input,   pid* yaw_output,   state* yaw_state);

void accX_PID(gain* X_acc_gains, estimator* X_acc, pid* accx_output, state* accX_state);
void accY_PID(gain* Y_acc_gains, estimator* Y_acc, pid* accy_output, state* accY_state);
void accZ_PID(gain* Z_acc_gains, estimator* Z_acc, pid* accz_output, state* accZ_state);

void motor_mixer(output* spinMotors, estimator* estimator_angle, pid* pid_outputs);

#endif // PROJECT_PID_MOTOR_H