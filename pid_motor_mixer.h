// pid_motor_mixer.h
// struct definitions for the PID and motor mixer modules

#ifndef PROJECT_PID_MOTOR_H
#define PROJECT_PID_MOTOR_H

// motor mixer output - one command per motor
typedef struct {
    float spinMotorOne;
    float spinMotorTwo;
    float spinMotorThree;
    float spinMotorFour;
} output;

// the big struct that flows through the whole pipeline
typedef struct {
    // current estimated angles
    float cur_roll, cur_pitch, cur_yaw;
    // angle setpoints (what we want)
    float roll_setpoint, pitch_setpoint, yaw_setpoint;

    // current body-frame accelerations (gravity not removed)
    float cur_accX, cur_accY, cur_accZ;
    // acceleration setpoints (not used yet - reserved for position control)
    float accX_setpoint, accY_setpoint, accZ_setpoint;

    float dt; // timestep, one source of truth for estimator and PIDs
} estimator;

// what each PID spits out
typedef struct {
    float roll_pid, pitch_pid, yaw_pid;
    float accX_pid, accY_pid, accZ_pid;
} pid;

// per-PID memory - integrator, last measurement (for D-on-measurement), filtered D
typedef struct {
    float prev_integral;
    float prev_measurement;
    float d_filtered;
} state;

// PID gains - one set per axis
typedef struct {
    float p_gain;
    float i_gain;
    float d_gain;
} gain;

// the three attitude PIDs
void roll_PID (gain* roll_gains,  estimator* roll_input,  pid* roll_output,  state* roll_state);
void pitch_PID(gain* pitch_gains, estimator* pitch_input, pid* pitch_output, state* pitch_state);
void yaw_PID  (gain* yaw_gains,   estimator* yaw_input,   pid* yaw_output,   state* yaw_state);

// the three acceleration PIDs (reserved - not driven yet)
void accX_PID(gain* X_acc_gains, estimator* X_acc, pid* accx_output, state* accX_state);
void accY_PID(gain* Y_acc_gains, estimator* Y_acc, pid* accy_output, state* accY_state);
void accZ_PID(gain* Z_acc_gains, estimator* Z_acc, pid* accz_output, state* accZ_state);

// turn PID outputs into per-motor commands
void motor_mixer(output* spinMotors, estimator* estimator_angle, pid* pid_outputs);

#endif