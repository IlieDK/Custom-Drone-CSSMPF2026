
// generic PID with anti-windup, output clamp, D-on-measurement, low-pass on D, yaw wrap

#include "pid_motor_mixer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

// anti-windup clamp on the integrator
const float integral_max_lim =  1000.0f;
const float integral_min_lim = -1000.0f;

// hard clamp on the final output
const float min_PID_value = -200.0f;
const float max_PID_value =  200.0f;

// one-pole low-pass on the D term - closer to 1 = heavier filter
static const float D_FILTER_ALPHA = 0.3f;

// wrap an angle into [-pi, pi] so yaw error takes the short way around
static float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

// the actual PID - takes error and measurement separately
static float PID(gain* gains, float error, float measurement, state* st, float dt) {

    // integrate the error and clamp to stop wind-up
    float raw_integral = st->prev_integral + (error * dt);
    if (raw_integral >  integral_max_lim) raw_integral =  integral_max_lim;
    if (raw_integral <  integral_min_lim) raw_integral =  integral_min_lim;
    st->prev_integral = raw_integral;

    // derivative on the measurement (not the error) - no kick on setpoint steps
    float d_meas = 0.0f;
    if (dt > 0.0f) {
        d_meas = (measurement - st->prev_measurement) / dt;
    }
    st->prev_measurement = measurement;

    // one-pole low-pass on the D term
    st->d_filtered = D_FILTER_ALPHA * st->d_filtered + (1.0f - D_FILTER_ALPHA) * d_meas;

    // sum the three terms - minus on D because d(error)/dt = -d(measurement)/dt for a constant setpoint
    float pid_output = gains->p_gain * error + gains->i_gain * raw_integral - gains->d_gain * st->d_filtered;

    // clamp the output
    if (pid_output >  max_PID_value) pid_output =  max_PID_value;
    if (pid_output <  min_PID_value) pid_output =  min_PID_value;

    return pid_output;
}

// wrappers - one per axis, all calling the same internal PID
void roll_PID(gain* roll_gains, estimator* roll_input, pid* roll_output, state* roll_state) {
    float error = roll_input->roll_setpoint - roll_input->cur_roll; // error = setpoint - measurement
    roll_output->roll_pid = PID(roll_gains, error, roll_input->cur_roll, roll_state, roll_input->dt);
}

void pitch_PID(gain* pitch_gains, estimator* pitch_input, pid* pitch_output, state* pitch_state) {
    float error = pitch_input->pitch_setpoint - pitch_input->cur_pitch;
    pitch_output->pitch_pid = PID(pitch_gains, error, pitch_input->cur_pitch, pitch_state, pitch_input->dt);
}

void yaw_PID(gain* yaw_gains, estimator* yaw_input, pid* yaw_output, state* yaw_state) {
    // wrap the error - yaw is circular
    float error = wrap_pi(yaw_input->yaw_setpoint - yaw_input->cur_yaw);
    yaw_output->yaw_pid = PID(yaw_gains, error, yaw_input->cur_yaw, yaw_state, yaw_input->dt);
}

// acceleration PIDs - not wired up yet, reserved for a future position controller
void accX_PID(gain* X_acc_gains, estimator* X_acc, pid* accx_output, state* accX_state) {
    float error = X_acc->accX_setpoint - X_acc->cur_accX;
    accx_output->accX_pid = PID(X_acc_gains, error, X_acc->cur_accX, accX_state, X_acc->dt);
}

void accY_PID(gain* Y_acc_gains, estimator* Y_acc, pid* accy_output, state* accY_state) {
    float error = Y_acc->accY_setpoint - Y_acc->cur_accY;
    accy_output->accY_pid = PID(Y_acc_gains, error, Y_acc->cur_accY, accY_state, Y_acc->dt);
}

void accZ_PID(gain* Z_acc_gains, estimator* Z_acc, pid* accz_output, state* accZ_state) {
    float error = Z_acc->accZ_setpoint - Z_acc->cur_accZ;
    accz_output->accZ_pid = PID(Z_acc_gains, error, Z_acc->cur_accZ, accZ_state, Z_acc->dt);
}
