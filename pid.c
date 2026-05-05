//
// Created by Gabriel Kihara on 01/05/2026.
//

#include "pid_motor_mixer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

const float integral_max_lim =  1000.0f;
const float integral_min_lim = -1000.0f;

const float min_PID_value = -200.0f;
const float max_PID_value =  200.0f;

// One-pole IIR cutoff: alpha closer to 1 = heavier filtering (more lag,
// less noise). 0.3 corresponds to ~22 Hz cutoff at a 100 Hz loop rate,
// which is a sensible default for attitude control.
static const float D_FILTER_ALPHA = 0.3f;

static float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

static float PID(gain* gains, float error, float measurement, state* st, float dt) {

    /* Integral with anti-windup clamp */
    float raw_integral = st->prev_integral + (error * dt);
    if (raw_integral >  integral_max_lim) raw_integral =  integral_max_lim;
    if (raw_integral <  integral_min_lim) raw_integral =  integral_min_lim;
    st->prev_integral = raw_integral;

    /* Derivative on measurement, with one-pole low-pass filter */
    float d_meas = 0.0f;
    if (dt > 0.0f) {
        d_meas = (measurement - st->prev_measurement) / dt;
    }
    st->prev_measurement = measurement;

    st->d_filtered = D_FILTER_ALPHA * st->d_filtered + (1.0f - D_FILTER_ALPHA) * d_meas;

    /* Note the minus sign on the D term: with derivative-on-measurement
       and a constant setpoint, d(error)/dt = -d(measurement)/dt. */
    float pid_output = gains->p_gain * error + gains->i_gain * raw_integral - gains->d_gain * st->d_filtered;

    if (pid_output >  max_PID_value) pid_output =  max_PID_value;
    if (pid_output <  min_PID_value) pid_output =  min_PID_value;

    return pid_output;
}

void roll_PID(gain* roll_gains, estimator* roll_input, pid* roll_output, state* roll_state) {
    float error = roll_input->roll_setpoint - roll_input->cur_roll;
    roll_output->roll_pid = PID(roll_gains, error, roll_input->cur_roll, roll_state, roll_input->dt);
}

void pitch_PID(gain* pitch_gains, estimator* pitch_input, pid* pitch_output, state* pitch_state) {
    float error = pitch_input->pitch_setpoint - pitch_input->cur_pitch;
    pitch_output->pitch_pid = PID(pitch_gains, error, pitch_input->cur_pitch, pitch_state, pitch_input->dt);
}

void yaw_PID(gain* yaw_gains, estimator* yaw_input, pid* yaw_output, state* yaw_state) {
  
    float error = wrap_pi(yaw_input->yaw_setpoint - yaw_input->cur_yaw);
    yaw_output->yaw_pid = PID(yaw_gains, error, yaw_input->cur_yaw, yaw_state, yaw_input->dt);
}

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