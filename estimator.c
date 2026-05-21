/*
 * estimator.c
 *
 *  Created on: 2026. 3. 22.
 *      Author: yeongyun
 */

// estimator.c
// complementary filter attitude estimator

#include "imu_types.h"
#include "estimator.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

// static state - persists across ticks
static float prev_roll, prev_pitch, prev_yaw;
static Estimator_Config estimator_config;

// forward decls for the helpers
static void preprocess_IMU(const IMU_Data *input, IMU_Data *output);
static void compute_accel_angles(const IMU_Data *imu, float *roll_acc, float *pitch_acc);
static void integrate_gyro(const IMU_Data *imu, float dt, float *roll_gyro, float *pitch_gyro, float *yaw_gyro);
static void apply_complementary_filter(float roll_acc, float pitch_acc, float roll_gyro, float pitch_gyro, float yaw_gyro, estimator *est);


// reset everything to zero, set sensible defaults
void Estimator_Init(void) {
    prev_roll  = 0.0f;
    prev_pitch = 0.0f;
    prev_yaw   = 0.0f;

    // zero biases - the caller can set real ones via SetConfig
    estimator_config.ax_bias = 0.0f;
    estimator_config.ay_bias = 0.0f;
    estimator_config.az_bias = 0.0f;

    estimator_config.gx_bias = 0.0f;
    estimator_config.gy_bias = 0.0f;
    estimator_config.gz_bias = 0.0f;

    estimator_config.alpha = 0.98f; // default - trust gyro more
}

// copy a new config in
void Estimator_SetConfig(Estimator_Config *cfg) {
    if (cfg == NULL) return; // guard against null pointer
    estimator_config = *cfg;
}

// run one tick - this is the main entry point
void Estimator_Update(const IMU_Data *imu, estimator *est) {
    if (imu == NULL || est == NULL) return; // guard

    // locals for the intermediate results
    IMU_Data corrected_imu;
    float roll_acc, pitch_acc;
    float roll_gyro, pitch_gyro, yaw_gyro;

    // pipeline: bias correction -> two angle paths -> blend
    preprocess_IMU(imu, &corrected_imu);
    compute_accel_angles(&corrected_imu, &roll_acc, &pitch_acc);
    integrate_gyro(&corrected_imu, est->dt, &roll_gyro, &pitch_gyro, &yaw_gyro);
    apply_complementary_filter(roll_acc, pitch_acc, roll_gyro, pitch_gyro, yaw_gyro, est);

    // expose the bias-corrected accels for downstream consumers
    est->cur_accX = corrected_imu.ax;
    est->cur_accY = corrected_imu.ay;
    est->cur_accZ = corrected_imu.az;

    // remember this tick's angles for the next call
    prev_roll  = est->cur_roll;
    prev_pitch = est->cur_pitch;
    prev_yaw   = est->cur_yaw;
}

// subtract the configured biases from the raw reading
static void preprocess_IMU(const IMU_Data *input, IMU_Data *output) {
    if (input == NULL || output == NULL) return;

    // accel
    output->ax = input->ax - estimator_config.ax_bias;
    output->ay = input->ay - estimator_config.ay_bias;
    output->az = input->az - estimator_config.az_bias;

    // gyro
    output->gx = input->gx - estimator_config.gx_bias;
    output->gy = input->gy - estimator_config.gy_bias;
    output->gz = input->gz - estimator_config.gz_bias;
}

// derive roll and pitch from the gravity vector
static void compute_accel_angles(const IMU_Data *imu, float *roll_acc, float *pitch_acc) {
    if (imu == NULL || roll_acc == NULL || pitch_acc == NULL) return;

    // atan2 of the gravity components - this is the textbook formula
    *roll_acc  = atan2f(imu->ay, imu->az);
    *pitch_acc = atan2f(-imu->ax, sqrtf(imu->ay * imu->ay + imu->az * imu->az));
}

// integrate body angular velocity into Euler angles
static void integrate_gyro(const IMU_Data *imu, float dt, float *roll_gyro, float *pitch_gyro, float *yaw_gyro) {
    if (imu == NULL || roll_gyro == NULL || pitch_gyro == NULL || yaw_gyro == NULL) return;

    // if dt is zero or negative, just hold the previous values (don't divide by zero either)
    if (dt <= 0.0f) {
        *roll_gyro  = prev_roll;
        *pitch_gyro = prev_pitch;
        *yaw_gyro   = prev_yaw;
        return;
    }

    // trig values at the previous attitude - used by the transformation below
    float sin_roll = sinf(prev_roll);
    float cos_roll = cosf(prev_roll);
    float sin_pitch = sinf(prev_pitch);
    float cos_pitch = cosf(prev_pitch);

    // gimbal lock guard - keep cos(pitch) away from 0 so we don't divide by it
    const float COS_PITCH_MIN = 0.017f; // ~89 degrees
    if (cos_pitch >= 0.0f && cos_pitch <  COS_PITCH_MIN) cos_pitch =  COS_PITCH_MIN;
    if (cos_pitch <  0.0f && cos_pitch > -COS_PITCH_MIN) cos_pitch = -COS_PITCH_MIN;

    float tp = sin_pitch / cos_pitch; // tan(pitch)

    // body rates -> Euler rates (ZYX convention)
    float roll_dot  = imu->gx + sin_roll * tp * imu->gy + cos_roll * tp * imu->gz;
    float pitch_dot = cos_roll * imu->gy - sin_roll * imu->gz;
    float yaw_dot   = (sin_roll / cos_pitch) * imu->gy + (cos_roll / cos_pitch) * imu->gz;

    // integrate forward by one tick
    *roll_gyro  = prev_roll  + roll_dot  * dt;
    *pitch_gyro = prev_pitch + pitch_dot * dt;
    *yaw_gyro   = prev_yaw   + yaw_dot   * dt;
}

// blend the two angle estimates with weight alpha
static void apply_complementary_filter(float roll_acc, float pitch_acc, float roll_gyro, float pitch_gyro, float yaw_gyro, estimator *est) {
    if (est == NULL) return;

    // alpha * gyro + (1 - alpha) * accel for roll and pitch
    est->cur_roll  = estimator_config.alpha * roll_gyro  + (1.0f - estimator_config.alpha) * roll_acc;
    est->cur_pitch = estimator_config.alpha * pitch_gyro + (1.0f - estimator_config.alpha) * pitch_acc;

    // yaw has no accel reference (gravity is along yaw), so it's pure gyro - it will drift
    est->cur_yaw   = yaw_gyro;
}