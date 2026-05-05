/*
 * estimator.c
 *
 *  Created on: 2026. 3. 22.
 *      Author: yeongyun
 */

#include "imu_types.h"
#include "estimator.h"
#include <math.h>
#include <stddef.h>   /* NULL */

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

static float prev_roll, prev_pitch, prev_yaw;
static Estimator_Config estimator_config;

static void preprocess_IMU(const IMU_Data *input, IMU_Data *output);
static void compute_accel_angles(const IMU_Data *imu, float *roll_acc, float *pitch_acc);
static void integrate_gyro(const IMU_Data *imu, float dt, float *roll_gyro, float *pitch_gyro, float *yaw_gyro);
static void apply_complementary_filter(float roll_acc, float pitch_acc, float roll_gyro, float pitch_gyro, float yaw_gyro, estimator *est);

void Estimator_Init(void) {
    prev_roll  = 0.0f;
    prev_pitch = 0.0f;
    prev_yaw   = 0.0f;

    estimator_config.ax_bias = 0.0f;
    estimator_config.ay_bias = 0.0f;
    estimator_config.az_bias = 0.0f;

    estimator_config.gx_bias = 0.0f;
    estimator_config.gy_bias = 0.0f;
    estimator_config.gz_bias = 0.0f;

    estimator_config.alpha = 0.98f;
}

void Estimator_SetConfig(Estimator_Config *cfg) {
    if (cfg == NULL) return;
    estimator_config = *cfg;
}

void Estimator_Update(const IMU_Data *imu, estimator *est) {
    if (imu == NULL || est == NULL) return;

    IMU_Data corrected_imu;
    float roll_acc, pitch_acc;
    float roll_gyro, pitch_gyro, yaw_gyro;

    preprocess_IMU(imu, &corrected_imu);
    compute_accel_angles(&corrected_imu, &roll_acc, &pitch_acc);
    integrate_gyro(&corrected_imu, est->dt, &roll_gyro, &pitch_gyro, &yaw_gyro);
    apply_complementary_filter(roll_acc, pitch_acc, roll_gyro, pitch_gyro, yaw_gyro, est);

    est->cur_accX = corrected_imu.ax;
    est->cur_accY = corrected_imu.ay;
    est->cur_accZ = corrected_imu.az;

    prev_roll  = est->cur_roll;
    prev_pitch = est->cur_pitch;
    prev_yaw   = est->cur_yaw;
}

static void preprocess_IMU(const IMU_Data *input, IMU_Data *output) {
    if (input == NULL || output == NULL) return;

    output->ax = input->ax - estimator_config.ax_bias;
    output->ay = input->ay - estimator_config.ay_bias;
    output->az = input->az - estimator_config.az_bias;

    output->gx = input->gx - estimator_config.gx_bias;
    output->gy = input->gy - estimator_config.gy_bias;
    output->gz = input->gz - estimator_config.gz_bias;
}

static void compute_accel_angles(const IMU_Data *imu, float *roll_acc, float *pitch_acc) {
    if (imu == NULL || roll_acc == NULL || pitch_acc == NULL) return;

    *roll_acc  = atan2f(imu->ay, imu->az);
    *pitch_acc = atan2f(-imu->ax, sqrtf(imu->ay * imu->ay + imu->az * imu->az));
}

static void integrate_gyro(const IMU_Data *imu, float dt, float *roll_gyro, float *pitch_gyro, float *yaw_gyro) {
    if (imu == NULL || roll_gyro == NULL || pitch_gyro == NULL || yaw_gyro == NULL) return;

    if (dt <= 0.0f) {
        *roll_gyro  = prev_roll;
        *pitch_gyro = prev_pitch;
        *yaw_gyro   = prev_yaw;
        return;
    }

    float sin_roll = sinf(prev_roll);
    float cos_roll = cosf(prev_roll);
    float sin_pitch = sinf(prev_pitch);
    float cos_pitch = cosf(prev_pitch);

    /* Gimbal-lock guard: keep |cos(pitch)| >= ~0.017 (i.e. pitch within ~89°) */
    const float COS_PITCH_MIN = 0.017f;
    if (cos_pitch >= 0.0f && cos_pitch <  COS_PITCH_MIN) cos_pitch =  COS_PITCH_MIN;
    if (cos_pitch <  0.0f && cos_pitch > -COS_PITCH_MIN) cos_pitch = -COS_PITCH_MIN;

    float tp = sin_pitch / cos_pitch;

    float roll_dot  = imu->gx + sin_roll * tp * imu->gy + cos_roll * tp * imu->gz;
    float pitch_dot = cos_roll * imu->gy - sin_roll * imu->gz;
    float yaw_dot   = (sin_roll / cos_pitch) * imu->gy + (cos_roll / cos_pitch) * imu->gz;

    *roll_gyro  = prev_roll  + roll_dot  * dt;
    *pitch_gyro = prev_pitch + pitch_dot * dt;
    *yaw_gyro   = prev_yaw   + yaw_dot   * dt;
}

static void apply_complementary_filter(float roll_acc, float pitch_acc, float roll_gyro, float pitch_gyro, float yaw_gyro, estimator *est) {
    if (est == NULL) return;

    est->cur_roll  = estimator_config.alpha * roll_gyro  + (1.0f - estimator_config.alpha) * roll_acc;
    est->cur_pitch = estimator_config.alpha * pitch_gyro + (1.0f - estimator_config.alpha) * pitch_acc;
    /* No magnetometer → yaw is pure gyro integration; will drift slowly. */
    est->cur_yaw   = yaw_gyro;
}