/*
 * estimator.c
 *
 *  Created on: 2026. 3. 22.
 *      Author: yeongyun
 */

#include "imu_types.h"
#include "estimator.h"
#include "math.h"

// 전역으로 선언한 해당 변수들은 estimator.c 내부에서 자유롭게 접근 가능
static float prev_roll, prev_pitch, prev_yaw;	// previous attitude values -> current values will be calculated at Estimator_Update()
static Estimator_Config estimator_config;		// configuration values

static void preprocess_IMU(const IMU_Data *input, IMU_Data *output);
static void compute_accel_angles(const IMU_Data *imu, float *roll_acc, float *pitch_acc);
static void integrate_gyro(const IMU_Data *imu, float *roll_gyro, float *pitch_gyro, float *yaw_gyro);
static void apply_complementary_filter(float roll_acc, float pitch_acc,
										float roll_gyro, float pitch_gyro, float yaw_gyro,
										Attitude *att);

// initialize internal state
void Estimator_Init(){

	// initialize previous values
	prev_roll = 0.0f;
	prev_pitch = 0.0f;
	prev_yaw = 0.0f;

	// initialize configuration values
	estimator_config.ax_bias = 0.0f;
	estimator_config.ay_bias = 0.0f;
	estimator_config.az_bias = 0.0f;

	estimator_config.gx_bias = 0.0f;
	estimator_config.gy_bias = 0.0f;
	estimator_config.gz_bias = 0.0f;

	estimator_config.alpha = 0.98f;

}

// for tuning & calibration values -> proper values will be found through experiments
void Estimator_SetConfig(Estimator_Config *cfg){

	// check input pointer
	if(cfg == NULL){
		return;
	}

	estimator_config = *cfg;

}

// input: IMU data -> calculate roll, pitch, and yaw
void Estimator_Update(const IMU_Data *imu, Attitude *att){

	// check input pointers
	if(imu == NULL || att == NULL){
		return;
	}

	// local variables for corrected IMU values, calculated roll & pitch
	IMU_Data corrected_imu;
	float roll_acc, pitch_acc;
	float roll_gyro, pitch_gyro, yaw_gyro;

	// preprocess: bias removal
	preprocess_IMU(imu, &corrected_imu);

	// calculate accelerometer based roll & pitch
	compute_accel_angles(&corrected_imu, &roll_acc, &pitch_acc);

	// integrate gyroscope data -> roll, pitch, yaw
	integrate_gyro(&corrected_imu, &roll_gyro, &pitch_gyro, &yaw_gyro);

	// complementary filter fusion
	apply_complementary_filter(roll_acc, pitch_acc, roll_gyro, pitch_gyro, yaw_gyro, att);

	// update previous state
	prev_roll = att->roll;
	prev_pitch = att->pitch;
	prev_yaw = att->yaw;

}

// raw IMU data -> bias removed IMU data
static void preprocess_IMU(const IMU_Data *input, IMU_Data *output){

	// check input pointers
	if(input == NULL || output == NULL){
		return;
	}

	output->ax = input->ax - estimator_config.ax_bias;
	output->ay = input->ay - estimator_config.ay_bias;
	output->az = input->az - estimator_config.az_bias;

	output->gx = input->gx - estimator_config.gx_bias;
	output->gy = input->gy - estimator_config.gy_bias;
	output->gz = input->gz - estimator_config.gz_bias;

	output->dt = input->dt;

}

// bias removed IMU data -> roll_acc, pitch_acc
static void compute_accel_angles(const IMU_Data *imu, float *roll_acc, float *pitch_acc){

	// check input pointers
	if(imu == NULL || roll_acc == NULL || pitch_acc == NULL){
		return;
	}

	*roll_acc = atan2f(imu->ay, imu->az);
	*pitch_acc = atan2f(-imu->ax, sqrtf(imu->ay * imu->ay + imu->az * imu->az));

}

// bias removed IMU data + prev attitude + dt -> roll_gyro, pitch_gyro, yaw_gyro
static void integrate_gyro(const IMU_Data *imu, float *roll_gyro, float *pitch_gyro, float *yaw_gyro){

	// check input pointers
	if(imu == NULL || roll_gyro == NULL || pitch_gyro == NULL || yaw_gyro == NULL){
		return;
	}

	// check dt value
	if(imu->dt <= 0.0f){
		return;
	}

	// gyro = rad/s, dt = s -> gyro * dt = rad
	*roll_gyro = prev_roll + imu->gx * imu->dt;
	*pitch_gyro = prev_pitch + imu->gy * imu->dt;
	*yaw_gyro = prev_yaw + imu->gz * imu->dt;

}

// accel angle + gyro angle + alpha -> final roll, pitch, and yaw
static void apply_complementary_filter(float roll_acc, float pitch_acc,
										float roll_gyro, float pitch_gyro, float yaw_gyro,
										Attitude *att){

	// check input pointer
	if(att == NULL){
		return;
	}

	att->roll = estimator_config.alpha * roll_gyro + (1.0f - estimator_config.alpha) * roll_acc;
	att->pitch = estimator_config.alpha * pitch_gyro + (1.0f - estimator_config.alpha) * pitch_acc;
	att->yaw = yaw_gyro;

}
