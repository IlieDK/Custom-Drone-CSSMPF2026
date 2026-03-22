/*
 * imu_types.h
 *
 *  Created on: 2026. 3. 21.
 *      Author: yeongyun
 */

#ifndef INC_IMU_TYPES_H_
#define INC_IMU_TYPES_H_

typedef struct {
	float ax, ay, az;
	float gx, gy, gz;
	float dt;
} IMU_Data;

typedef struct {
	float roll;
	float pitch;
	float yaw;
} Attitude;

typedef struct {
	float ax_bias, ay_bias, az_bias;
	float gx_bias, gy_bias, gz_bias;
	float alpha;
} Estimator_Config;

#endif /* INC_IMU_TYPES_H_ */
