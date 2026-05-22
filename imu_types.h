/*
 * imu_types.h
 *
 *  Created on: 2026. 3. 21.
 *      Author: yeongyun
 */

// shared struct definitions for IMU readings and estimator config
#ifndef INC_IMU_TYPES_H_
#define INC_IMU_TYPES_H_

// raw IMU reading - accel and gyro on all three axes
typedef struct {
    float ax, ay, az; // accelerometer m/s^2
    float gx, gy, gz; // gyroscope rad/s
} IMU_Data;

// estimator tuning - biases (subtracted from raw) and the complementary filter weight
typedef struct {
    float ax_bias, ay_bias, az_bias; // accel biases
    float gx_bias, gy_bias, gz_bias; // gyro biases
    float alpha; // complementary filter weight (closer to 1 = trust gyro more)
} Estimator_Config;

#endif