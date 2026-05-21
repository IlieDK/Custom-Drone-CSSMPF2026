/*
 * estimator.h
 *
 *  Created on: 2026. 3. 21.
 *      Author: yeongyun
 */

// public API for the attitude estimator
#ifndef INC_ESTIMATOR_H_
#define INC_ESTIMATOR_H_

#include "imu_types.h"
#include "pid_motor_mixer.h"

void Estimator_Init(void); // zero out internal state, set default config
void Estimator_SetConfig(Estimator_Config *cfg); // change biases or alpha
void Estimator_Update(const IMU_Data *imu, estimator *est); // run one tick

#endif