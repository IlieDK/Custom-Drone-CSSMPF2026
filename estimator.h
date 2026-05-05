/*
 * estimator.h
 *
 *  Created on: 2026. 3. 21.
 *      Author: yeongyun
 */

#ifndef INC_ESTIMATOR_H_
#define INC_ESTIMATOR_H_

#include "imu_types.h"
#include "pid_motor_mixer.h"

void Estimator_Init(void);
void Estimator_SetConfig(Estimator_Config *cfg);
void Estimator_Update(const IMU_Data *imu, estimator *est);

#endif /* INC_ESTIMATOR_H_ */