/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#ifndef POSITION_CONTROLLER_H_
#define POSITION_CONTROLLER_H_

#include "common_header.h"
//#include "stabilizer_types.h"
#define POSITION_RATE 100
#define GRAVITY_ACC 9.764f
extern state_t my_setpoint;
// A position controller calculate the thrust, roll, pitch to approach
// a 3D position setpoint
void positionEstimate(state_t* estimate,
		const float vicon_data,
		const long timestamp,
		EstimateParams* params,
		const float dt);
void positionControllerInit(void);
void positionControllerResetAllPID(void);
float runPid(float input, struct pidAxis_s *axis,  float setpoint, float dt);

void positionController(float* thrust,float* pitch,float* roll,
                                                             const state_t *state);
float data_fusion(int flag,float vicon_z,int acc_z);

#endif /* POSITION_CONTROLLER_H_ */
