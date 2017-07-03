/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 * position_estimator_pid.c: PID-based implementation of the position controller
 */


#include "pid.h"
#include "position_controller.h"

// Maximum roll/pitch angle permited
//static float rpLimit  = 20;
//static float rpLimitOverhead = 1.10f;
// Velocity maximums
 float zVelMax  = 10.0f;
 float velMaxOverhead = 1.10f;
 const float thrustScale = 1000.0f;

#define POSITION_RATE 100
#define DT (float)(1.0/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 20.0f
#define POSITION_LPF_ENABLE true

struct this_s this = {

  .pidVZ = {
    .init = {
      .kp = 1,
      .ki = 0.05f,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidZ = {
    .init = {
      .kp = 0.16f,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .thrustBase = 1850,
  .thrustMin  = 600,
};

void positionEstimate(state_t* estimate,
		const float vicon_data,
		const long timestamp,
		EstimateParams* params,
		const float dt){
	float filteredZ;
	static float prev_estimatedZ=0;
	static int estimateMode=0;
	if(estimate->timestamp==timestamp){
		filteredZ=(params->alpha)*params->estimatedZ
				+(1.0f-params->alpha)*vicon_data;
		params->estimatedZ=filteredZ+params->estimatedVZ*dt;
	}else{
		if(params->estimatedZ==0.0f){
			filteredZ=vicon_data;
		}else{
			filteredZ=(params->alpha);
		}
	}
}
void positionControllerInit ()
{
 pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.init.kp, this.pidZ.init.ki, this.pidZ.init.kd,
      this.pidZ.pid.dt);
  pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp, this.pidVZ.init.ki, this.pidVZ.init.kd,
      this.pidVZ.pid.dt);
}

float runPid(float input, struct pidAxis_s *axis,  float setpoint, float dt) {
  axis->setpoint = setpoint;
  pidSetDesired(&(axis->pid), axis->setpoint);
  return pidUpdate(&(axis->pid), input, 1);
}
float data_fusion(int flag,float vicon_z,int acc_z){
	static float result=0;
	static float acc_get_vel=0;
	static float acc_get_pos=0;
	float acc;
	acc=(acc_z-10000)/GRAVITY_ACC;
	if(flag==1){
		result=vicon_z;
		acc_get_vel=0;
		acc_get_pos=0;
	}else if(flag==0){
		acc_get_vel+=acc_z*DT;
		acc_get_pos+=acc_get_vel;
		result+=acc_get_pos;
	}
	return result;
}
void positionController(float* thrust,
                                                             const state_t *state)
{
	float thrustRaw;
  this.pidZ.pid.outputLimit = max(zVelMax, 80.0f)  * velMaxOverhead;
    my_setpoint.velocity.z = runPid(state->position.z, &this.pidZ, my_setpoint.position.z, DT);
    this.pidVZ.pid.outputLimit =1000.0f;
    // Thrust
    thrustRaw = runPid(state->velocity.z, &this.pidVZ, my_setpoint.velocity.z, DT);
    // Scale the thrust and add feed forward term
    *thrust = thrustRaw + this.thrustBase;
    // Check for minimum thrust
    if (*thrust < this.thrustMin) {
      *thrust = this.thrustMin;
    }
}

void positionControllerResetAllPID()
{
  pidReset(&this.pidZ.pid);
  pidReset(&this.pidVZ.pid);
}
