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

#include "sdk.h"
#include "pid.h"
#include "position_controller.h"

// Maximum roll/pitch angle permited
float rpLimit  = 800.0f;
float rpLimitOverhead = 1.10f;
// Velocity maximums
float xyVelMax=400.0f;
 float zVelMax  = 10.0f;
 float velMaxOverhead = 1.10f;
 const float thrustScale = 1000.0f;

#define POSITION_RATE 100
#define DT (float)(1.0/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 20.0f
#define POSITION_LPF_ENABLE true
struct this_s my_this = {
	.pidVX = {
	    .init = {
	      .kp = 1,
	      .ki = 0.4,
	      .kd = 0,
	    },
	    .pid.dt = DT,
		.pid.iLimit=200,
	  },

	  .pidVY = {
	    .init = {
	      .kp = 1.2f,
	      .ki =0.4 ,
	      .kd = 0,
	    },
	    .pid.dt = DT,
		.pid.iLimit=200,
	  },

  .pidVZ = {
    .init = {
      .kp = 1,
      .ki = 0.05f,
      .kd = 0,
    },
    .pid.dt = DT,
  },
  .pidX = {
    .init = {
      .kp = 0.4,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },
  .pidY = {
    .init = {
      .kp = 1.6f,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },
  .pidZ = {
    .init = {
      .kp = 0.8f,
      .ki = 0,
      .kd = 0,
    },
    .pid ={
    		.dt=DT,
			.iLimit=100,
    },
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
  pidInit(&my_this.pidX.pid, my_this.pidX.setpoint, my_this.pidX.init.kp, my_this.pidX.init.ki, my_this.pidX.init.kd,
      my_this.pidX.pid.dt);
  pidInit(&my_this.pidY.pid, my_this.pidY.setpoint, my_this.pidY.init.kp, my_this.pidY.init.ki, my_this.pidY.init.kd,
      my_this.pidY.pid.dt);
  pidInit(&my_this.pidZ.pid, my_this.pidZ.setpoint, my_this.pidZ.init.kp, my_this.pidZ.init.ki, my_this.pidZ.init.kd,
      my_this.pidZ.pid.dt);

  pidInit(&my_this.pidVX.pid, my_this.pidVX.setpoint, my_this.pidVX.init.kp, my_this.pidVX.init.ki, my_this.pidVX.init.kd,
      my_this.pidVX.pid.dt);
  pidInit(&my_this.pidVY.pid, my_this.pidVY.setpoint, my_this.pidVY.init.kp, my_this.pidVY.init.ki, my_this.pidVY.init.kd,
      my_this.pidVY.pid.dt);
  pidInit(&my_this.pidVZ.pid, my_this.pidVZ.setpoint, my_this.pidVZ.init.kp, my_this.pidVZ.init.ki, my_this.pidVZ.init.kd,
      my_this.pidVZ.pid.dt);

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
void positionController(float* thrust,float* pitch,float* roll,
                                                             const state_t *state)
{
	float thrustRaw;
	static int y_temp=-1000;
	my_this.pidX.pid.outputLimit = xyVelMax  * velMaxOverhead;
	my_this.pidY.pid.outputLimit =  xyVelMax  * velMaxOverhead;
    my_this.pidZ.pid.outputLimit = max(zVelMax, 200.0f)  * velMaxOverhead;

//    float cosyaw = cosf(state->attitude.yaw * DEG_TO_RAD);
//    float sinyaw = sinf(state->attitude.yaw * DEG_TO_RAD);
//    float bodyvx = my_setpoint.velocity.x;
//    float bodyvy = my_setpoint.velocity.y;

    my_setpoint.velocity.x = runPid(state->position.x, &my_this.pidX, my_setpoint.position.x, DT);
//    my_setpoint.velocity.y = runPid(state->position.y, &my_this.pidY, my_setpoint.position.y, DT);
//    my_setpoint.velocity.x=0;
    if(y_temp++>400){
    	my_setpoint.velocity.y=500;
    }else{
    	my_setpoint.velocity.y=-500;
    }
    if(y_temp>800)y_temp=0;
    my_setpoint.velocity.z = runPid(state->position.z, &my_this.pidZ, my_setpoint.position.z, DT);

    my_this.pidVX.pid.outputLimit = rpLimit * rpLimitOverhead;
    my_this.pidVY.pid.outputLimit = rpLimit * rpLimitOverhead;
    my_this.pidVZ.pid.outputLimit =250.0f;

    float rollRaw  = runPid(state->velocity.x, &my_this.pidVX, my_setpoint.velocity.x, DT);
      float pitchRaw = runPid(state->velocity.y, &my_this.pidVY, my_setpoint.velocity.y, DT);

     float yawRad = state->attitude.yaw ;
     // float yawRad = 0 ;
      *pitch = -(rollRaw  * cosf(yawRad)) - (pitchRaw * sinf(yawRad));
      *roll  = -(pitchRaw * cosf(yawRad)) + (rollRaw  * sinf(yawRad));

      *roll  = constrain(*roll,  -rpLimit, rpLimit);
      *pitch = constrain(*pitch, -rpLimit, rpLimit);

    // Thrust
    thrustRaw = runPid(state->velocity.z, &my_this.pidVZ, my_setpoint.velocity.z, DT);
    // Scale the thrust and add feed forward term
    *thrust = thrustRaw + my_this.thrustBase;
    // Check for minimum thrust
    if (*thrust < my_this.thrustMin) {
      *thrust = my_this.thrustMin;
    }
}

void positionControllerResetAllPID()
{
  pidReset(&my_this.pidZ.pid);
  pidReset(&my_this.pidVZ.pid);
}
