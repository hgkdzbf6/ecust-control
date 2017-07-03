/*
 * common_header.h
 *
 *  Created on: 2017年6月24日
 *      Author: zbf
 */

#ifndef COMMON_HEADER_H_
#define COMMON_HEADER_H_

/** Attitude in euler angle form */
typedef struct attitude_s {
  long int timestamp;  // Timestamp when the data was computed

  double roll;
  double pitch;
  double yaw;
} attitude_t;

/* x,y,z vector */
struct vec3_s {
  volatile float x;
  volatile float y;
  volatile float z;
};

typedef struct vec3_s velocity_t;
typedef struct vec3_s point_t;

typedef struct state_s {
	  point_t position;
	  velocity_t velocity;
	  long timestamp;
} state_t;
typedef state_t setpoint_t;
typedef struct __estimate_params{
	float alpha;
	float estimatedZ;
	float estimatedVZ;
}EstimateParams;

#endif /* COMMON_HEADER_H_ */
