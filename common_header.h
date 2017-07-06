/*
 * common_header.h
 *
 *  Created on: 2017年6月24日
 *      Author: zbf
 */

#ifndef COMMON_HEADER_H_
#define COMMON_HEADER_H_

#define MY_PI 3.1415926535898
#define RAD_TO_DEG ((float)(57.295779513082))
#define DEG_TO_RAD ((float)(0.017453292519943))

/** Attitude in euler angle form */
typedef struct attitude_s {
  long int timestamp;  // Timestamp when the data was computed

  float roll;
  float pitch;
  float yaw;
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
	  attitude_t attitude;
	  long timestamp;
} state_t;
typedef state_t setpoint_t;
typedef struct __estimate_params{
	float alpha;
	float estimatedZ;
	float estimatedVZ;
}EstimateParams;

#endif /* COMMON_HEADER_H_ */
