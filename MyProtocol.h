/*
 * MyProtocol.h
 *
 *  Created on: 2017年6月29日
 *      Author: zbf
 */

#ifndef MYPROTOCOL_H_
#define MYPROTOCOL_H_
#include <string.h>
#include "uart.h"

//#define PARAM_DEBUG_MODE
#define DEBUG_DATA_MODE

typedef struct __param_debug{
	int thrust;
	float kp_v;
	float ki_v;
	float kp_p;
	float ki_p;

	float z;
	float vz;
	float set_velocity;
	float calc_thrust;
}ParamDebug;

typedef struct __my_debug_data{
	//unit: packages
	int timestamp;
	//unit: mm
	float z;
	//unit: mm/s
	float vz;
	//unit mV
	int battery;
	int cpu_load;
	int vicon_count;

	float set_position;
	float set_velocity;
	float calc_thrust;

}DebugData;

typedef struct __my_vicon_data{
	//unit ms
	int timestamp;
	//unit: mm
	float x;
	float y;
	float z;
	//unit: degree
	float roll;
	float pitch;
	float yaw;
	//unit: mm/s
	float vx;
	float vy;
	float vz;
}MyViconData;

typedef struct __system_state{
	//unit mV
	int battery;
	int cpu_load;
	int vicon_count;
}SystemState;

typedef struct __sensor_data{
	//unit -10000..+10000 = -1g..+1g
	int acc_x;
	int acc_y;
	int acc_z;
}SensorData;

typedef struct __fusion_Data{
	//unit mm
	int fusion_height;
	//unit mm/s
	//	int fd,
	//			void* buffer,
	//			void* data,
	//			unsigned char check
	int fusion_dheight;
}FusionData;

typedef enum __package_define{
	PACKAGE_DEFINE_ALL,
	PACKAGE_DEFINE_STATUS,
	PACKAGE_DEFINE_VICON,
	PACKAGE_DEFINE_SENSOR,
	PACKAGE_DEFINE_FUSION,
	PACKAGE_DEFINE_DEBUG,
	PACKAGE_DEFINE_PARAM,
	PACKAGE_DEFINE_CMD
}PackageDefine;

typedef struct __cmd_data{
	int cmd;
}CmdData;

#define VICON_DATA_LENGTH ((unsigned char)(sizeof(MyViconData)))
#define SYSTEM_STATE_LENGTH ((unsigned char)(sizeof(SystemState)))
#define SENSOR_DATA_LENGTH ((unsigned char)(sizeof(SensorData)))
#define FUSION_DATA_LENGTH ((unsigned char)(sizeof(FusionData)))
#define DEBUG_DATA_LENGTH ((unsigned char)(sizeof(DebugData)))
#define PARAM_DEBUG_LENGTH ((unsigned char)(sizeof(ParamDebug)))
#define CMD_DATA_LENGTH ((unsigned char)(sizeof(CmdData)))

typedef enum __parse_status{
	PARSE_NOT_START,
	PARSE_READ_ID,
	PARSE_READ_LEN,
	PARSE_READ_DATA,
	PARSE_CHECK,
	PARSE_SUCCEED,
	PARSE_FAIL,
}ParseStatus;

typedef struct __package_info{
	unsigned char header;
	unsigned char packageIndex;
	unsigned char len;
}PackageInfo;
#define BUFFER_LENGTH 255

//remote control only
typedef struct __my_sql_data{
	MyViconData viconData;
	SystemState state;
	float set_position;
	float set_velocity;
	float calc_thrust;
}SqlData;

void my_send(int fd
		,PackageDefine pd
		,unsigned char pl
		,void* data
		,unsigned char check);

typedef enum __receive_state{
	RECEIVE_STATE_NOT_COMPLETED,
	RECEIVE_STATE_GOT_TYPE,
	RECEIVE_STATE_SUCCESS,
}ReceiveState;

void send_single(int fd,unsigned char c);
ReceiveState my_receive(int fd,void* buffer,void* data,
		int* id,unsigned char check);
int receive_single(int fd,unsigned char* result);

unsigned char getPackageLength(PackageDefine pd);

extern long signed int (*read_callback)(int,void*,unsigned long);
extern long signed int (*write_callback)(int,const void*,unsigned long int);
#endif /* MYPROTOCOL_H_ */
