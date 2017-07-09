/*
 * MyProtocol.cpp
 *
 *  Created on: 2017年6月29日
 *      Author: zbf
 */

#include "MyProtocol.h"


void my_send(int fd
		,PackageDefine pd
		,unsigned char pl
		,void* data
		,unsigned char check){
	int i=0;
	unsigned char t;
	unsigned char crc;
	//send header
	send_single(fd,254);
	//send package define
	send_single(fd,(unsigned char)pd);
	//send package length
	send_single(fd,pl);
	if(check==0){
		for(i=0;i<pl;i++){
			t=*((unsigned char*)data+i);
			send_single(fd,t);
		}
	}else{
		crc=254+pd+pl;
		for(i=0;i<pl;i++){
			t=*((unsigned char*)data+i);
			send_single(fd,t);
			crc+=t;
		}
	}
	if(check==0){
		send_single(fd,0);
	}else{
		send_single(fd,crc);
	}
}

void send_single(int fd,unsigned char c){
	UARTWriteChar(c);//read_callback(fd,&c,1);
}

int receive_single(int fd,unsigned char* result){
	*result=(unsigned char)fd;
	return 1;
}

//return 0: package is not completed
//return 1:package is available
ReceiveState my_receive(
		int fd,
		void* buffer,
		void* data,
		int* id,
		unsigned char check){
	//character store temporally
	static int status;
	static ParseStatus flag=PARSE_NOT_START;
	static unsigned char t;
	//get package index
	static unsigned char my_index;
	//get package length
	static unsigned char len;
	static unsigned char i=0;
	static unsigned char crc;
	status=receive_single(fd,&t);
	if(status!=0){
		//some data received
		if(t==254&&flag==PARSE_NOT_START){
			crc=254;
			flag=PARSE_READ_ID;
		}else if(flag==PARSE_READ_ID){
			my_index=t;
			crc+=t;
			flag=PARSE_READ_LEN;
		}else if(flag==PARSE_READ_LEN){
			len=t;
			i=0;
			crc+=t;
			flag=PARSE_READ_DATA;
		}else if(flag==PARSE_READ_DATA){
			memset(((unsigned char*)buffer+i),t,1);
			i++;
			crc+=t;
			if(i==len){
				flag=PARSE_CHECK;
			}
		}else if(flag==PARSE_CHECK){
			//need to check?
			//if check==0, t=0, so crc=0 ,and it also can be tested
			if(t==crc){
				flag=PARSE_SUCCEED;
			}else{
				//printf("%x    %x\n",crc,t);
				flag=PARSE_FAIL;
			}
		}
		if(flag==PARSE_SUCCEED){
			//need to check?
			memcpy(data,buffer,len);
			*id=my_index;
			status=0;flag=PARSE_NOT_START;t=0;
			my_index=0;len=0;i=0;crc=0;
			return RECEIVE_STATE_SUCCESS;
		}else if(flag==PARSE_FAIL){
			//need to check?
			//memcpy(data,buffer,len);
			memset(data,0,len);
			status=0;flag=PARSE_NOT_START;t=0;
			my_index=0;len=0;i=0;crc=0;
		}
		//printf("%d  %d  %d  %d\n",crc,len,i,t);
	}
	*id=-1;
	return RECEIVE_STATE_NOT_COMPLETED;
}
unsigned char getPackageLength(PackageDefine pd){
	switch(pd){
	case PACKAGE_DEFINE_STATUS:
		return SYSTEM_STATE_LENGTH;
	case PACKAGE_DEFINE_VICON:
		return VICON_DATA_LENGTH;
	case PACKAGE_DEFINE_SENSOR:
		return SENSOR_DATA_LENGTH;
	case PACKAGE_DEFINE_FUSION:
		return FUSION_DATA_LENGTH;
	case PACKAGE_DEFINE_DEBUG:
		return DEBUG_DATA_LENGTH;
	case PACKAGE_DEFINE_PARAM:
		return PARAM_DEBUG_LENGTH;
	case PACKAGE_DEFINE_CMD:
		return CMD_DATA_LENGTH;
	case PACKAGE_DEFINE_POSITION_WAY_POINT:
			return POSITION_WAY_POINT_LENGTH;
	default:
		return -1;
	}
}
