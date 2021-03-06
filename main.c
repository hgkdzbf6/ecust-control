/*

AscTec AutoPilot HL SDK v2.0

Copyright (c) 2011, Ascending Technologies GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

 */

/**********************************************************
                  Header files
 **********************************************************/
#include "LPC214x.h"
#include "main.h"
#include "system.h"
#include "uart.h"
#include "mymath.h"
#include "hardware.h"
#include "irq.h"
#include "i2c.h"
#include "gpsmath.h"
#include "adc.h"
#include "uart.h"
#include "ssp.h"
#include "LL_HL_comm.h"
#include "sdk.h"
#include "buzzer.h"
#include "ublox.h"
#include "pelican_ptu.h"
#include "declination.h"
#include "MyProtocol.h"
#include "pid.h"
#include "common_header.h"

/* *********************************************************
               Function declarations
  ********************************************************* */

void Initialize(void);
void feed(void);
void beeper(unsigned char);

/**********************************************************
                  Global Variables
 **********************************************************/
struct HL_STATUS HL_Status;
struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;
struct GPS_TIME GPS_Time;

volatile unsigned int int_cnt=0, cnt=0, mainloop_cnt=0;
volatile unsigned char mainloop_trigger=0;
volatile unsigned int GPS_timeout=0;
volatile unsigned int trigger_cnt=0;
volatile char SYSTEM_initialized=0;

unsigned int uart_cnt;
unsigned char DataOutputsPerSecond=10;

extern MyViconData receivedViconData;
extern DebugData sendDebugData;
extern DebugData receiveDebugData;
extern ParamDebug sendParamDebug;
extern PositionWayPointData sendPositionWayPointData;
extern PositionWayPointData receivePositionWayPointData;
extern LandSignal sendLandSignal;
extern LandSignal receiveLandSignal;
extern CmdData receiveCmdData;
extern state_t my_setpoint;
extern state_t my_state;
extern int cpu_load;
extern int vicon_count;
extern float calc_thrust;
extern struct this_s my_this ;
extern int output_thrust;
extern int receive_valid_data_flag;
extern int vicon_tp;
extern float calc_pitch;
extern float calc_roll;
extern NormalData sendNormalData;
extern DebugArray sendDebugArray;
extern NormalData receiveNormalData;
extern int use_way_point_flag;
extern float change_vicon_x;
void timer0ISR(void) __irq
{
  T0IR = 0x01;      //Clear the timer 0 interrupt
  IENABLE;
  trigger_cnt++;
  if(trigger_cnt==ControllerCyclesPerSecond)
  {
  	trigger_cnt=0;
  	HL_Status.up_time++;
  	HL_Status.cpu_load=mainloop_cnt;

  	mainloop_cnt=0;
  }

  if(mainloop_trigger<10) mainloop_trigger++;

  IDISABLE;
  VICVectAddr = 0;		// Acknowledge Interrupt
}

/**********************************************************
                       MAIN
**********************************************************/
int	main (void) {

  static int vbat1; //battery_voltage (lowpass-filtered)

  init();
  buzzer(OFF);
  LL_write_init();
  PTU_init();
  ADC0triggerSampling(1<<VOLTAGE_1); //activate ADC sampling

  HL_Status.up_time=0;

  LED(1,ON);

//  read_callback=my_read_callback;
//  write_callback=my_write_callback;

  while(1)
  {
      if(mainloop_trigger)
      {
     	if(GPS_timeout<ControllerCyclesPerSecond) GPS_timeout++;
	  	else if(GPS_timeout==ControllerCyclesPerSecond)
	  	{
  	 		GPS_timeout=ControllerCyclesPerSecond+1;
	  		GPS_Data.status=0;
	  		GPS_Data.numSV=0;
	  	}

        //battery monitoring
        ADC0getSamplingResults(0xFF,adcChannelValues);
        vbat1=(vbat1*14+(adcChannelValues[VOLTAGE_1]*9872/579))/15;	//voltage in mV

		HL_Status.battery_voltage_1=vbat1;
        mainloop_cnt++;
		if(!(mainloop_cnt%10)) buzzer_handler(HL_Status.battery_voltage_1);

	    if(mainloop_trigger) mainloop_trigger--;
        mainloop();
      }
  }
  return 0;
}


void mainloop(void) //mainloop is triggered at 1 kHz
{
    static unsigned char led_cnt=0, led_state=1;
	unsigned char t;
	static int mainloop_test=0;
	mainloop_test++;
	//blink red led if no GPS lock available
	led_cnt++;
	if((GPS_Data.status&0xFF)==0x03)
	{
		LED(0,OFF);
	}
	else
	{
	    if(led_cnt==150)
	    {
	      LED(0,ON);
	    }
	    else if(led_cnt==200)
	    {
	      led_cnt=0;
	      LED(0,OFF);
	    }
	}

	//after first lock, determine magnetic inclination and declination
	if (SYSTEM_initialized)
	{
		if ((!declinationAvailable) && (GPS_Data.horizontal_accuracy<10000) && ((GPS_Data.status&0x03)==0x03)) //make sure GPS lock is valid
		{
			int status;
			estimatedDeclination=getDeclination(GPS_Data.latitude,GPS_Data.longitude,GPS_Data.height/1000,2012,&status);
			if (estimatedDeclination<-32000) estimatedDeclination=-32000;
			if (estimatedDeclination>32000) estimatedDeclination=32000;
			declinationAvailable=1;
		}
	}

	//toggle green LED and update SDK input struct when GPS data packet is received
    if (gpsLEDTrigger)
    {
		if(led_state)
		{
			led_state=0;
			LED(1,OFF);
		}
		else
		{
			LED(1,ON);
			led_state=1;
		}

		RO_ALL_Data.GPS_height=GPS_Data.height;
		RO_ALL_Data.GPS_latitude=GPS_Data.latitude;
		RO_ALL_Data.GPS_longitude=GPS_Data.longitude;
		RO_ALL_Data.GPS_speed_x=GPS_Data.speed_x;
		RO_ALL_Data.GPS_speed_y=GPS_Data.speed_y;
		RO_ALL_Data.GPS_status=GPS_Data.status;
		RO_ALL_Data.GPS_sat_num=GPS_Data.numSV;
		RO_ALL_Data.GPS_week=GPS_Time.week;
		RO_ALL_Data.GPS_time_of_week=GPS_Time.time_of_week;
		RO_ALL_Data.GPS_heading=GPS_Data.heading;
		RO_ALL_Data.GPS_position_accuracy=GPS_Data.horizontal_accuracy;
		RO_ALL_Data.GPS_speed_accuracy=GPS_Data.speed_accuracy;
		RO_ALL_Data.GPS_height_accuracy=GPS_Data.vertical_accuracy;

		gpsLEDTrigger=0;
    }

//	//re-trigger UART-transmission if it was paused by modem CTS pin
//	if(trigger_transmission)
//	{
//		if(!(IOPIN0&(1<<CTS_RADIO)))
//	  	{
//	  		trigger_transmission=0;
//	    if(ringbuffer(RBREAD, &t, 1))
//		    {
//		      transmission_running=1;
//		      UARTWriteChar(t);
//		    }
//	  	}
//	}
//
//	//send attitude data packet as an example how to use HL_serial_0 (please refer to uart.c for details)

//
//
	if(uart_cnt++==ControllerCyclesPerSecond/DataOutputsPerSecond)
	{
		uart_cnt=0;
		if(receiveCmdData.cmd==PACKAGE_DEFINE_DEBUG){
			sendDebugData.timestamp=vicon_tp;
			sendDebugData.x=my_state.position.x;
			sendDebugData.y=my_state.position.y;
			sendDebugData.z=my_state.position.z;
			sendDebugData.vx=receiveDebugData.vx;
			sendDebugData.vy=receiveDebugData.vy;
			sendDebugData.vz=my_setpoint.velocity.y;
//			sendDebugData.pitch=my_state.attitude.pitch;
//			sendDebugData.roll=my_state.attitude.roll;
//			sendDebugData.yaw=my_state.attitude.yaw;
			sendDebugData.pitch=RO_ALL_Data.angle_pitch;
			sendDebugData.roll=RO_ALL_Data.angle_roll;
			sendDebugData.yaw=RO_ALL_Data.angle_yaw;
			sendDebugData.battery=RO_ALL_Data.fusion_height;
			sendDebugData.cpu_load=RO_ALL_Data.fusion_dheight;
			sendDebugData.set_position=my_this.pidVY.setpoint;
			sendDebugData.set_velocity=WO_CTRL_Input.roll;
			sendDebugData.vicon_count=vicon_count;
			sendDebugData.calc_thrust=calc_thrust;
			my_send(1,PACKAGE_DEFINE_DEBUG,
					getPackageLength(PACKAGE_DEFINE_DEBUG),
					&sendDebugData,1);
		}else if(receiveCmdData.cmd==PACKAGE_DEFINE_PARAM){
			sendParamDebug.calc_thrust=calc_thrust;
			sendParamDebug.ki_p=my_this.pidZ.pid.ki;
			sendParamDebug.kp_p=my_this.pidZ.pid.kp;
			sendParamDebug.ki_v=my_this.pidVZ.pid.ki;
			sendParamDebug.kp_v=my_this.pidVZ.pid.kp;
			sendParamDebug.set_velocity=my_setpoint.velocity.z;
			sendParamDebug.vz=my_state.velocity.z;
			sendParamDebug.z=my_state.position.z;
			sendParamDebug.thrust=output_thrust;
			my_send(1,PACKAGE_DEFINE_PARAM,
					getPackageLength(PACKAGE_DEFINE_PARAM),
					&sendParamDebug,1);
			receiveCmdData.cmd=PACKAGE_DEFINE_NOMAL_DATA;
		}else if(receiveCmdData.cmd==PACKAGE_DEFINE_POSITION_WAY_POINT){
			sendPositionWayPointData.x=my_setpoint.position.x;
			sendPositionWayPointData.y=my_setpoint.position.y;
			sendPositionWayPointData.z=my_setpoint.position.z;
			my_send(1,PACKAGE_DEFINE_POSITION_WAY_POINT,
					getPackageLength(PACKAGE_DEFINE_POSITION_WAY_POINT),
					&sendPositionWayPointData,1);
			receiveCmdData.cmd=PACKAGE_DEFINE_NOMAL_DATA;
		}else if(receiveCmdData.cmd==PACKAGE_DEFINE_LAND){
			sendLandSignal.mode=receiveLandSignal.mode;
			my_send(1,PACKAGE_DEFINE_LAND,
					getPackageLength(PACKAGE_DEFINE_LAND),
					&sendLandSignal,1);
			receiveCmdData.cmd=PACKAGE_DEFINE_NOMAL_DATA;
		}else if(receiveCmdData.cmd==PACKAGE_DEFINE_NOMAL_DATA){
			sendNormalData.timestamp=vicon_tp;
			sendNormalData.x=my_state.position.x;
			sendNormalData.y=my_state.position.y;
			sendNormalData.z=my_state.position.z;
			sendNormalData.vx=my_state.velocity.x;
			sendNormalData.vy=my_state.velocity.y;
			sendNormalData.vz=my_state.velocity.z;
			sendNormalData.yaw=my_state.attitude.yaw;

			sendNormalData.sp_x=my_setpoint.position.x;
			sendNormalData.sp_y=my_setpoint.position.y;
			sendNormalData.sp_z=my_setpoint.position.z;
			sendNormalData.sp_flag=use_way_point_flag;

			sendNormalData.debug_1=mainloop_test;
			sendNormalData.debug_2=my_setpoint.velocity.x;
			sendNormalData.debug_3=my_setpoint.velocity.y;
			sendNormalData.debug_4=change_vicon_x;//change_vicon_x;
			my_send(1,PACKAGE_DEFINE_NOMAL_DATA,
					getPackageLength(PACKAGE_DEFINE_NOMAL_DATA),
					&sendNormalData,1);
			mainloop_test=0;
		}else if (receiveCmdData.cmd==PACKAGE_DEFINE_DEBUG_ARRAY){
			my_send(1,PACKAGE_DEFINE_DEBUG_ARRAY,
					getPackageLength(PACKAGE_DEFINE_DEBUG_ARRAY),
					&sendDebugArray,1);
		}
	}
	//handle gps data reception
    uBloxReceiveEngine();

	//run SDK mainloop. Please put all your data handling / controller code in sdk.c
	SDK_mainloop();

    //write data to transmit buffer for immediate transfer to LL processor
    HL2LL_write_cycle();

    //control pan-tilt-unit ("cam option 4" @ AscTec Pelican)
    PTU_update();


}


