/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//#include <AP_HAL.h>
#include <AP_Common.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../AP_HAL_AVR/utility/pins_arduino_mega.h"

#include <AP_Roboclaw.h>
#include <AP_Math.h>
     
extern const AP_HAL::HAL& hal;

bool AP_Roboclaw::write_n(uint8_t cnt, ... )
{
	uint8_t crc=0;
	
	//send data with crc
	va_list marker;
	va_start( marker, cnt );     /* Initialize variable arguments. */
	for(uint8_t index=0;index<cnt;index++){
		uint8_t data = va_arg(marker, uint16_t);
		crc+=data;
		SEND_CMD(data);
	}
	va_end( marker );              /* Reset variable arguments.      */

	SEND_CMD(crc&0x7F);
	
	return true;
}

// init - initialise library
void AP_Roboclaw::init()
{
  //motor control channel
  //Serial1.begin(MOTOR_SERIAL_RATE);

  //Reset Encoders
  ResetEncoders(ADDRESS);  

  //set Kp Ki Kd vals
  SetM1VelocityPID(ADDRESS, Kd, Kp, Ki, qpps);
  SetM2VelocityPID( ADDRESS, Kd, Kp, Ki, qpps);  

}

bool AP_Roboclaw::ResetEncoders(uint8_t address){
	return write_n(2,ADDRESS,RESETENC);
}

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(arg>>8),(uint8_t)arg

bool AP_Roboclaw::SetM1VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, uint32_t qpps_){
	uint32_t kd = kd_fp*65536;
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	return write_n(18,ADDRESS,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps_));
}

bool AP_Roboclaw::SetM2VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, uint32_t qpps_){
	uint32_t kd = kd_fp*65536;
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	return write_n(18,ADDRESS,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps_));
}

//For roboclaw speed is in units of encoder quadrature counts per second
// Max value defined is 17000, but DO NOT USE 17000 as it may damage the motors
// For running, use RANGE as +/- 12650 
bool AP_Roboclaw::SpeedM1(uint8_t address, uint32_t speed){
	return write_n(6,ADDRESS,M1SPEED,SetDWORDval(speed));
}

bool AP_Roboclaw::SpeedM2(uint8_t address, uint32_t speed){
	return write_n(6,ADDRESS,M2SPEED,SetDWORDval(speed));
}

bool AP_Roboclaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2){
	return write_n(10,ADDRESS,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}


void AP_Roboclaw::shut_down(){

    SpeedM1(ADDRESS,0);
    SpeedM2(ADDRESS,0);

}

