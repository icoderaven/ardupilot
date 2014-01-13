/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../AP_HAL_AVR/utility/pins_arduino_mega.h"
#include <AP_Math.h>
#include <AP_Odometry.h>

extern const AP_HAL::HAL& hal;
extern const AP_AHRS_DCM ahrs;

extern volatile long pos0;
extern volatile long pos1;

static void position0();
static void position1();
/* table of user settable parameters
const AP_Param::GroupInfo AP_Odometry::var_info[] PROGMEM = {
	// start numbering at 1 because 0 was previous used for body frame accel offsets
	// @Param: TC_XY
	// @DisplayName: Horizontal Time Constant
	// @Description: Time constant for GPS and accel mixing. Higher TC decreases GPS impact on position estimate
	// @Range: 0 10
	// @Increment: 0.1
	AP_GROUPINFO("TC_XY",   1, AP_InertialNav, _time_constant_xy, AP_INTERTIALNAV_TC_XY),

	// @Param: TC_Z
	// @DisplayName: Vertical Time Constant
	// @Description: Time constant for baro and accel mixing. Higher TC decreases barometers impact on altitude estimate
	// @Range: 0 10
	// @Increment: 0.1
	AP_GROUPINFO("TC_Z",    2, AP_InertialNav, _time_constant_z, AP_INTERTIALNAV_TC_Z),

	AP_GROUPEND
};*/

// init - initialise library
void AP_Odometry::init()
{
    pos0=0;
    pos1=0;
    prevLeftEncoderCount=0;
    prevRightEncoderCount=0;
    mx=0.0;
    my=0.0;
    mth=0.0;

    #ifdef USE_AHRS_YAW_FOR_ODOM
    prevYaw = ahrs.yaw;
    #endif

    pose = Vector3f(0,0,0);
    
	PI_VAL = 3.14159;
	countsPerRevolution = 816;
	wheelDiameter = 0.1242;
	wheelBase= 0.235;

	distancePerCount = (PI_VAL * wheelDiameter) / (float)countsPerRevolution;
	radiansPerCount = PI_VAL * (wheelDiameter / wheelBase) / countsPerRevolution;

    hal.gpio->pinMode(encLtA,GPIO_INPUT);
    hal.gpio->pinMode(encRtA,GPIO_INPUT);

	hal.gpio->attach_interrupt(INT0,position0,CHANGE);
	hal.gpio->attach_interrupt(INT1,position1,CHANGE);
}

//update odometry position estimate
void AP_Odometry::update_pose()
{
	long leftEncoderCount=0;
	long rightEncoderCount=0;
	long dLEC=0;
	long dREC=0;
	float dDistance=0.0f;
	float dX=0.0f;
	float dY=0.0f;
	float dth=0.0f;

    noInterrupts();    
	leftEncoderCount = pos0;
	rightEncoderCount = pos1;
    interrupts();

	dLEC = leftEncoderCount - prevLeftEncoderCount;
	dREC = rightEncoderCount - prevRightEncoderCount;

	dDistance = 0.5f * (dLEC + dREC) * distancePerCount;

	dX = dDistance * (float) cos(mth);
	dY = dDistance * (float) sin(mth);

    #ifdef USE_AHRS_YAW_FOR_ODOM
    dth = -(ahrs.yaw - prevYaw);
    prevYaw = ahrs.yaw;
    #else
	dth = (float)(dREC - dLEC) * radiansPerCount;
    #endif
 

	//update measured x,y,theta
	mx += dX;
	my += dY;
	mth += dth;

	prevLeftEncoderCount = leftEncoderCount;
	prevRightEncoderCount = rightEncoderCount;

	if(mth > PI_VAL)
		mth = mth - (2 * PI_VAL);
	else if(mth <= -PI_VAL)
		mth = mth + (2 * PI_VAL);  

}

Vector3f AP_Odometry::get_pose()
 {    
     noInterrupts();
     pose.x = mx;
     pose.y = my;
     pose.z = mth;
     interrupts();
     return pose; }

static void position0()
{
	if(LT_PHASE_A == LT_PHASE_B)
		pos0++;
	else
		pos0--; 
}

static void position1(){

	if(RT_PHASE_A == RT_PHASE_B)
		pos1--;
	else
		pos1++;

}

