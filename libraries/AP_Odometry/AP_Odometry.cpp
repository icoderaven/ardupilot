/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AP_Odometry.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_InertialNav::var_info[] PROGMEM = {
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
};

// init - initialise library
void AP_Odometry::init()
{
	PI_VAL = 22.0/7.0;
	countsPerRevolution = 816;
	wheelDiameter = 12.0/100.0;
	wheelBase= 24.0/100.0;

	distancePerCount = (PI_VAL * wheelDiameter) / (float)countsPerRevolution;
	radiansPerCount = PI_VAL * (wheelDiameter / wheelBase) / countsPerRevolution;

	attachInterrupt(INT0,position0,CHANGE);
	attachInterrupt(INT1,position1,CHANGE);
}

//update odometry position estimate
void AP_Odometry::update_pos()
{
	long leftEncoderCount=0;
	long rightEncoderCount=0;
	long dLEC=0;
	long dREC=0;
	float dDistance=0.0f;
	float dX=0.0f;
	float dY=0.0f;
	float dth=0.0f;

	leftEncoderCount = pos0;
	rightEncoderCount = pos1;

	dLEC = leftEncoderCount - prevLeftEncoderCount;
	dREC = rightEncoderCount - prevRightEncoderCount;

	dDistance = 0.5f * (dLEC + dREC) * distancePerCount;

	dX = dDistance * (float) cos(mth);
	dY = dDistance * (float) sin(mth);
	dth = (float)(dLEC - dREC) * radiansPerCount;

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

void AP_Odometry::position0()
{
	if(LT_PHASE_A == LT_PHASE_B)
		pos0++;
	else
		pos0--; 
}

void AP_Odometry::position1(){

	if(RT_PHASE_A == RT_PHASE_B)
		pos1--;
	else
		pos1++;

}

