/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AP_Common.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../AP_HAL_AVR/utility/pins_arduino_mega.h"
#include "../AP_HAL_AVR/Semaphores.h"
#include <AP_PololuMotorsSerial.h>
#include <AP_Math.h>
#include <AP_Odometry.h>
     


extern const AP_HAL::HAL& hal;
extern volatile long pos0;
extern volatile long pos1;

AP_HAL_AVR::AVRSemaphore AP_PololuMotorsSerial::_motor_sem;

// table of user settable parameters

/*try to take a semaphore safely from both in a timer and outside*/
bool AP_PololuMotorsSerial::_sem_take(uint8_t timeout)
{
    if (hal.scheduler->in_timerprocess()) {
        return _motor_sem.take_nonblocking();
    }
    return _motor_sem.take(timeout);
}

// init - initialise library
void AP_PololuMotorsSerial::init()
{
  //motor control channel
  //Serial1.begin(MOTOR_SERIAL_RATE);
}

void AP_PololuMotorsSerial::set_motors(unsigned int leftWheelDirection,unsigned int leftWheelVelocity,unsigned int rightWheelDirection,unsigned int rightWheelVelocity){

  unsigned int command_byte;
  command_byte = SET_MOTOR_BOTH | (rightWheelDirection * 4) | leftWheelDirection;
  SEND_CMD(command_byte);
  SEND_CMD(leftWheelVelocity);
  SEND_CMD(rightWheelVelocity);

}

void AP_PololuMotorsSerial::accelerate_motors(unsigned int leftWheelDirection,unsigned int leftWheelVelocity,unsigned int rightWheelDirection,unsigned int rightWheelVelocity){

  unsigned int command_byte;
  command_byte = ACCL_MOTOR_BOTH | (rightWheelDirection * 4) | leftWheelDirection;
  SEND_CMD(command_byte);
  SEND_CMD(leftWheelVelocity);
  SEND_CMD(rightWheelVelocity);

}

void AP_PololuMotorsSerial::move_forward(unsigned int wheel,unsigned int velocity){

  unsigned int command_byte;
  switch(wheel){

  case BOTH_WHEELS :
    command_byte = SET_MOTOR_BOTH | (GO_FORWARD * 4) | GO_FORWARD;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);
    SEND_CMD(velocity);
    break;

  case LEFT_WHEEL : 
    command_byte = SET_MOTOR_LEFT | GO_FORWARD;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);

    break;

  case RIGHT_WHEEL :
    command_byte = SET_MOTOR_RIGHT | GO_FORWARD;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);
    break;

  default : 
    break;  

  }

}

void AP_PololuMotorsSerial::move_reverse(unsigned int wheel,unsigned int velocity){

  unsigned int command_byte;
  switch(wheel){

  case BOTH_WHEELS :
    command_byte = SET_MOTOR_BOTH | (GO_REVERSE * 4) | GO_REVERSE;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);
    SEND_CMD(velocity);
    break;

  case LEFT_WHEEL : 
    command_byte = SET_MOTOR_LEFT | GO_REVERSE;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);

    break;

  case RIGHT_WHEEL : 
    command_byte = SET_MOTOR_RIGHT | GO_REVERSE;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);
    break;

  default : 
    break;  

  }
}

void AP_PololuMotorsSerial::brake(unsigned int wheel,unsigned int velocity){
  unsigned int command_byte;
  switch(wheel){

  case BOTH_WHEELS :
    command_byte = SET_MOTOR_BOTH | (BRAKE_LOW_3 * 4) | BRAKE_LOW_3;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);
    SEND_CMD(velocity);
    break;

  case LEFT_WHEEL : 
    command_byte = SET_MOTOR_LEFT | BRAKE_LOW_3;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);

    break;

  case RIGHT_WHEEL :
    command_byte = SET_MOTOR_RIGHT | BRAKE_LOW_3;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);
    break;

  default : 
    break;  

  }


}

void AP_PololuMotorsSerial::accelerate_forward(unsigned int wheel,unsigned int velocity){

  unsigned int command_byte;
  switch(wheel){

  case BOTH_WHEELS :
    command_byte = ACCL_MOTOR_BOTH | (GO_FORWARD * 4) | GO_FORWARD;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);
    SEND_CMD(velocity);
    break;

  case LEFT_WHEEL : 
    command_byte = ACCL_MOTOR_LEFT | GO_FORWARD;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);

    break;

  case RIGHT_WHEEL :
    command_byte = ACCL_MOTOR_RIGHT | GO_FORWARD;
    SEND_CMD(command_byte);
    SEND_CMD(velocity);
    break;

  default : 
    break;  

  }
}

void AP_PololuMotorsSerial::set_target_velocity(unsigned int leftDirection,unsigned int left,unsigned rightDirection,unsigned int right){

    //get
    if(!_sem_take(5))
        return;

    
    running = true;

    //driveLeft =0.0;
    //driveRight=0.0;
    targetLeftVelocity = left;
    targetRightVelocity = right;

    if(leftDirection == GO_REVERSE)
        targetLeftDirection = -1;
    else
        targetLeftDirection = 1;

    
    if(rightDirection == GO_REVERSE)
        targetRightDirection = -1;
    else
        targetRightDirection = 1;
        
     _motor_sem.give();
    //rel
}

void AP_PololuMotorsSerial::god_speed(){

    unsigned int driveLeftVel;
    unsigned int driveRightVel;

    long leftEncoderDiff;
    long rightEncoderDiff;
    
    float dVLeft;
    float dVRight;
    float curLeft;
    float curRight;

    unsigned long curMillis;
    unsigned long dMillis;

    unsigned int leftDir=GO_FORWARD;
    unsigned int rightDir=GO_FORWARD;
    
    if(!running)
        return;

    if(prevMillis == 0){
        prevMillis = hal.scheduler->millis();
        return;
    }    
    curMillis = hal.scheduler->millis();

    

    dMillis = curMillis - prevMillis;

    if(dMillis > 1000){
        prevMillis = hal.scheduler->millis();
        return;
    }

    noInterrupts();
    leftEncoderDiff = pos0 - prevLeftEncoderCount;
    rightEncoderDiff= pos1 - prevRightEncoderCount;
    interrupts();

    //get curr velocity
    curLeft = leftEncoderDiff / dMillis*1000.0 ; 
    curRight = rightEncoderDiff / dMillis*1000.0 ;     

   
     //get
    if(!_sem_take(5))
        return;
    
     dVLeft = kp * (targetLeftVelocity * targetLeftDirection * 7614.0/127.0  - curLeft);
     dVRight = kp * (targetRightVelocity * targetRightDirection* 7614.0/127.0 - curRight);

    if(targetLeftDirection == -1)
        leftDir=GO_REVERSE;

    if(targetRightDirection == -1)
        rightDir=GO_REVERSE;

     _motor_sem.give();
    //rel

    driveLeft +=  dVLeft;
    driveRight += dVRight;
   
    set_motors(leftDir,driveLeft,rightDir,driveRight); 
    hal.console->printf("ldiff %d rdiff %d dvleft %f dvright %f dt %d\n",leftEncoderDiff,rightEncoderDiff,dVLeft,dVRight,dMillis);

    prevMillis = curMillis;
    
}

void AP_PololuMotorsSerial::shut_down(){

    hal.console->printf("Shutdown\n");
    running = false;
    set_motors(BRAKE_LOW_3,127,BRAKE_LOW_3,127);

}

