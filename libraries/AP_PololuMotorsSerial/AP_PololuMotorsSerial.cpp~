/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AP_PololuMotorsSerial.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters


// init - initialise library
void AP_PololuMotorsSerial::init()
{
  //motor control channel
  Serial1.begin(MOTOR_SERIAL_RATE);
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


