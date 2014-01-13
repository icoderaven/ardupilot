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


// table of user settable parameters

// init - initialise library
void AP_Roboclaw::init()
{
  //motor control channel
  //Serial1.begin(MOTOR_SERIAL_RATE);
}


void AP_PololuMotorsSerial::shut_down(){

    hal.console->printf("Shutdown\n");
    running = false;
    set_motors(BRAKE_LOW_3,127,BRAKE_LOW_3,127);

}

