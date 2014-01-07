/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_PololuMotorsSerial_H__
#define __AP_PololuMotorsSerial_H__

//TO pololu motor controller using UART1 reserved for GPS on ardupilot board
#define SEND_CMD(x)         hal.uartB->print((char)x)
#define MOTOR_SERIAL_RATE   19200
//wheel
#define LEFT_WHEEL   0
#define RIGHT_WHEEL  1
#define BOTH_WHEELS  2

//RPM
#define MAX_SPEED_RPM 285

//Pololu motor driver commands, assume motor1 = left, motor2 = right
#define GET_SIGNATURE       0x81
#define GET_MODE            0x82
#define CHECK_SERIAL        0X83
#define SET_MOTOR_LEFT      0xC0
#define SET_MOTOR_RIGHT     0xC8
#define ACCL_MOTOR_LEFT     0xC4
#define ACCL_MOTOR_RIGHT    0xCC

#define SET_MOTOR_BOTH      0xD0
#define ACCL_MOTOR_BOTH     0xE0

//Direction Bits
#define BRAKE_LOW_0         0x00
#define GO_REVERSE          0x01
#define GO_FORWARD          0x02
#define BRAKE_LOW_3         0x03

class AP_PololuMotorsSerial
{
public:

    // Constructor, what is AHRS ??
    AP_PololuMotorsSerial()
        
        {
            //AP_Param::setup_object_defaults(this, var_info);
        }

    // Initialisation
    void        init();
    void set_motors(unsigned int leftWheelDirection,unsigned int leftWheelVelocity,unsigned int rightWheelDirection,unsigned int     rightWheelVelocity);
    void accelerate_motors(unsigned int leftWheelDirection,unsigned int leftWheelVelocity,unsigned int rightWheelDirection,unsigned int rightWheelVelocity);
    void move_forward(unsigned int wheel,unsigned int velocity);
    void move_reverse(unsigned int wheel,unsigned int velocity);
    void brake(unsigned int wheel,unsigned int velocity);
    void accelerate_forward(unsigned int wheel,unsigned int velocity);

    void god_speed();
    void set_target_velocity(unsigned int leftWheelDirection,unsigned int leftWheelVelocity,unsigned int rightWheelDirection,unsigned int     rightWheelVelocity);
    bool _sem_take(uint8_t timeout);

private :
    long prevLeftEncoderCount=0;
    long prevRightEncoderCount=0;
    unsigned long prevMillis=0;
    long targetLeftVelocity=0;
    long targetRightVelocity=0;
    int targetLeftDirection=1;
    int targetRightDirection=1;

    //last velocity sent to motors
    unsigned int driveLeft=0;
    unsigned int driveRight=0;

    static AP_HAL_AVR::AVRSemaphore _motor_sem;
    
    float kp=0.5;


};

#endif // __AP_PololuMotorsSerial_H__
