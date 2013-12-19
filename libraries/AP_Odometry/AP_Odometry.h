/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ODOMETRY_H__
#define __AP_ODOMETRY_H__

//Interrupts used 
#define INT0 0
#define INT1 1

//Pins used
//Encoder Left
#define encLtA  2
#define encLtB  A0
//Encoder Right
#define encRtA  3
#define encRtB  A1

//Read macros
#define LT_PHASE_A digitalRead(encLtA)
#define LT_PHASE_B digitalRead(encLtB)
#define RT_PHASE_A digitalRead(encRtA)
#define RT_PHASE_B digitalRead(encRtB)


class AP_Odometry
{
public:

    // Constructor, what is AHRS ??
    AP_Odometry(AP_AHRS* ahrs):
        _ahrs(ahrs),
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    // Initialisation
    void        init();

    // update odometer position estimate
    void        update_pos();

    // get_position - returns current x,y coordinate in m and theta in radians
    float  get_position_x() const { return mx; }
    float  get_position_y() const { return my; }
    float  get_position_th() const { return mth; }	

    // encoder position counter callbacks
    void position0();
    void position1();		

    // class level parameters , check ??
    static const struct AP_Param::GroupInfo var_info[];

protected:


    AP_AHRS*                _ahrs;                      // pointer to ahrs object
    //Count variables
    volatile long pos0; //left motor
    volatile long pos1; //right motor

    //distances in m
    unsigned long countsPerRevolution;
    float wheelDiameter;
    float wheelBase;

    float distancePerCount;
    float radiansPerCount;
    long prevLeftEncoderCount;
    long prevRightEncoderCount;
    float mx;
    float my;
    float mth;
    float PI_VAL;


};

#endif // __AP_INERTIALNAV_H__
