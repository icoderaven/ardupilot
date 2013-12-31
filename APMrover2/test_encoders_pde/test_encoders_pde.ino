

#include <math.h>
#include <Time.h>
#include <TimerOne.h>

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

//TO pololu motor controller  
#define SEND_CMD(x)         Serial1.write(x)

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

//Count variables
volatile long pos0 = 0;
volatile long pos1 = 0;

//distances in m
unsigned long countsPerRevolution = 0;
float wheelDiameter = 0.0f;
float wheelBase= 0.0f;

float distancePerCount = 0.0f;
float radiansPerCount = 0.0f;
long prevLeftEncoderCount = 0;
long prevRightEncoderCount = 0;
float mx = 0.0f;
float my = 0.0f;
float mth= 0.0f;


float PI_VAL = 0.0f;

// the setup routine runs once when you press reset:
void setup() {

  PI_VAL = 22.0/7.0;
  countsPerRevolution = 816;
  wheelDiameter = 12.0/100.0;
  wheelBase= 24.0/100.0;

  distancePerCount = (PI_VAL * wheelDiameter) / (float)countsPerRevolution;
  radiansPerCount = PI_VAL * (wheelDiameter / wheelBase) / countsPerRevolution;

  //debug channel
  Serial.begin(115200);

  //motor control channel
  Serial1.begin(19200);

  //timer used for pos update at 10Hz
  Timer1.initialize(100000);
  Timer1.attachInterrupt(posUpdate); 

  attachInterrupt(INT0,position0,CHANGE);
  attachInterrupt(INT1,position1,CHANGE);

}

#define NUM_WAY_POINTS 2
float wayPoint[2][2] = {{3.0,0.0},{3.0,3.0}};
int current_WP_index = 0;

int leftDirection=0;
int rightDirection=0;
float target_heading=0.0;
int flag=0;

// the loop routine runs over and over again forever:
void loop() {

  //1 move forward
  set_motors(GO_FORWARD,30,GO_FORWARD,30);
  
  while(1){
  
  print_pose();  
  //noInterrupts();
  if(compare_float(mx,2.0) >= 0){
    
    set_motors(BRAKE_LOW_3,50,BRAKE_LOW_3,50);   
    
    Serial.println("Reached 3.0 X");
    print_pose();
    
    //interrupts();
    break;
  
  }
  //interrupts();
  }
  
  delay(1000);
  
  //rotate 90
  set_motors(GO_FORWARD,30,GO_REVERSE,30);
  
  while(1){
    print_pose(); 
  //noInterrupts();
  if(compare_float(mth,1.57) >= 0){
    set_motors(BRAKE_LOW_3,40,BRAKE_LOW_3,40);
    print_pose();
    Serial.println("Turned 90 degrees");
    //interrupts();
    break;
  
  }
  //interrupts(); 
  }
  
  delay(1000);
  
  //y direction
  set_motors(GO_FORWARD,30,GO_FORWARD,30);
  
  while(1){
  print_pose(); 
  //noInterrupts();
  if(compare_float(my,1.0) >= 0){
    
    set_motors(BRAKE_LOW_3,50,BRAKE_LOW_3,50);
    print_pose();
    Serial.println("Reached 2.0 Y");
    
    //interrupts();
    break;
  
  }
  //interrupts();
  }
  
  print_pose();
  Serial.println("Done");
  while(1);
  
  
  
}

void print_pose(){

    Serial.print(mx);
    Serial.print(" ");
    Serial.print(my);
    Serial.print(" ");
    Serial.print(mth); 

    Serial.print(" ");
    Serial.print(prevLeftEncoderCount);
    Serial.print(" ");
    Serial.println(prevRightEncoderCount);


}
//returns 1 ,if a > b
//       -1 ,if a < b
//        0 , if a almost equal to b
int compare_float(float a,float b){
  
  long a_val = (long)(a * 1000.0);
  long b_val = (long)(b * 1000.0);

  /*Serial.print(a_val);
  Serial.print(" ");
  Serial.println(b_val);
  */
  
  if(abs(a_val - b_val) <= 10)
      return 0;
      
  if(a_val > b_val)
      return 1;
  else
      return -1;  

}


void position0()
{
  if(LT_PHASE_A == LT_PHASE_B)
    pos0++;//pos0--;
  else
    pos0--;//pos0++; 
}

void position1(){

  if(RT_PHASE_A == RT_PHASE_B)
    pos1--;//pos1++;
  else
    pos1++;//pos1--; 

}


void set_motors(unsigned int leftWheelDirection,unsigned int leftWheelVelocity,unsigned int rightWheelDirection,unsigned int rightWheelVelocity){

  unsigned int command_byte;
  command_byte = SET_MOTOR_BOTH | (rightWheelDirection * 4) | leftWheelDirection;
  SEND_CMD(command_byte);
  SEND_CMD(leftWheelVelocity);
  SEND_CMD(rightWheelVelocity);

}

void accelerate_motors(unsigned int leftWheelDirection,unsigned int leftWheelVelocity,unsigned int rightWheelDirection,unsigned int rightWheelVelocity){

  unsigned int command_byte;
  command_byte = ACCL_MOTOR_BOTH | (rightWheelDirection * 4) | leftWheelDirection;
  SEND_CMD(command_byte);
  SEND_CMD(leftWheelVelocity);
  SEND_CMD(rightWheelVelocity);

}

void move_forward(unsigned int wheel,unsigned int velocity){

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

void move_reverse(unsigned int wheel,unsigned int velocity){

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

void brake(unsigned int wheel,unsigned int velocity){
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

void accelerate_forward(unsigned int wheel,unsigned int velocity){

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

void posUpdate()
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



