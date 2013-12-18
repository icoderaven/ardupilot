

#include <math.h>
#include <Time.h>

/*
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */

 
//Interrupts used 
#define INT0 0
#define INT1 1

//Refresh period
#define DELAY 500
 
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

/*
//x,y,theta calculations
float ticdistance = .00051589879;
long lastLeft = 0;
long lastRight = 0;
float x = 0;
float y = 0;
float th= 0;
float vr=0;
float vl=0;
float V=0;
float w=0;
time_t lastTime=0;
time_t dt = 0;
float wheelBase = .24;
long currentPos0 = 0;
long currentPos1 = 0;
*/

//distances in m
#define distancePerCount  0.00046f;
#define radiansPerCount  0.0019f;//0.007699;
unsigned long prevLeftEncoderCount = 0;
unsigned long prevRightEncoderCount = 0;
float mx = 0.0f;
float my = 0.0f;
float mth= 0.0f;
float wheelBase= 0.24f;

#define PI_VAL 3.14159

// the setup routine runs once when you press reset:
void setup() {

  //required ??
  /*pinMode(encLtA, INPUT); 
  digitalWrite(encLtA, HIGH);       // turn on pullup resistor
  pinMode(encLtB, INPUT); 
  digitalWrite(encLtB, HIGH);       // turn on pullup resistor
  
  pinMode(encRtA, INPUT); 
  digitalWrite(encRtA, HIGH);       // turn on pullup resistor
  pinMode(encRtB, INPUT); 
  digitalWrite(encRtB, HIGH);       // turn on pullup resistor
  */
  //debug channel
  Serial.begin(115200);
  
  //motor control channel
  Serial1.begin(19200);
  
  attachInterrupt(INT0,position0,CHANGE);
  attachInterrupt(INT1,position1,CHANGE);
  
  //lastTime = now();  
}

unsigned int step_num = 100 ;
unsigned int step_count = 0;

unsigned int count=0;
int motion = 0;

// the loop routine runs over and over again forever:
void loop() {
  
  //delay(DELAY);
/*
  if(count % 100 == 0){
  noInterrupts();
  currentPos1 = pos1;
  currentPos0 = pos0;
  interrupts();

  dt = now() - lastTime;
  vl = (currentPos1-lastLeft)*ticdistance*2;
  lastLeft = currentPos1;
  vr = (currentPos0-lastRight)*ticdistance*2;
  lastRight = currentPos0;
  //Serial.print(vr);
  //Serial.print(" ");
  //Serial.print(vl);
  //Serial.print(" ");
  V = (vr+vl)/2;
  w = (vr-vl)/wheelBase;
  //Serial.print(V);
  //Serial.print(" ");
  //Serial.println(w);
  th+=w*dt/2;
  x +=V*cos(th)*dt;
  y +=V*sin(th)*dt;
  th+=w*dt/2;
  lastTime+=dt;
  }
 */ 
 
 unsigned long leftEncoderCount=0;
 unsigned long rightEncoderCount=0;
 unsigned long dLEC=0;
 unsigned long dREC=0;
 float dDistance=0.0f;
 float dX=0.0f;
 float dY=0.0f;
 float dth=0.0f;
 
 if(count % 1000 == 0){
  noInterrupts();
  leftEncoderCount = pos1;
  rightEncoderCount = pos0;
  interrupts();
  
  dLEC = leftEncoderCount - prevLeftEncoderCount;
  dREC = rightEncoderCount - prevRightEncoderCount;
  
  dDistance = 0.5f * (dLEC + dREC) * distancePerCount;

  dX = dDistance * cos(mth);
  dY = dDistance * sin(mth);
  dth = 1.0f * (dLEC - dREC) * distancePerCount;
  dth = dth/wheelBase;
  
  
  //sad..tch tch
  if(dth > PI_VAL || dth < -PI_VAL){
    
    dth = 1.0f * (dREC - dLEC) * distancePerCount;
    dth = dth/wheelBase;
    
    if(dth > PI_VAL || dth < -PI_VAL){
    Serial.println(dth);
    dth = 0;
    }
  
  }
  mx += dX;
  my += dY;
  mth += dth;
  
  prevLeftEncoderCount = leftEncoderCount;
  prevRightEncoderCount = rightEncoderCount;
  
  if(mth > PI_VAL)
    mth = mth - (2 * PI_VAL);
  else if(mth <= -PI_VAL)
    mth = mth + (2 * PI_VAL);  
  
  
  if(count % 1000 == 0){
  Serial.print(mx);
  Serial.print(" ");
  Serial.print(my);
  Serial.print(" ");
  Serial.print(mth); 
  Serial.print(" ");
  Serial.print(leftEncoderCount);
  Serial.print(" ");
  Serial.println(rightEncoderCount);
  }
  
  }
  
  //basic test
  /*
  delay(3000);
  
  Serial1.write(0xDA);
  //Serial1.write(32);
  Serial1.write(0x50);
  Serial1.write(0x50);
  
  delay(3000);
  
  
  Serial1.write(0xDA);
  //Serial1.write(32);
  Serial1.write(0x0);
  Serial1.write(0x0);
  */
  
  /*
  if(count % 1000 == 0){
  
    Serial.print(vr);
  Serial.print(" ");
  Serial.print(vl);
  Serial.print(" ");
  
  Serial.print(V);
  Serial.print(" ");
  Serial.println(w);
  
  Serial.println("Pose: ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(th); 

  Serial.print(currentPos0);
  Serial.print(" ");
  Serial.println(currentPos1);
  
  }*/
  
    count = millis();
    
    if(count % 2000 == 0){
  //   Serial.println("B");
      move_reverse(BOTH_WHEELS,0);    
    }
    else if(count % 1000 == 0){     
    //  Serial.println("A");
      move_reverse(BOTH_WHEELS,80);
    }
 
  
  
  
}

void position0()
{
  if(LT_PHASE_A == LT_PHASE_B)
  pos0--;
  else
  pos0++; 
}

void position1(){
  
  if(RT_PHASE_A == RT_PHASE_B)
  pos1++;
  else
  pos1--; 

}

void move_forward(unsigned int wheel,unsigned int velocity){

  unsigned int command_byte;
  switch(wheel){
    
    case BOTH_WHEELS :command_byte = SET_MOTOR_BOTH | (GO_FORWARD * 4) | GO_FORWARD;
                      SEND_CMD(command_byte);
                      SEND_CMD(velocity);
                      SEND_CMD(velocity);
                      break;
    
    case LEFT_WHEEL : command_byte = SET_MOTOR_LEFT | GO_FORWARD;
                      SEND_CMD(command_byte);
                      SEND_CMD(velocity);
      
                      break;
      
    case RIGHT_WHEEL :command_byte = SET_MOTOR_RIGHT | GO_FORWARD;
                      SEND_CMD(command_byte);
                      SEND_CMD(velocity);
                        break;
  
    default : break;  
  
  }
  
}

void move_reverse(unsigned int wheel,unsigned int velocity){

  unsigned int command_byte;
  switch(wheel){
    
    case BOTH_WHEELS :command_byte = SET_MOTOR_BOTH | (GO_REVERSE * 4) | GO_REVERSE;
                      SEND_CMD(command_byte);
                      SEND_CMD(velocity);
                      SEND_CMD(velocity);
                      break;
    
    case LEFT_WHEEL : command_byte = SET_MOTOR_LEFT | GO_REVERSE;
                      SEND_CMD(command_byte);
                      SEND_CMD(velocity);
    
                      break;
      
    case RIGHT_WHEEL : command_byte = SET_MOTOR_RIGHT | GO_REVERSE;
                      SEND_CMD(command_byte);
                      SEND_CMD(velocity);
                        break;
  
    default : break;  
  
  }
}

void brake(unsigned int wheel,unsigned int velocity){

 //TODO
}

