//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

//Includes required to use Roboclaw library
#include <BMSerial.h>
#include <RoboClaw.h>
#include <math.h>
#include <Time.h>
#include <TimerOne.h>

//interrupt constants and Odometry
/*************************************************************/
/********** Odometry and rover measurements ******************/
/*************************************************************/

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

//Count variables
volatile long pos0 = 0;
volatile long pos1 = 0;

/**********************************************/
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


/*************************************************************/
/********* Roboclaw info *************************************/
/*************************************************************/
#define address 0x80

//Velocity PID coefficients
#define Kp 1.1//0.00390625//1.0
#define Ki 0.5//0.001953125//0.5
#define Kd 0.25//0.0009765625//0.25
#define qpps 17000 

//Definte terminal for display. Use hardware serial pins 0 and 1
BMSerial terminal(0,1);

//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw(19,18,10000);

void setup() {
  //Open terminal and roboclaw at 38400bps
  terminal.begin(9600);
  roboclaw.begin(38400);

  terminal.println("Starting...");

  roboclaw.ResetEncoders(address);

  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps); 
  
  
  /*****************************************************/
  
  PI_VAL = 3.14159;
  countsPerRevolution = 816;
  wheelDiameter = 0.1221;
  wheelBase= 0.24;

  distancePerCount = (PI_VAL * wheelDiameter) / (float)countsPerRevolution;
  radiansPerCount = PI_VAL * (wheelDiameter / wheelBase) / countsPerRevolution;

  //timer used for pos update at 10Hz
  Timer1.initialize(100000);
  Timer1.attachInterrupt(posUpdate); 

  pinMode(encLtA, INPUT);           // set pin to input
  pinMode(encLtB, INPUT);
  pinMode(encRtA, INPUT);
  pinMode(encRtB, INPUT);            // set pin to input
  digitalWrite(encLtA, HIGH);       // turn on pullup resistors
  digitalWrite(encRtA, HIGH);       // turn on pullup resistors 

  attachInterrupt(INT0,position0,CHANGE);
  attachInterrupt(INT1,position1,CHANGE);

  //roboclaw.ForwardM1(address,32);
  //roboclaw.ForwardM2(address,32);
}


void displayspeed() {
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;

  //Read all the data from Roboclaw before displaying on terminal window
  //This prevents the hardware serial interrupt from interfering with
  //reading data using software serial.
  uint32_t enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  uint32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  uint32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  uint32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  uint32_t diffEnc=0;
  uint32_t diffSpeed=0;

  if(valid1){
    terminal.print("Encoder1:");
    terminal.print(enc1,DEC);
    terminal.print(" ");
    terminal.print(status1,HEX);
    terminal.print(" ");
  }
  if(valid2){
    terminal.print("Encoder2:");
    terminal.print(enc2,DEC);
    terminal.print(" ");
    terminal.print(status2,HEX);
    terminal.print(" [");

    if(enc1 > enc2)
      diffEnc = enc1 - enc2;
    else
      diffEnc = enc2 - enc1;

    terminal.print(diffEnc,DEC);
    terminal.print("] ");
  }
  if(valid3){
    terminal.print("Speed1:");
    terminal.print(speed1,DEC);
    terminal.print(" ");
  }
  if(valid4){
    terminal.print("Speed2:");
    terminal.print(speed2,DEC);
    terminal.print(" [");

    if(speed1 > speed2)
      diffSpeed = speed1 - speed2;
    else
      diffSpeed = speed2 - speed1;

    terminal.print(diffSpeed,DEC);
    terminal.print("] ");
    terminal.println();
  }


}

unsigned int count =0; //count is number of seconds to drive

void loop() {

  if(count < 16) {  

    roboclaw.SpeedM1(address,3550);
    roboclaw.SpeedM2(address,3550);

  }
  else{

    roboclaw.SpeedM1(address,0);
    roboclaw.SpeedM2(address,0);    
  }

  delay(1000);

  /*for(uint8_t i = 0;i<10;i++){
    displayspeed();
    delay(10);
  }*/

  print_pose();

  count++;

}


void position0()
{
  //stateLeftA = !stateLeftA;

  if(LT_PHASE_A == LT_PHASE_B)
    pos0++;//pos0--;
  else
    pos0--;//pos0++; 
}

void position1(){

  //stateRightA = !stateRightA;

  if(RT_PHASE_A == RT_PHASE_B)
    pos1--;//pos1++;
  else
    pos1++;//pos1--; 

}

void print_pose(){

  float x,y,th;
  long left,right;
   
  x=mx;
  y=my;
  th=mth;
  left = pos0;
  right = pos1;
 
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(th); 

  Serial.print(" ");
  Serial.print(left);
  Serial.print(" ");
  Serial.print(right);
  Serial.print(" ");
  Serial.println(left - right);


  /*
  long leftCount=0;
   long rightCount=0;
   long diff=0;
   
   noInterrupts();
   leftCount = pos0;
   rightCount = pos1;
   interrupts();
   
   if(leftCount > rightCount)
   diff = leftCount - rightCount;
   else
   diff = rightCount - leftCount;
   
   Serial.print(leftCount);
   Serial.print(" ");
   Serial.print(rightCount);
   Serial.print(" ");
   Serial.println(diff);
   */
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



