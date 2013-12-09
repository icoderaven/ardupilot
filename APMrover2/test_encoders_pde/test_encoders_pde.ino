/*
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */
//Count variables
long pos=0;
int last_pos=HIGH;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  attachInterrupt(0,position,CHANGE);
}

// the loop routine runs over and over again forever:
void loop() {
  
  // print out the value you read:
    int motor1 = digitalRead(A0);
  int motor2 = digitalRead(A1);
  
  if (motor1 == LOW && last_pos==HIGH){
   Serial.print("FORWARD"); 
   pos = pos+1;
   last_pos = LOW;
  }
  else if (motor1==HIGH && last_pos==LOW){
    Serial.print("??");
    pos=pos+2;
    last_pos=HIGH;
  }
  Serial.println(pos);
  
}

void position()
{
  // read the input on analog pin 0:

   
}
