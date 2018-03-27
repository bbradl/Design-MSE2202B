
//LIBRARIES
#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>

//SERVOS
Servo Left_Motor;
Servo Right_Motor;
//ENCODERS
I2CEncoder Left_encoder; // Encoders are only used for calibration of motors
I2CEncoder Right_encoder;

//PIN ASSIGNEMENTS
const int Right_Motor_Pin = 8;
const int Left_Motor_Pin = 9;

//GLOBAL VARIABLES
int side=4;
int front=20;
int mode =0;
unsigned long frontDist;
unsigned long sideDist;
void setup() {
 Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);
  //set up ultrasonic a
  pinMode(3,INPUT);
  pinMode(2,OUTPUT);
  //set up ultrasonic b
  pinMode (5,INPUT);
  pinMode (4,OUTPUT);
  // Set up motors
  pinMode(Right_Motor_Pin, OUTPUT);
  Right_Motor.attach(Right_Motor_Pin);
  pinMode(Left_Motor_Pin, OUTPUT);
  Left_Motor.attach(Left_Motor_Pin);
}

void loop() 
{
   switch(mode)
  {
  case 0: //find cube
  {
    //put find cube function
  }
  case 1: //get cube
  {
    //put get cube function here
  }
  case 2: //find pyramid
  {
     //put find pyramid function here
  }
  case 3: //tilt pyramid
  {
    //put tilt pyramid function here
  }
  case 4: //drop cube
  {
    //put drop cube function here
  } 
 case 5: //finish
  {
    //put finish function here
  }
 }
}
//MAIN FUNCTIONS

//SUB FUNCTIONS       
       










