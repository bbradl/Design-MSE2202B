
// Motor use through Motor Controller

#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>

//servos
Servo Left_Motor;
Servo Right_Motor;
//encoders
I2CEncoder Left_encoder; // Encoders are only used for calibration of motors
I2CEncoder Right_encoder;

//pin assignements
const int Right_Motor_Pin = 8;
const int Left_Motor_Pin = 9;

//constants
int side=4;
int front=20;
//variables
int mode =0;
unsigned long frontDist;
unsigned long sideDist;
void setup() 
{
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
  PingFront();
  PingSide();
  if (frontDist<front&&sideDist==side)
  {
  Right_Motor.writeMicroseconds(1600);      
  Left_Motor.writeMicroseconds(1600);
  }
} 
  }
}
void PingFront()
{
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(2, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(2, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  frontDist= pulseIn(3, HIGH, 10000)/58;
}
void PingSide()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(4, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(4, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  sideDist= pulseIn(5, HIGH, 10000)/58;
}











