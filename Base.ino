

// Motor use through Motor Controller

#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo Left_Motor;
Servo Right_Motor;

I2CEncoder Left_encoder; // Encoders are only used for calibration of motors
I2CEncoder Right_encoder;

const int Right_Motor_Pin = 8;
const int Left_Motor_Pin = 9;
int mode =1;
void setup() 
{
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

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
    case 1: //wall follow
  Right_Motor.writeMicroseconds(1600);      
  Left_Motor.writeMicroseconds(1600);
  }
} 











