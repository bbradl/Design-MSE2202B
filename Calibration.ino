#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include <I2CEncoder.h>
Servo servo_RightMotor;
Servo servo_LeftMotor;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
unsigned int leftMotor;
unsigned int rightMotor;
long rightWheelPosition;
long leftWheelPosition;
int right_Motor_Pin = 8;
int left_Motor_Pin = 9;
byte b_LowByte;
byte b_HighByte;
unsigned long leftMotorOffset;
unsigned long rightMotorOffset;

void setup() {
  Wire.begin(); // Wire library required for I2CEncoder library
  Serial.begin(9600);
  pinMode(right_Motor_Pin, OUTPUT);
  servo_RightMotor.attach(right_Motor_Pin);
  pinMode(left_Motor_Pin, OUTPUT);
  servo_LeftMotor.attach(left_Motor_Pin);
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false); // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true); // adjust for positive count when moving forward
  b_LowByte = EEPROM.read(3);
  b_HighByte = EEPROM.read(4);
  leftMotorOffset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(0);
  b_HighByte = EEPROM.read(1);
  rightMotorOffset = word(b_HighByte, b_LowByte);
}

void loop() {
  // put your main code here, to run repeatedly:
  calibratioMagico();
}
void calibratioMagico()
{
  int timeStart;
  bool started = false;
  if(!started){
   started = true;
   encoder_LeftMotor.zero();
   encoder_RightMotor.zero();
   timeStart = millis();
   servo_LeftMotor.writeMicroseconds(1750);
   servo_RightMotor.writeMicroseconds(1750);
  }
  else if((millis() - timeStart) > 5000)
  {
    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);
    leftWheelPosition = encoder_LeftMotor.getRawPosition();
    rightWheelPosition = encoder_RightMotor.getRawPosition();
    if (leftWheelPosition > rightWheelPosition)
    {
      leftMotorOffset = (leftWheelPosition - rightWheelPosition)/4;
      rightMotorOffset = 0;
    }
    else
    {
      rightMotorOffset = (rightWheelPosition - leftWheelPosition)/4;
      rightMotorOffset = 0;
    }
    EEPROM.write(0, lowByte(rightMotorOffset));
    EEPROM.write(1, highByte(rightMotorOffset));
    EEPROM.write(2, lowByte(leftMotorOffset));
    EEPROM.write(3, highByte(leftMotorOffset));
  }
 
}

