#include <I2CEncoder.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
Servo servo_RightMotor;
Servo servo_LeftMotor;

double PingFront();
double PingSide();

double side = 4.5;
double front = 20;
int pushBPin = 4;
int mode =0;

byte b_LowByte;
byte b_HighByte;
const int ci_Ultrasonic_PingSide = 2; //input plug
const int ci_Ultrasonic_DataSide = 3; //output plug
const int ci_Ultrasonic_PingFront = 4; //input plug
const int ci_Ultrasonic_DataFront = 5; //output plug
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
const int ci_Left_Motor_Offset_Address_L = 0;
const int ci_Left_Motor_Offset_Address_H = 1;
const int ci_Right_Motor_Offset_Address_L = 2;
const int ci_Right_Motor_Offset_Address_H = 3;
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Motors_Speed = 1900; // Default run speed
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;
unsigned long ul_Echo_Time;

boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;
boolean bt_Motors_Enabled = true;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

void setup() {
  Wire.begin(); // Wire library required for I2CEncoder library
  Serial.begin(9600);
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false); // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true); // adjust for positive count when moving forward
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
  pinMode(ci_Ultrasonic_PingSide, OUTPUT);
  pinMode(ci_Ultrasonic_DataSide, INPUT);
  pinMode(ci_Ultrasonic_PingFront, OUTPUT);
  pinMode(ci_Ultrasonic_DataFront, INPUT);
}

void loop() 
{
  // set motor speeds
  ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1300, 2100);
  ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1300, 2100);

  followWall();
}
void followWall()
{
 
 while(1)//change this to something like when the claw arm is closed, read it and break
 {
 switch(mode)
  {
  case 0: 
  {
  
 //put find claw code 
  if (PingFront() > front)
  {
    mode=1;//90 degree turn
  }
  else if(PingSide()==side)
  {
  servo_RightMotor.writeMicroseconds(1700);      
  servo_LeftMotor.writeMicroseconds(1700);
  }
  else if(PingSide()>side)
  {
  servo_RightMotor.writeMicroseconds(1600);      
  servo_LeftMotor.writeMicroseconds(1800);
  }
  else if(PingSide()<side)
  {
  servo_RightMotor.writeMicroseconds(1800);      
  servo_LeftMotor.writeMicroseconds(1600);
  } 
  break;//find cube
 }
  case 1: //90 degree turn
  {
    encoder_RightMotor.zero();
    servo_RightMotor.writeMicroseconds(1800);      
    servo_LeftMotor.writeMicroseconds(1400);
    while(encoder_RightMotor.getPosition()<50)//go back and fix this
    {
      
    }
   servo_RightMotor.writeMicroseconds(1700);      
   servo_LeftMotor.writeMicroseconds(1700);
   mode=0;//go back to looking for cube
   break;
  }
 }
}
}
double PingSide()
{
  double sideDist = 0;
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(2, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(2, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  sideDist= pulseIn(3, HIGH, 10000)/58;
  return sideDist;
}
double PingFront()
{
  double frontDist = 0;
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(4, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(4, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  frontDist= pulseIn(5, HIGH, 10000)/58;
  return frontDist;
}
