// Test Run 

#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <SoftwareSerial.h>

//Claw
Servo Gripper;
Servo Spinner;
const int gripper_Servo = 7;
const int spin_Servo = 6;
int cube_Found = 0;

//IR
SoftwareSerial IR(7,11);
Servo urMyBoyBlue;
int IRsensorPin = 7;
int blueServoPin = 10;
int switchPin = A1;
int yay = -1;

// Motor Calibration
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
long rightWheelPosition;
long leftWheelPosition;
byte b_LowByte;
byte b_HighByte;
unsigned long leftMotorOffset;
unsigned long rightMotorOffset;
int check = 0;
int timeStart = 0;
bool started = false;

//Oval
int increaseFactor = 0;
int turnCounter = 0;
bool switchOrNah = 0;


// Wall Following 
double desired_Distance = 6.5;
double max_Eyes = 7;
double leftP_factor;
double rightP_factor;
double ul_Echo_Time_Front;
double ul_Echo_Time_Back;
double ul_Echo_Time_Eyes;
double distance_Error; // Distance of the front ultrasonic
double angle_Error;
double eyes_Error;
double front_Distance;
double back_Distance;
double eyes_Distance;
const int right_Motor_Pin = 8;
const int left_Motor_Pin = 9;
const int front_Ultrasonic_Trigger = 2;
const int front_Ultrasonic_Echo = 3;
const int back_Ultrasonic_Trigger = 4;
const int back_Ultrasonic_Echo = 5;
const int eyes_Ultrasonic_Trigger = 6;
const int eyes_Ultrasonic_Echo = 7;
Servo right_Motor;
Servo left_Motor;
long right_Motor_Speed;
long left_Motor_Speed;
int pb4 = A0;
unsigned long ninety_Timer = 10000;
unsigned long current_Millis;
int ninety_Count = 0;
double ticksPerCM = 22.38;

//misc
int mode = 0;

void setup()
{
  Serial.begin(9600);
  Wire.begin(); // Wire library required for I2CEncoder library

  // Push Button
  pinMode(pb4, INPUT_PULLUP);
  
  // Motors
  pinMode(right_Motor_Pin, OUTPUT);
  right_Motor.attach(right_Motor_Pin);
  pinMode(left_Motor_Pin, OUTPUT);
  left_Motor.attach(left_Motor_Pin);

  // Ultrasonics
  pinMode(front_Ultrasonic_Trigger, OUTPUT);
  pinMode(front_Ultrasonic_Echo, INPUT);
  pinMode(back_Ultrasonic_Trigger, OUTPUT);
  pinMode(back_Ultrasonic_Echo, INPUT);
  pinMode(eyes_Ultrasonic_Trigger, OUTPUT);
  pinMode(eyes_Ultrasonic_Echo, INPUT);

  // Encoders
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

  //Calibration counters
  check = 0;
  timeStart = 0;
  started = false;

  //IR
  pinMode(IRsensorPin, INPUT);
  pinMode(blueServoPin, OUTPUT);
  pinMode(switchPin, OUTPUT);
  urMyBoyBlue.attach(blueServoPin);

  //Claw
  pinMode(gripper_Servo, OUTPUT);
  Gripper.attach(gripper_Servo);
  pinMode(spin_Servo, OUTPUT);
  Spinner.attach(spin_Servo);

  // Set gripper and spinner to proper positions
  Gripper.write(180);
  Spinner.write(0);
    
  // Variables that need to reset
  ninety_Count = 0;
}

void loop()
{
  switch(mode)
  {
    case 0: //follow wall and find cube, also has calibration
    {
      followWall();
      cubeFinder();
    
      //  Calibration
      if ((analogRead(pb4) < 250) && (check == 0))
      { 
        calibratioMagico();
      }
      break;
     }
    case 1:
    {
      followWall();
      IRsensei();
      break;
    }
    case 2:
    {
      //hardcode it to slam down on the pyramid, future B.O.I.S. problems
      break;
    }
  }
}
void followWall()
{
  // Drive 
    // Reset Error factor
    // Get ping distances
    leftP_factor = 0;
    rightP_factor = 0;
    front_Distance = Side_Ping_Front();
    back_Distance = Side_Ping_Back();
    eyes_Distance = Eyes_Ping();
    desired_Distance = 6.5 + increaseFactor;
    max_Eyes = 9 + increaseFactor;



    // For testing
    Serial.println("Forward");
    Serial.println(eyes_Distance);
    
    
    //Serial.print("Front: ");
    //Serial.println(front_Distance);
    //Serial.print("Back: ");
    //Serial.println(back_Distance);

    // Calculate error and see if 90 degree turn is coming up
    angle_Error = (front_Distance - back_Distance);
    distance_Error = (front_Distance - desired_Distance);
    
    eyes_Error = (eyes_Distance - max_Eyes);
    
    Serial.println("Error");
    Serial.println(eyes_Error);
  //turn left close to wall
  if ((eyes_Error <= 0) && (turnCounter <= 3)) //&& (((millis() - current_Millis) > ninety_Timer) || (ninety_Count == 0)))
    {
      ninetyTurn();
      //increment counter for narrowing the d in other function
      turnCounter++;
    }
    //Increment the distance from the wall every third turn
    if (!(turnCounter%3)&&(switchOrNah!=0))
    {
      increaseFactor += 12;
    }
     // Robot points away from wall
     if (angle_Error > 0) 
     {    
        rightP_factor += angle_Error*23.0;
     }
  
    // Robot points towards wall
    if (angle_Error < 0) 
    {   
      leftP_factor -= angle_Error*23.0;
    }
    
    // Front of robot is too far from wall
    if (distance_Error > 0) 
    {     
      rightP_factor += distance_Error*23.0;
    }

    // Front of robot is too close to wall
    if (distance_Error < 0) 
    {   
      leftP_factor -= distance_Error*40.0;
    }
   
    //left_Motor_Speed = constrain(1800 + leftMotorOffset, 1200, 2100);
    //right_Motor_Speed = constrain(1800 + rightMotorOffset, 1200, 2100);
    left_Motor_Speed = (1800 - leftP_factor);
    right_Motor_Speed = (1800 - rightP_factor);
    left_Motor.writeMicroseconds(left_Motor_Speed);
    right_Motor.writeMicroseconds(right_Motor_Speed);
    /*Serial.println(angle_Error);
    Serial.println(" ");
    Serial.println("Left:");
    Serial.println(leftP_factor);
    Serial.println("Right:");
    Serial.println(rightP_factor);*/
}
void ninetyTurn()
{
  delay(40);
  Serial.println("Yup");
  Serial.println("1");
  encoder_RightMotor.zero();
  encoder_LeftMotor.zero();
  // Angle back
  left_Motor.writeMicroseconds(1375);
  right_Motor.writeMicroseconds(1220);
  Serial.print("im in");
  //reverse left motor 10cm
  while(encoder_LeftMotor.getRawPosition() > -ticksPerCM*13)
  {
    Serial.println(encoder_LeftMotor.getRawPosition());
  }
  // Angle again
  left_Motor.writeMicroseconds(1220);
  right_Motor.writeMicroseconds(1375);
  encoder_RightMotor.zero();
  encoder_LeftMotor.zero();
  //reverse right motor 10cm aka square her up
  while(encoder_RightMotor.getRawPosition() > -ticksPerCM*22)
  {
    
  }
  Serial.println("2");
  // Drive back towards the wall
  left_Motor.writeMicroseconds(1650);
  right_Motor.writeMicroseconds(1650);
  encoder_RightMotor.zero();
  encoder_LeftMotor.zero();
  //go foward 20cm
  while(encoder_LeftMotor.getRawPosition() < ticksPerCM*38)
  {
    
  }
  Serial.println("3");
  // Get parallel to the wall
  left_Motor.writeMicroseconds(1350);
  right_Motor.writeMicroseconds(1650);
  //theoretically should be a 90 degree pivot so the distance the right wheel should
  //spin is 2*(pi/4)*(distance from wheel to centre) if it is truly pivoting
  encoder_RightMotor.zero();
  encoder_LeftMotor.zero();
  while(encoder_RightMotor.getRawPosition() < ticksPerCM*23.36)
  {
    
  }  
  Serial.println("magic");  
  // Reset for ninety turns
  current_Millis = millis();
  ninety_Count++;  
}
void IRsensei()
{
  urMyBoyBlue.writeMicroseconds(90);
  checkRed();
  if(yay)
  {
    //stop the bus
    right_Motor.writeMicroseconds(1500);      
    left_Motor.writeMicroseconds(1500);
    if ((analogRead(switchPin) > 800) && ((IR.read()=='A')||(IR.read() == 'E')))//Switch high means A is right
    {
      mode = 2;
    }
    else if((analogRead(switchPin) < 300) && ((IR.read()=='A')||(IR.read() == 'E')))
    {
      mode = 2;
    }
    else
    {
      //fake pyramid
      yay = 0;
    }
  }
}
void checkRed()
{
  if(IR.available())
  {
    yay = 1;
  }
  else
  {
    yay = 0;
  }
}
double Side_Ping_Front()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  double distance;
  digitalWrite(front_Ultrasonic_Trigger, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(front_Ultrasonic_Trigger, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time_Front = pulseIn(front_Ultrasonic_Echo, HIGH, 10000);
  // Return ping value
  distance = ul_Echo_Time_Front/58.0;
  return distance;
} 

double Side_Ping_Back()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  double distance;
  digitalWrite(back_Ultrasonic_Trigger, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(back_Ultrasonic_Trigger, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time_Back = pulseIn(back_Ultrasonic_Echo, HIGH, 10000);
  // Return ping value
  distance = ul_Echo_Time_Back/58.0;
  return distance;
} 

double Eyes_Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  double distance;
  digitalWrite(eyes_Ultrasonic_Trigger, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(eyes_Ultrasonic_Trigger, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time_Eyes = pulseIn(eyes_Ultrasonic_Echo, HIGH, 10000);
  // Return ping value
  distance = ul_Echo_Time_Eyes/58.0;
  return distance;
}

void cubeFinder()
{
  cube_Found = Serial.read();
  Serial.print(cube_Found);
  

  if (cube_Found == 1)
  {
    left_Motor.writeMicroseconds(1500);
    right_Motor.writeMicroseconds(1500);
    Gripper.write(0);
    delay(1000);
    Spinner.write(100);
    delay(1000);
    Gripper.write(180);
    delay(1000);
    Gripper.detach();
    Spinner.detach();
    switchOrNah = 1;
    mode = 1;
  }
}
void calibratioMagico()
{
  if(!started)
  {
   Serial.print("Top");
   started = true;
   encoder_LeftMotor.zero();
   encoder_RightMotor.zero();
   timeStart = millis();
   left_Motor.writeMicroseconds(1900);
   right_Motor.writeMicroseconds(1900);
  }
  else if((millis() - timeStart) > 5000)
  {
    left_Motor.writeMicroseconds(1500);
    right_Motor.writeMicroseconds(1500);
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
    Serial.print("bottom");
    check++;
  }
}

