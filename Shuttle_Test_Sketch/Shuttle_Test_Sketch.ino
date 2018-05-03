#include <Wire.h>
#include <SparkFunDS1307RTC.h>
#include <previousTime.h>


//definitions for directions of the motors
#define OPEN   2
#define CLOSE  1

//definitions for actuators
#define Right 0
#define Left 1

//initialize pins for actuators
int inApin[2] = {7,4};
int inBpin[2] = {8,9};
int pwmpin[2] = {5,6};
int cspin[2] = {2, 3};
int enpin[2] = {0, 1};

/////////////////////////////////////TIME VARIABLES/////////////////////////////////////
unsigned int DelayForMotors = 42;           //int variable that determines how long the program waits for motors to open/close
bool ReadPreviousSecond = false;
bool DebugFlag = true;
previousTime PreviousTime(0, 0, 0);

/////////////////////////////////////DOOR MOVEMENT VARIABLES/////////////////////////////////////
//input pins for the buttons
int SwitchPinRight = 10;    //switchd definition for the right switch
int SwitchPinLeft = 11;     //switchd definition for the left s ~``witch
int valRight = 0;           //value for the right switch
int valLeft = 0;            //value for the left switch

void setup()
{
  Serial.begin(9600);

  rtc.begin();
  //rtc.setTime(00, 27, 12, 6, 19, 5, 17);   //set the time to 12:15:00 Friday, May 19, 2017
  
  //initialize the switches for input
  pinMode(SwitchPinRight, INPUT_PULLUP);
  pinMode(SwitchPinLeft, INPUT_PULLUP);

  //initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }

  //make sure the motors are stopped
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}


void loop()
{
  rtc.update();
  /*
  //CloseDoors();
  motorGo(Right, OPEN, 255);
  motorGo(Left, OPEN, 250);
  delay(60000);
  */

  Serial.println("Current time: " + String(rtc.hour()) + ":" + String(rtc.minute()));
  
/*
   if (digitalRead(SwitchPinRight) == LOW && digitalRead(SwitchPinLeft) == LOW)
  {
    Serial.println("Both Switches are pressed");
  }
  else if(digitalRead(SwitchPinRight) == HIGH && digitalRead(SwitchPinLeft) == LOW)
  {
    Serial.println("Left Switch are pressed");
  }
  else if(digitalRead(SwitchPinRight) == LOW && digitalRead(SwitchPinLeft) == HIGH)
  {
    Serial.println("Right Switch are pressed");
  }

/*
  val = digitalRead(SwitchPin);

  //while the button is unpressed
  while (val != HIGH && flag == false)
  {
    //run the motor for 100 miliseconds
    motorGo(Right, IN, 255);
    //read the pin again
    val = digitalRead(SwitchPin);
  }
  Serial.println("Switch was pressed");
  flag = true;
  motorOff(Right);
  */
}

void CloseDoors()
{
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The purpose of this function is to close the doors on the shuttle by using two switches and a safety timer.//
  //Once a switch is pressed, its coorisponding door stops, and if both                                        //
  //doors are closing and the timer runs out, then both doors stop                                             //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //get the value of the switches
  //valRight = digitalRead(SwitchPinRight);
  //valLeft = digitalRead(SwitchPinLeft);
    
  //if both switches are unpressed
  if (digitalRead(SwitchPinRight) == LOW && digitalRead(SwitchPinLeft) == LOW)
  {
    Serial.println("Both switches are un-pressed");
    //run both motors
    motorGo(Right, CLOSE, 255);
    motorGo(Left, CLOSE, 255);
    //get the value of the switches
    //valRight = digitalRead(SwitchPinRight);
    //valLeft = digitalRead(SwitchPinLeft);
  }
  //if the left switch is pressed and the right is unpressed
  else if (digitalRead(SwitchPinRight) == LOW && digitalRead(SwitchPinLeft) == HIGH)
  {
    //turn off the left motor
    motorOff(Left);
    //run the right motor
    motorGo(Right, CLOSE, 255);
    //continue to read the right switch
    //valRight = digitalRead(SwitchPinRight);
   }
   //if the right switch is pressed and the right is unpressed
   else if (digitalRead(SwitchPinRight) == HIGH && digitalRead(SwitchPinLeft) == LOW)
   {
      //turn off the right motor
      motorOff(Right);
      //run the left motor
      motorGo(Left, CLOSE, 255);
      //continue to read the left switch
      //valLeft = digitalRead(SwitchPinLeft);
   }
   //if both switches are pressed
   else 
   {
      //turn off the actuators
      motorOff(Right);
      motorOff(Left);
    }
} //end sub


void motorOff(int motor)
{
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

