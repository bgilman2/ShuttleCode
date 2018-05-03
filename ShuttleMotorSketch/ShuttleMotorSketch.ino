
//////////////////////////////////////////////////////
//            Shuttle Motor Program                 //
//              by Brett Gilman                     //
//           last updated: 4.4.2017                 //
//The purpose of this program is to open and close  //
//the shuttle bay doors of the scale shuttle        //
//model that the art room is making. This program   //
//opens and closes the doors every 87min, starting  //
//at 8am.                                           //
//////////////////////////////////////////////////////

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

//variable for timer
previousTime PreviousTime(0, 0, 0);
bool ReadPreviousSecond = false;
int lastTimeMin = 0;


/////////////////////////////////////DOOR MOVEMENT VARIABLES/////////////////////////////////////
//input pins for the buttons
int SwitchPinRight = 10;    //switchd definition for the right switch
int SwitchPinLeft = 11;     //switchd definition for the left switch
int valRight = 0;           //value for the right switch
int valLeft = 0;            //value for the left switch


////////////////////////////////////STATE MACHINE VARIABLES////////////////////////////////////
enum DoorStates {
  Wait, 
  Closing,  
  Opening
};

DoorStates State = Wait; //default state
bool NewStateFlag = false;
bool CheckTime = true;

//structure that holds the time and state
typedef struct  
{
  int hours;
  int minutes;
  DoorStates newState;
} STATETIME;

//an array containing all of the open/close times
STATETIME ActionTimes[8] = {
  {8, 0, Opening},
  {9, 27, Closing},
  {10, 54, Opening},
  {12, 21, Closing},
  {13, 48, Opening},
  {15, 15, Closing},
  {20, 49, Opening},
  {2, 23, Closing}
};

void setup()
{
  Serial.begin(9600);

  //setup the RTC module
  rtc.begin();                            //start the rtc
  //rtc.set24Hour();                        //make sure it is in 24 hour mode
  //rtc.setTime(00, 34, 12, 6, 19, 5, 17);   //set the time to 12:34:00 Friday, May 19, 2017
  
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

  //Serial.println("Program starting..");
  //Serial.println("Current time: " + String(rtc.hour()) + ":" + String(rtc.minute()));
}

void loop()
{
    //update the rtc module
    rtc.update();                 

    if (rtc.minute() != lastTimeMin && CheckTime == true)
    {
        //for every item in the time structure
        for (int i = 0; i <= 7; i++)
        {
          if (!NewStateFlag)
          {
            //if it is time to open or close
            if (rtc.hour() == ActionTimes[i].hours && rtc.minute() == ActionTimes[i].minutes)
            {
              lastTimeMin = ActionTimes[i].minutes;
              State = ActionTimes[i].newState; 
              NewStateFlag = true;
              CheckTime = false;
            }
            else
              State = Wait; //the shuttle is closed/opened, wait
          }
        }
    }
    //check the state of the doors
    switch (State)
    {
      case Closing:
        if (rtc.second() == CalculateSeconds() || (digitalRead(SwitchPinRight) == LOW && digitalRead(SwitchPinLeft) == LOW))
        {
          ReadPreviousSecond = false;       //stop reading the seconds
          NewStateFlag = false;
          lastTimeMin = rtc.minute();
          State = Wait;
          BothMotorsOff();                     
        }
        else
          CloseDoors();                       //close the doors until the switches are triggered 
      break;
    
      case Opening:
          //allow the doors to fully open (using a statemachine)
          if (rtc.second() == CalculateSeconds())
          {
            ReadPreviousSecond = false;     //stop reading seconds
            lastTimeMin = rtc.minute();
            NewStateFlag = false;
            State = Wait;                   //the shuttle is now opened
            BothMotorsOff();                //turn off the motors
          }
          else  
          {
            //have the actuators move outward (open doors)
            motorGo(Left, OPEN, 255);
            motorGo(Right, OPEN, 205);
          }
      break;
      case Wait:
        BothMotorsOff();
        CheckTime = true;
      break;
    } //end switch
  } //end if

//this is a simple subroutine that turns off both of the motors
void BothMotorsOff()
{
  motorOff(Right);
  motorOff(Left);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The purpose of this function is to close the doors on the shuttle by using two switches and a safety timer.//
//Once a switch is pressed, its coorisponding door stops, and if both                                        //
//doors are closing and the timer runs out, then both doors stop                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CloseDoors()
{  
  //if both switches are unpressed
  if (digitalRead(SwitchPinRight) == HIGH && digitalRead(SwitchPinLeft) == HIGH)
  {
    //run both motors
    motorGo(Right, CLOSE, 205);
    motorGo(Left, CLOSE, 255);
  }
  //if the left switch is pressed and the right is unpressed
  else if (digitalRead(SwitchPinRight) == HIGH && digitalRead(SwitchPinLeft) == LOW)
  {
    //turn off the left motor
    motorOff(Left);
    //run the right motor
    motorGo(Right, CLOSE, 205);
    delay(5); //delay to allow the motor to keep going
   }
   //if the right switch is pressed and the left is unpressed
   else if (digitalRead(SwitchPinRight) == LOW && digitalRead(SwitchPinLeft) == HIGH)
   {
      //turn off the right motor
      motorOff(Right);
      //run the left motor
      motorGo(Left, CLOSE, 255);
      delay(5); //delay to allow the motor to keep going
   }
   //if both switches are pressed
   else 
   {
      //turn off the actuators
      motorOff(Right);
      motorOff(Left);
    }
} //end sub

int CalculateSeconds()
{
  if (!ReadPreviousSecond)                              //if the program hasn't read the seconds
  {
    PreviousTime.Second = rtc.second();                 //save the time
    ReadPreviousSecond = true;                          //flag that the program has read the time
  }
  if ((PreviousTime.Second + DelayForMotors) > 60)      //if the current second plus the time delay is greater than 60
    return (PreviousTime.Second + DelayForMotors) - 60; //return the sum - 60
  else
    return PreviousTime.Second + DelayForMotors;        //return the sum
}

//This function turns off the motor that is passed in
void motorOff (int motor)
{
  for (int i = 0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}


//This funtion turns on a motor with a specific direction along with a passed in PWM
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  //if the motor is less than or equal to 1
  if (motor <=1)
  {
    //if the direction is less than or equal to 4
    if (direct <= 4)
    {
      //set inA[motor]
      if (direct <= 1)
      {
        //turn on the motor
        digitalWrite(inApin[motor], HIGH);
      }
      else 
      {
        //else, turn it off
        digitalWrite(inApin[motor], LOW);
      }

      //set inB[motor]
      if ((direct==0)||(direct==2))
      {
        //turn on the motor
        digitalWrite(inBpin[motor], HIGH);
      }
      else 
      {
        //else, turn off the motor
        digitalWrite(inBpin[motor], LOW);
      }
        
      //set the pwm
      analogWrite(pwmpin[motor], pwm);
    }
  }
}

