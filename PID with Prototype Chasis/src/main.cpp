/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftMotor            motor_group   11, 20          
// RightMotor           motor_group   1, 10           
// Intake               motor_group   5, 7            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


#include <cmath>






void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

int maxIntegral = 300;
int integralBound = 20;
int turnMaxIntegral = 300;
int nullifiedIntegral = 0;
int SpeedmaxIntegral = 300;
int SpeedintegralBound = 20;
int SpeedturnMaxIntegral = 300;
int SpeednullifiedIntegral = 0;

double SpeedkP=0.009333;
double SpeedkD=0.001;
double SpeedkI=0.0322222;


int SpeedError;
int SpeedprevError=0;
int Speedderivative;
int SpeedtotalError=0;

int desiredSpeed;

bool enableSpeedPID = true;
bool resetSpeedSensors = false;



 ///////////////

 
//Settings
double kP=0.12;//error
double kI=0.0002;//minor change or error
double kD=0.052;//speed up and slow down depend on speed
double turnkP=0;
double turnkI=0;
double turnkD=0;
double signnum_c(double x) {
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return x;
}

//////converting degree/rotation to inches
//////motor rotation in revolutions * gear ratio * (wheel diameter * pi).


int leftError;
int rightError;  //Sensor Value - Desired Value. Positional Value
int leftPrevError=0; // Position 20 milli seconds ago
int rightPrevError=0;
int leftDerivative;
int rightDerivative; // error - prevError = Speed
int leftTotalError=0;  //totalError = totalError + error
int rightTotalError=0;


int turnError;  //Sensor Value - Desired Value. Positional Value
int turnPrevError=0; // Position 20 milli seconds ago
int turnDerivative; // error - prevError = Speed
int turnTotalError=0;  //totalError = totalError + error

//Autonomous Settings
double desiredValue;
double desiredTurnValue;


//Variables Modified for Use
bool enabledrivePID = true;
bool resetDriveSensors = false;

int drivePID(){

  while(enabledrivePID){
  

   

   //////////////////////////////////////
   /////Lateral Movement PID
   //////////////////////////////////////
   //Position of both motors at the start
  
   int leftmotorPosition = LeftMotor.position(degrees);
   int rightmotorPosition = RightMotor.position(degrees);

   //Average of both motors
   

   //Potential how far off the robot is from the goal
   leftError = leftmotorPosition - desiredValue;
   rightError = rightmotorPosition - desiredValue;

   //Derivative
   leftDerivative = leftError - leftPrevError; 
   rightDerivative = rightError - rightPrevError;

   //Integral
   leftTotalError +=leftError;
    rightTotalError +=rightError; 

    //totalError += error;
   
   double leftLateralMotorPower = (leftError * kP + leftDerivative * kD + leftTotalError *kI); 
   double rightLateralMotorPower = (rightError * kP + rightDerivative * kD + rightTotalError *kI);

   /////////////////////////////////////////////////////////////
   //////////////////////////////////////
   /////Turning Movement PID
   //////////////////////////////////////
   //Average of both motors

   int turnDifference = leftmotorPosition - rightmotorPosition;

   //Potential
   turnError = turnDifference - desiredTurnValue;

   //Derivative
   turnDerivative = turnError - turnPrevError;

   //Integral
   turnTotalError += turnError;
  

  if(abs(leftError) < integralBound){
    leftTotalError+=leftError; 
    }  else {
    leftTotalError = 0; 
    }
  if(abs(rightError) < integralBound){
    rightTotalError+=rightError; 
    }  else {
    rightTotalError = 0; 
    }
    //totalError += error;

    leftTotalError = abs(leftTotalError) > maxIntegral ? signnum_c(leftTotalError) * maxIntegral : leftTotalError;
    rightTotalError = abs(rightTotalError) > maxIntegral ? signnum_c(rightTotalError) * maxIntegral : rightTotalError;
    //totalError += error;

   double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;
   //////////////////////////////////////
   //Because of the way that the motor is coded in The code cannot be run on a drive train or when the Left and Right side have the same value (Error, prevError) 
   //this will cause the error to be inaccurate since the error value can cancel out, so when we give the two side two their own error it become more accurate
   LeftMotor.spin(reverse,leftLateralMotorPower + turnMotorPower,voltageUnits::volt);
   RightMotor.spin(reverse,rightLateralMotorPower + turnMotorPower,voltageUnits::volt);
   

  
   


   //code
   leftPrevError =leftError;
   rightPrevError = rightError;
   turnPrevError =turnError;
   vex::task::sleep(20);


  }

  return 1;
}

void reset(){
    LeftMotor.setPosition(0,degrees);
    RightMotor.setPosition(0,degrees);
    
    

   }

//Settings for manual control



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

  vex::task drive(drivePID);
  
  reset();
  desiredValue = 800;
  vex::task::sleep(1000);
  reset();
  desiredValue = 800;
  vex::task::sleep(1000);
  
while(1){
 Brain.Screen.print(static_cast<float>(RightMotor.position(degrees)));
 Brain.Screen.newLine();
 Brain.Screen.print(static_cast<float>(LeftMotor.position(degrees)));
 
    Brain.Screen.newLine();
    wait(2000,msec);
  Brain.Screen.clearScreen();
}
  vex::task::sleep(1000);
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
  
 


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  LeftMotor.setVelocity(100,percent);
  RightMotor.setVelocity(100,percent);
  Controller1.ButtonR1.released(IntakeOff1);
  Controller1.ButtonR2.pressed(IntakeR2);
  Controller1.ButtonR2.released(IntakeOff2);
  Controller1.ButtonR1.pressed(IntakeR1);
  
  
 
  


  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
  


}
