/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // Motors
  public static final   int     leftFrontPort     =   0;
  public static final   int     leftBackPort      =   1;
  public static final   int     rightFrontPort    =   2;
  public static final   int     rightBackPort     =   3;
  public static final   int     shoulderPort      =   4;

  //Encoders & Sensors
  public static final   int     encoderPortA      =   2;
  public static final   int     encoderPortB      =   3;
  public static final   int     limitSwitch1Port  =   3;
  public static final   int     limitSwitch2Port  =   4;
  
  //Pneumatics
  public static final   int     pistonOne_1       =   2;
  public static final   int     pistonOne_2       =   3;

  public static final   int     pistonTwo_1       =   4;
  public static final   int     pistonTwo_2       =   5;

  public static final   int     pistonThree_1     =   0;
  public static final   int     pistonThree_2     =   1;

  public static final   int     discBrake_1       =   6;
  public static final   int     discBrake_2       =   7;
  

  //XBOX Controller / JOYSTICK PORTS
  public static final   int     controllerPort    =  0;

  public static final   int     XBOX_A_Button     =  1;
  public static final   int     XBOX_B_Button     =  2;
  public static final   int     XBOX_X_Button     =  3;
  public static final   int     XBOX_Y_Button     =  4;
  public static final   int     XBOX_leftBumper   =  5;
  public static final   int     XBOX_rightBumper  =  6;
  public static final   int     XBOX_backButton   =  7;
  public static final   int     XBOX_startButton  =  8;

  //Components for Arm Torque Calculation **FILLER VALUES, SUBSITUTE REAL NUMBERS - USE SI UNITS
  public static final   double  armMass           = 3;
  public static final   double  centreOfMass      = 0.79;
  public static final   double  gearRatio_1       = 1/63;
  public static final   double  gearRatio_2       = 16/66;
  public static final   double  gearRatio         = gearRatio_1*gearRatio_2;
  public static final   double  scaleFactor       = (4096*gearRatio)/360;

  public static final   double  stallTorque       = 1.4;


  //PRESET ENCODER READINGS
  public static final   double  ticks15deg        = 0;
  
  //PID, Motion Profile
  public static final   double  kP                = 0;
  public static final   double  kI                = 0;
  public static final   double  kD                = 0;
  public static final   double  kF                = 0;

  public static final   double  maxAccel          = 1;
  public static final   double  maxVel            = 10;
  
}
