/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Author: Abdur Javaid                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    // Motors
    public static final int leftFrontPort = 0;
    public static final int leftBackPort = 1;
    public static final int rightFrontPort = 2;
    public static final int rightBackPort = 3;
    public static final int shoulderPort = 4;

    //DIGITAL INPUTS
    public static final int leftDriveEncoder_A = 0;
    public static final int leftDriveEncoder_B = 1;
    public static final int rightDriveEncoder_A = 2;
    public static final int rightDriveEncoder_B = 3;
    public static final int armEncoder_A = 4;
    public static final int armEncoder_B = 5;
    public static final int limitSwitchTopPort = 6;
    public static final int limitSwitchBotPort = 7;

    //Pneumatics - PCM 0
    public static final int pistonOne_1 = 0;
    public static final int pistonOne_2 = 1;

    public static final int pistonTwo_1 = 2;
    public static final int pistonTwo_2 = 3;

    public static final int pistonThree_1 = 4;
    public static final int pistonThree_2 = 5;

    public static final int discBrake_1 = 6;
    public static final int discBrake_2 = 7;

    //Pneumatics - PCM 1


    //XBOX Controller / JOYSTICK PORTS
    public static final int controllerPort = 0;
    public static final int operatorPort = 1;

    public static final int XBOX_A_Button = 2;
    public static final int XBOX_B_Button = 3;
    public static final int XBOX_X_Button = 1;
    public static final int XBOX_Y_Button = 4;
    public static final int XBOX_leftBumper = 5;
    public static final int XBOX_rightBumper = 6;
    public static final int XBOX_backButton = 7;
    public static final int XBOX_startButton = 8;

    //XBOX Controller / OPERATOR PORTS


    //Components for Arm Torque Calculation
    public static final double armMass = 12.5;
    public static final double centreOfMass = 0.79;
    public static final double gearRatio_1 = 1.0 / 63.0;
    public static final double gearRatio_2 = 16.0 / 66.0;
    public static final double gearRatio = gearRatio_1 * gearRatio_2;
    public static final double scaleFactor = (4096 * gearRatio) / 360;
    public static final double stallTorque = 1.4;


    //PRESET ENCODER READINGS
    private static final double driveEncoderSens = 2048.0;
    private static final double wheelDiameter = 0.5;

    private static final double armEncoderSens = 2048.0;
    public static final double armDistPerPulse = (1 / armEncoderSens * gearRatio) * 2 * Math.PI;

    //Wheel Diameter = 6" => 0.5' -> returns distance per pulse in feet
    public static final double driveDistPerPulse = (1 / driveEncoderSens) * wheelDiameter * Math.PI;

    public static final double rocketPosition = 70000;

    //PID, Motion Profile
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;


    // -- Stuff for Auto

    /*
     **
     * FOR ARM
     * V = ka*accel + kv*vel
     * V = 8V (you can only give 12v to motor, 2v is used by feedforward ish)
     * ka = 12/1300
     * kv = 12/134

     * approx 50deg/s as max vel
     * 8v = ka*accel + kv*50deg/s
     * 8v = ka*accel + 4.45v
     * 3.55 = ka*accel
     * accel = 3.55/ka
     **
     */

    public static double getVoltageComp;

    public static final double arm_kv = 12.0 / 134.0;
    public static final double arm_ka = 12.0 / 1300.0;
    public static final int kDeg = 21;

    public static final double armMaxAccel = 200;
    public static final double armMaxVel = 50;

    public static final double drive_kv = 0.48;
    public static final double drive_ka = 0.045;
    public static final double Vintercept = 1.7;

    //124
    public static final double driveMaxAccel = 30;
    public static final double driveMaxVel = 5;

}