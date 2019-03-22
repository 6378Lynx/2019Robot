/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Author: Abdur Javaid                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.RobotMap;
import frc.robot.commands.autonomous.DriveForwardCommand;
import frc.robot.commands.teleopDriveCommand;
import frc.robot.Robot;
import frc.robot.CheesyPID;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem implements Loggable {

  //Singular motors
  private Spark leftBack = new Spark(RobotMap.leftBackPort);
  private Spark leftFront = new Spark(RobotMap.leftFrontPort);

  private Spark rightBack = new Spark(RobotMap.rightBackPort);
  private Spark rightFront = new Spark(RobotMap.rightFrontPort);

  //Group Motors
  private SpeedControllerGroup leftMotor = new SpeedControllerGroup(leftBack, leftFront);
  private SpeedControllerGroup rightMotor = new SpeedControllerGroup(rightBack, rightFront);

  //Drive
  private DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  //Drive Encoders
  private Encoder leftEncoder = new Encoder(RobotMap.leftDriveEncoder_A, RobotMap.leftDriveEncoder_B, false, Encoder.EncodingType.k4X);
  private Encoder rightEncoder = new Encoder(RobotMap.rightDriveEncoder_A, RobotMap.rightDriveEncoder_B, true, Encoder.EncodingType.k4X);

  //Gyro
  @Log.Gyro
  private Gyro gyro = new ADXRS450_Gyro();

  //Use Constants after tuning
  @Config.PIDController
  private CheesyPID leftPID = new CheesyPID(0,0,0,0);
  @Config.PIDController
  private CheesyPID rightPID = new CheesyPID(0,0,0,0);
  @Config.PIDController
  private CheesyPID gyroPID = new CheesyPID(0,0,0,0);

  private double prevTime   = Double.NaN;
  private double currentTime;

  public DriveSubsystem(){
    leftEncoder.setDistancePerPulse(RobotMap.driveDistPerPulse);
    rightEncoder.setDistancePerPulse(RobotMap.driveDistPerPulse);

    leftPID.setOutputRange(-1, 1);
    rightPID.setOutputRange(-1, 1);
    gyroPID.setInputRange(-360,360);
  }

    
  //USED IN TELEOP-PERIODIC
  public void teleopDrive(double frontSpeed, double sideSpeed){

    //Slow down movement to 1/4 when Right Bumper is pressed
    if(Robot.oi.controller.getBumper(GenericHID.Hand.kRight)){
      drive.curvatureDrive(frontSpeed/4, sideSpeed/4, Robot.oi.controller.getBumper(GenericHID.Hand.kRight));
    }

    //Otherwise use Right Trigger (weirdly acts as a button on the controller instead of an axis) -> Port 8
    else{
      drive.curvatureDrive(frontSpeed, sideSpeed, Robot.oi.controller.getRawButton(8));
    }
  }

  //USED IN AUTO
  public void setAutonDrive(double setpoint, double gyroAngle, double leftMotionFeedForward, double rightMotionFeedForward){
    currentTime = Timer.getFPGATimestamp();
    leftPID.setSetpoint(setpoint);
    rightPID.setSetpoint(setpoint);
    gyroPID.setSetpoint(gyroAngle);

    //calculate dT if its not the first loop, otherwise just use 0.02 (50Hz)
    double dt = prevTime == Double.NaN ? 0.02 : currentTime-prevTime;

    double leftOut = leftPID.calculate(leftEncoder.getRaw(), dt) + leftMotionFeedForward;
    double rightOut= rightPID.calculate(rightEncoder.getRaw(), dt) + rightMotionFeedForward;
    double straightOut = gyroPID.calculate(gyro.getAngle(), dt);

    drive.tankDrive(leftOut+straightOut, rightOut-straightOut);

    System.out.printf("Left Output: %f | Right Output %f | Left FeedForward %f | Right Feedforward %f", leftOut, rightOut, leftMotionFeedForward, rightMotionFeedForward);

    prevTime = currentTime;
  }

  public void turnToAngle(double angle){
    currentTime = Timer.getFPGATimestamp();
    double dt = prevTime == Double.NaN ? 0.02 : currentTime-prevTime;
    gyroPID.setSetpoint(angle);
    double output = gyroPID.calculate(gyro.getAngle(), dt);
    drive.tankDrive(output,-output);
    prevTime = currentTime;
  }



  
  //Configs(Using 449's Library to make it easier) - Make it easier to tune values when testing

  //Right Side
  @Log
  public double getRightEncoder(){
    return rightEncoder.getDistance();
  }
  @Log
  public double getLeftEncoder(){
    return leftEncoder.getDistance();
  }


  @Override
  public void initDefaultCommand() {
    // Default -> TeleopDriveCommand so its always running
    setDefaultCommand(new teleopDriveCommand());
  }
}