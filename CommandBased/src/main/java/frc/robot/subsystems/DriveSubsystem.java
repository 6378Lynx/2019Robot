/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Author: Abdur Javaid                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.RobotMap;
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
  Spark leftBack = new Spark(RobotMap.leftBackPort);
  Spark leftFront = new Spark(RobotMap.leftFrontPort);
  Spark rightBack = new Spark(RobotMap.rightBackPort);
  Spark rightFront = new Spark(RobotMap.rightFrontPort);
  //Group Motors
  SpeedControllerGroup leftMotor = new SpeedControllerGroup(leftBack, leftFront);
  SpeedControllerGroup rightMotor = new SpeedControllerGroup(rightBack, rightFront);
  //Drive
  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  //Drive Encoders
  Encoder leftEncoder = new Encoder(RobotMap.leftDriveEncoder_A, RobotMap.leftDriveEncoder_B);
  Encoder rightEncoder = new Encoder(RobotMap.rightDriveEncoder_A, RobotMap.rightDriveEncoder_B);
  //Gyro
  @Log.Gyro
  AnalogGyro gyro = new AnalogGyro(1);

  private double scalingFactor;
  private double prevTime   = Double.NaN;
  private double currentTime;
  
  //Use Constants after tuning
  CheesyPID leftPID = new CheesyPID(0,0,0,0);
  CheesyPID rightPID = new CheesyPID(0,0,0,0);

  public DriveSubsystem(){
    leftEncoder.setDistancePerPulse(0);
    rightEncoder.setDistancePerPulse(0);

    leftPID.setOutputRange(-1, 1);
    rightPID.setOutputRange(-1, 1);
  }

  //Configs(Using 449's Library to make it easier) - Make it easier to tune values when testing

  //Right Side
  @Config
  public void setRightPID(double kP, double kI, double kD, double kF){
    rightPID.setPID(kP, kI, kD, kF);
  }
  @Config
  public void setRightPulses(double distancePerPulse){
    rightEncoder.setDistancePerPulse(distancePerPulse);
  }
  @Log
  public int getRightEncoder(){
    return rightEncoder.getRaw();
  }

  //Left Side
  @Config
  public void setLeftPID(double kP, double kI, double kD, double kF){
    leftPID.setPID(kP, kI, kD, kF);
  }
  @Config
  public void setLeftPulses(double distancePerPulse){
    leftEncoder.setDistancePerPulse(distancePerPulse);
  }
  @Log
  public int getLeftEncoder(){
    return leftEncoder.getRaw();
  }

    
  //USED IN TELEOP-PERIODIC
  public void teleopDrive(double frontSpeed, double sideSpeed){
    //prevent controller drift
    if(Math.abs(frontSpeed) <= 0.08){
      frontSpeed = 0;
    }

    if(Math.abs(sideSpeed) <= 0.08){
      sideSpeed = 0;
    }
    
    //Slow down movement to 1/4 when Right Bumper is pressed
    if(Robot.oi.controller.getBumper(GenericHID.Hand.kRight)){
      drive.curvatureDrive(frontSpeed/4, sideSpeed/4, Robot.oi.controller.getBumper(GenericHID.Hand.kRight));
    }

    //Otherwise use Right Trigger (weirdly acts as a button on the controller instead of an axis) -> Port 8
    else{
      drive.curvatureDrive(frontSpeed/1.25, sideSpeed/1.25, Robot.oi.controller.getRawButton(8));
    }
  }

  //USED IN AUTO
  public void setAutonDrive(double setpoint, double leftMotionFeedForward, double rightMotionFeedForward){
    currentTime = Timer.getFPGATimestamp();
    leftPID.setSetpoint(setpoint);
    rightPID.setSetpoint(setpoint);
    //calculate dT if its not the first loop, otherwise just use 0.02 (50Hz)
    leftMotor.set(leftPID.calculate(leftEncoder.getRaw(), prevTime == Double.NaN ? 0.02 : currentTime-prevTime) + leftMotionFeedForward );
    rightMotor.set(rightPID.calculate(rightEncoder.getRaw(), prevTime == Double.NaN ? 0.02 : currentTime-prevTime) + rightMotionFeedForward );
    prevTime = currentTime;

  }
  




  @Override
  public void initDefaultCommand() {
    // Default -> TeleopDriveCommand so its always running
    setDefaultCommand(new teleopDriveCommand());
  }
}