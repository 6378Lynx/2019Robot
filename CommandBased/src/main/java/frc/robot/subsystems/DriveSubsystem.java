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
import frc.robot.RobotMap;
import frc.robot.commands.teleopDriveCommand;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.GenericHID;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem implements Loggable {
  Spark leftBack = new Spark(RobotMap.leftBackPort);
  Spark leftFront = new Spark(RobotMap.leftFrontPort);
  Spark rightBack = new Spark(RobotMap.rightBackPort);
  Spark rightFront = new Spark(RobotMap.rightFrontPort);

  SpeedControllerGroup leftMotor = new SpeedControllerGroup(leftBack, leftFront);
  SpeedControllerGroup rightMotor = new SpeedControllerGroup(rightBack, rightFront);

  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  double scalingFactor;

  public DriveSubsystem(){

  }

  @Config 
  public void setScalingFactor(double scalingFactor){
    this.scalingFactor = scalingFactor;
  }

  public void teleopDrive(double frontSpeed, double sideSpeed){
    //prevent controller drift
    if(Math.abs(frontSpeed) <= 0.08){
      frontSpeed = 0;
    }

    if(Math.abs(sideSpeed) <= 0.08){
      sideSpeed = 0;
    }
    //drive.arcadeDrive(frontSpeed, sideSpeed);

    //Slow down movement when right bumper is pressed
    if(Robot.oi.controller.getBumper(GenericHID.Hand.kRight)){
      drive.curvatureDrive(frontSpeed/scalingFactor, sideSpeed/scalingFactor, Robot.oi.controller.getBumper(GenericHID.Hand.kRight));
    }
    else{
      drive.curvatureDrive(frontSpeed/1.25, sideSpeed/1.25, Robot.oi.controller.getRawButton(8));
    }
     }

 
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleopDriveCommand());
  }
}