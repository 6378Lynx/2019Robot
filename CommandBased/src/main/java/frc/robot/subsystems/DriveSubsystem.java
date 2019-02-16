/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
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

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  Spark leftBack = new Spark(RobotMap.leftBackPort);
  Spark leftFront = new Spark(RobotMap.leftFrontPort);
  Spark rightBack = new Spark(RobotMap.rightBackPort);
  Spark rightFront = new Spark(RobotMap.rightFrontPort);

  SpeedControllerGroup leftMotor = new SpeedControllerGroup(leftBack, leftFront);
  SpeedControllerGroup rightMotor = new SpeedControllerGroup(rightBack, rightFront);

  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  public DriveSubsystem(){

  }

  public void teleopDrive(double frontSpeed, double sideSpeed){
    drive.arcadeDrive(frontSpeed, sideSpeed);
    drive.curvatureDrive(frontSpeed, sideSpeed, Robot.oi.controller.getBumper(GenericHID.Hand.kRight));
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleopDriveCommand());
  }
}
