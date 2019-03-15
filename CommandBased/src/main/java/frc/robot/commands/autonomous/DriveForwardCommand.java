/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import com.team254.lib.util.motion.*;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveForwardCommand extends Command {

  private MotionProfile leftProfile;
  private MotionProfile rightProfile;
  private double setPosition;

  public DriveForwardCommand(double setPosition) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
    this.setPosition = setPosition;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    MotionProfileConstraints leftConstraints = new MotionProfileConstraints(RobotMap.driveMaxVel, RobotMap.driveMaxAccel);
    MotionProfileConstraints rightConstraints = new MotionProfileConstraints(RobotMap.driveMaxVel, RobotMap.driveMaxAccel);

    leftProfile = MotionProfileGenerator.generateProfile(
      leftConstraints, 
      new MotionProfileGoal(setPosition),
      new MotionState(0,Robot.driveSubsystem.getLeftEncoder(),0,0));
   
    rightProfile = MotionProfileGenerator.generateProfile(
        rightConstraints, 
        new MotionProfileGoal(setPosition),
        new MotionState(0,Robot.driveSubsystem.getRightEncoder(),0,0));
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double t = timeSinceInitialized();
    MotionState leftState = leftProfile.stateByTimeClamped(t);
    MotionState rightState = leftProfile.stateByTimeClamped(t);

    double leftFeedForward = RobotMap.drive_ka * leftState.acc() + RobotMap.drive_kv * leftState.vel() + RobotMap.Vintercept;
    double rightFeedForward = RobotMap.drive_ka * rightState.acc() + RobotMap.drive_kv * rightState.vel() + RobotMap.Vintercept;

    Robot.driveSubsystem.setAutonDrive(setPosition, leftFeedForward, rightFeedForward, 0);    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double t = timeSinceInitialized();
    return leftProfile.stateByTimeClamped(t).coincident(leftProfile.endState()) && rightProfile.stateByTimeClamped(t).coincident(rightProfile.endState());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
