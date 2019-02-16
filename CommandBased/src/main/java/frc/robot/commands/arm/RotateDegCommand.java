/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import com.team254.lib.util.motion.*;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;



public class RotateDegCommand extends Command {
  
  private final MotionProfile profile;
  private double setPoint;

  public RotateDegCommand(double setPosition)
  {
      super(5);
      requires(Robot.shoulderSubsystem);
      MotionProfileConstraints constraints = new MotionProfileConstraints(RobotMap.maxVel, RobotMap.maxAccel);

      setPoint = setPosition;

      profile = MotionProfileGenerator.generateProfile(
                    constraints, 
                    new MotionProfileGoal(setPosition),
                    new MotionState(0,Robot.shoulderSubsystem.getPos(),0,0));

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double t = timeSinceInitialized();
    Robot.shoulderSubsystem.setSetpoint(setPoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    double t = timeSinceInitialized();
    return profile.stateByTimeClamped(t).coincident(profile.endState());

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
