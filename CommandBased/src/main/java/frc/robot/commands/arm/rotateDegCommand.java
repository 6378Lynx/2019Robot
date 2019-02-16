/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import com.team254.lib.util.motion.*;



public class rotateDegCommand extends TimedCommand {
  
  private final MotionProfile profile;

  public rotateDegCommand(double setPosition)
  {
      super(5);
      requires(Robot.shoulderSubsystem);
      MotionProfileConstraints constraints = new MotionProfileConstraints(RobotMap.maxVel, RobotMap.maxAccel);


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
    Robot.shoulderSubsystem.setDegrees(profile.stateByTimeClamped(t).pos());
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
    Robot.shoulderSubsystem.pid.reset();
    Robot.shoulderSubsystem.pid.resetIntegrator();
    Robot.shoulderSubsystem.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
