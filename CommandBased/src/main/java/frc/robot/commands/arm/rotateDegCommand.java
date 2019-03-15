/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Author: Abdur Javaid                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import com.team254.lib.util.motion.*;

public class rotateDegCommand extends TimedCommand {

  private MotionProfile profile;
  private double angle;

  public rotateDegCommand(double angle) {
    // Timeout 5 seconds, rotating shouldnt last more than 5
    super(5);
    requires(Robot.shoulderSubsystem);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    MotionProfileConstraints constraints = new MotionProfileConstraints(RobotMap.armMaxVel, RobotMap.armMaxAccel);

    profile = MotionProfileGenerator.generateProfile(constraints, new MotionProfileGoal(angle),
        new MotionState(0, Robot.shoulderSubsystem.getPos(), 0, 0));

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double t = timeSinceInitialized();
    MotionState state = profile.stateByTimeClamped(t);
    Robot.shoulderSubsystem.setDegrees(state.pos(), RobotMap.arm_ka * state.acc() + RobotMap.arm_kv * state.vel());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    double t = timeSinceInitialized();
    return profile.stateByTimeClamped(t).coincident(profile.endState()) || this.isTimedOut();

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
