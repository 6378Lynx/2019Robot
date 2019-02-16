/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.pneumatic;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Dusengages the disc brake
 */
public class disengageDiscBrakeCommand extends InstantCommand {


  public disengageDiscBrakeCommand() {
    super();
    requires(Robot.pneumaticSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.pneumaticSubsystem.disengageDiscBrake();
  }

}
