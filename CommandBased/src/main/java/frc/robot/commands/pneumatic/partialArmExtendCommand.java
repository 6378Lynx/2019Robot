/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Author: Abdur Javaid                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.pneumatic;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Partially extends the arm - Can be used higher than ~30deg
 */
public class partialArmExtendCommand extends InstantCommand {

  public partialArmExtendCommand() {
    super();
    // Use requires() here to declare subsystem dependencies
    requires(Robot.pneumaticSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.pneumaticSubsystem.partialArmExtend();
  }

}
