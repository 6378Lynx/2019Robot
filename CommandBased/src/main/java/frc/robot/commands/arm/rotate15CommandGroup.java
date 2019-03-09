/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Author: Abdur Javaid                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.pneumatic.engageDiscBrakeCommand;

public class rotate15CommandGroup extends CommandGroup {
  /**
   * Rotates the arm to 15 degrees - To change degree rotation, change parameter in rotateDegCommand
   * Should retract both arm phases before rotating, otherwise breaks frame perimeter
   */
  public rotate15CommandGroup() {
    requires(Robot.pneumaticSubsystem);
    requires(Robot.shoulderSubsystem);

    addSequential(new rotateDegCommand(RobotMap.ticks15deg));
    addSequential(new engageDiscBrakeCommand());

  }
}
