/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.PneumaticSubsystem;

public class RotateToAngleCommandGroup extends CommandGroup {
  /**Retracts the arm if extended, and then rotates to the desired angle
   * @param angle the desired angle for the arm to rotate to
   */
  public RotateToAngleCommandGroup(double angle) {
    addSequential(new InstantCommand( () -> Robot.pneumaticSubsystem.actuateArm(PneumaticSubsystem.ArmState.retracted)));
    addSequential(new rotateDegCommand(angle));
  }
}
