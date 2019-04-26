/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.pneumatic.disengageDiscBrakeCommand;
import frc.robot.commands.arm.CalibrateArmEncoderCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.PneumaticSubsystem;

public class CalibrationCommandGroup extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CalibrationCommandGroup() {
    addSequential(new disengageDiscBrakeCommand());
    addSequential(new InstantCommand( () -> Robot.pneumaticSubsystem.actuateArm(PneumaticSubsystem.ArmState.retracted)));
    addSequential(new CalibrateArmEncoderCommand());
  }
}
