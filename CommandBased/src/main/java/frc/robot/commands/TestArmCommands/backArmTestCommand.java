/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.TestArmCommands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.PneumaticCommands.engageDiscBrakeCommand;
import frc.robot.commands.TestArmCommands.testArm2;
import frc.robot.commands.PneumaticCommands.disengageDiscBrakeCommand;

public class backArmTestCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public backArmTestCommand() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    addSequential(new disengageDiscBrakeCommand());
    addSequential(new testArm2());
    addSequential(new engageDiscBrakeCommand());

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
