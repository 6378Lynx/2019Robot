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
import frc.robot.commands.pneumatic.*;

public class rotateToAngleCommandGroup extends CommandGroup {
  /**
   * Rotates the arm to set degrees
   * Should retract both arm phases before rotating, otherwise breaks frame perimeter
   */
  public rotateToAngleCommandGroup(double angle, int stage) {
    addSequential(new retractArmCommand());
    addSequential(new rotateDegCommand(angle));
    if(stage == 1){
      addSequential(new partialArmExtendCommand());
    }
    else if(stage == 2) {
      addSequential(new fullArmExtendCommand());
    }
  }
}