/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Author: Abdur Javaid                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.arm.rotateToAngleCommandGroup;
import frc.robot.commands.pneumatic.activateClawCommand;
import frc.robot.commands.pneumatic.partialArmExtendCommand;
import frc.robot.commands.pneumatic.fullArmExtendCommand;
import frc.robot.commands.test.backArmTestCommand;
import frc.robot.commands.test.fwdArmTestCommand;
import frc.robot.commands.test.stopArmTestCommand;



/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  
  public XboxController   controller  = new XboxController(RobotMap.controllerPort);
  
  public Joystick         joystick    = new Joystick(RobotMap.controllerPort),
                          operator    = new Joystick(RobotMap.operatorPort);

  Button             aButton     = new JoystickButton(joystick, RobotMap.XBOX_A_Button),
                          bButton     = new JoystickButton(joystick, RobotMap.XBOX_B_Button),
                          yButton     = new JoystickButton(joystick, RobotMap.XBOX_Y_Button),
                          xButton     = new JoystickButton(joystick, RobotMap.XBOX_X_Button),
                          leftBumper  = new JoystickButton(joystick, RobotMap.XBOX_leftBumper);

  public OI() {
    leftBumper.whenPressed(new activateClawCommand());
    aButton.whenPressed(new partialArmExtendCommand());
    bButton.whenPressed(new rotateToAngleCommandGroup(45,1));
  }


  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
