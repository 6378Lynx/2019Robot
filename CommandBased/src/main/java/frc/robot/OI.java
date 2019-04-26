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
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.commands.arm.CalibrationCommandGroup;
import frc.robot.commands.arm.RotateToAngleCommandGroup;
import frc.robot.commands.arm.rotateDegCommand;
import frc.robot.commands.pneumatic.engageDiscBrakeCommand;

import static frc.robot.subsystems.PneumaticSubsystem.ArmState.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.

    public XboxController   controller  = new XboxController(RobotMap.controllerPort),
                            operatorController = new XboxController(RobotMap.operatorPort);


    OI() {
        Joystick joystick    = new Joystick(RobotMap.controllerPort),
                operator    = new Joystick(RobotMap.operatorPort);


        //DRIVER
        Button  driverA_Button = new JoystickButton(joystick, RobotMap.DRIVER_A_Button),
                driverB_Button = new JoystickButton(joystick, RobotMap.DRIVER_B_Button),
                driverY_Button = new JoystickButton(joystick, RobotMap.DRIVER_Y_Button),
                driverX_Button = new JoystickButton(joystick, RobotMap.DRIVER_X_Button),
                driverLeftBumper  = new JoystickButton(joystick, RobotMap.DRIVER_leftBumper),
                //driverStartButton = new JoystickButton(joystick, 10),

                //OPERATOR
                operatorStart_Button = new JoystickButton(operator, RobotMap.OPERATOR_startButton),
                operatorA_Button     = new JoystickButton(operator, RobotMap.OPERATOR_A_Button),
                operatorB_Button     = new JoystickButton(operator, RobotMap.OPERATOR_B_Button),
                operatorY_Button     = new JoystickButton(operator, RobotMap.OPERATOR_Y_Button),
                operatorX_Button     = new JoystickButton(operator, RobotMap.OPERATOR_X_Button),
                operatorLeftBumper  = new JoystickButton(operator, RobotMap.OPERATOR_leftBumper),
                operatorRightBumper  = new JoystickButton(operator, RobotMap.OPERATOR_rightBumper);



        driverLeftBumper.whenPressed(new InstantCommand(Robot.pneumaticSubsystem::activateClaw));
        //driverA_Button.whenPressed(new InstantCommand(Robot.pneumaticSubsystem::setFrontClimb));
        driverB_Button.whenPressed(new InstantCommand(Robot.pneumaticSubsystem::setBackClimb));
        driverX_Button.whenPressed(new engageDiscBrakeCommand());


        operatorStart_Button.whenPressed(new CalibrationCommandGroup());
        operatorA_Button.whenPressed(new RotateToAngleCommandGroup(65));
        operatorB_Button.whenPressed(new RotateToAngleCommandGroup(60));
        operatorX_Button.whenPressed(new RotateToAngleCommandGroup(21));
        operatorLeftBumper.whenPressed(new InstantCommand(() -> Robot.pneumaticSubsystem.actuateArm(partial)));
        operatorRightBumper.whenPressed(new InstantCommand(() -> Robot.pneumaticSubsystem.actuateArm(retracted)));
        operatorY_Button.whenPressed(new InstantCommand(() -> Robot.pneumaticSubsystem.actuateArm(extended)));
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
