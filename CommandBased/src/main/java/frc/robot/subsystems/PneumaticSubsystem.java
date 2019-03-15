/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Author: Abdur Javaid                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Add your docs here.
 */
public class PneumaticSubsystem extends Subsystem implements Loggable{
  //Initialize
  @Log
  DoubleSolenoid phaseOne = new DoubleSolenoid(0,RobotMap.pistonOne_1, RobotMap.pistonOne_2);
  @Log
  DoubleSolenoid phaseTwo = new DoubleSolenoid(0,RobotMap.pistonTwo_1, RobotMap.pistonTwo_2);
  @Log
  DoubleSolenoid claw = new DoubleSolenoid(0,RobotMap.pistonThree_1, RobotMap.pistonThree_2);
  @Log
  DoubleSolenoid discBrake = new DoubleSolenoid(0,RobotMap.discBrake_1, RobotMap.discBrake_2);
  @Log
  Compressor compressor = new Compressor();

  //Extend the first phase of the arm
  public void partialArmExtend(){
    //If the arm is already extended, retract the arm
    if(phaseOne.get() ==  DoubleSolenoid.Value.kForward){
    phaseOne.set(DoubleSolenoid.Value.kReverse);
  }
    //If the arm is off or retracted, extend the arm
    else{
      phaseOne.set(DoubleSolenoid.Value.kForward);
    }
  }

  //Extend the second phase of the arm
  public void fullArmExtend(){
    //If the arm is partially extended, extend the second phase only
    if(phaseOne.get() ==  DoubleSolenoid.Value.kForward && phaseTwo.get() != DoubleSolenoid.Value.kForward){
    phaseTwo.set(DoubleSolenoid.Value.kForward);
  }
    //If neither phase one or phase two are extended, extend both
    else if(phaseOne.get() != DoubleSolenoid.Value.kForward && phaseTwo.get() != DoubleSolenoid.Value.kForward){
      phaseOne.set(DoubleSolenoid.Value.kForward);
      phaseTwo.set(DoubleSolenoid.Value.kForward);
    }
    //If both are extended(only case left), then retract both
    else{
      phaseTwo.set(DoubleSolenoid.Value.kReverse);
      phaseOne.set(DoubleSolenoid.Value.kReverse);
    }
  }
  
  //Activate the claw
  public void activateClaw(){
    //Retract if activated
    if(claw.get() ==  DoubleSolenoid.Value.kForward){
    claw.set(DoubleSolenoid.Value.kReverse);
  }
    else{
      claw.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void engageDiscBrake(){
    discBrake.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void disengageDiscBrake(){
    discBrake.set(DoubleSolenoid.Value.kForward);
  }

  public void retractArm(){
    phaseOne.set(DoubleSolenoid.Value.kReverse);
    phaseTwo.set(DoubleSolenoid.Value.kReverse);    
  }
    
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
