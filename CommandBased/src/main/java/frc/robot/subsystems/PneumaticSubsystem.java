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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Add your docs here.
 */
public class PneumaticSubsystem extends Subsystem implements Loggable{
  //Initialize
  private DoubleSolenoid phaseOne = new DoubleSolenoid(0,RobotMap.pistonOne_1, RobotMap.pistonOne_2);
  private DoubleSolenoid phaseTwo = new DoubleSolenoid(0,RobotMap.pistonTwo_1, RobotMap.pistonTwo_2);
  @Log
  private DoubleSolenoid claw   = new DoubleSolenoid(0,RobotMap.pistonThree_1, RobotMap.pistonThree_2);
  //private DoubleSolenoid discBrake  = new DoubleSolenoid(1,RobotMap.discBrake_1, RobotMap.discBrake_2);
  //private DoubleSolenoid frontClimb = new DoubleSolenoid(1, RobotMap.front_1, RobotMap.front_2);
  //private DoubleSolenoid backClimb  = new DoubleSolenoid(1, RobotMap.back_1,RobotMap.back_2);
  private AnalogInput pressureSensor = new AnalogInput(0);

  Compressor compressor = new Compressor(1);

  public enum ArmState {retracted, partial, extended}

  private ArmState state;

  /**Sets the arm to desired state
  **@param state the desired state of the arm
   */
  public void actuateArm(ArmState state){
    double currentAngle = Robot.shoulderSubsystem.getPos();

    if(state == ArmState.retracted){
      phaseOne.set(DoubleSolenoid.Value.kReverse);
      phaseTwo.set(DoubleSolenoid.Value.kReverse);
      this.state = ArmState.retracted;
    }
    else if(state == ArmState.partial && currentAngle > 40){
      phaseOne.set(DoubleSolenoid.Value.kForward);
      phaseTwo.set(DoubleSolenoid.Value.kReverse);
      this.state = ArmState.partial;
    }
    else if(state == ArmState.extended && currentAngle > 60){
      phaseOne.set(DoubleSolenoid.Value.kForward);
      phaseTwo.set(DoubleSolenoid.Value.kForward);
      this.state = ArmState.extended;
    }
  }

  public ArmState getArmState(){
    return state;
  }


  //Activate the claw
  public void activateClaw(){
    //Retract if activated
    if(claw.get() ==  DoubleSolenoid.Value.kReverse){
    claw.set(DoubleSolenoid.Value.kForward);
  }
    else{
      claw.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void engageDiscBrake(){
    //discBrake.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void disengageDiscBrake(){
    //discBrake.set(DoubleSolenoid.Value.kForward);
  }

  public void retractArm(){
    phaseOne.set(DoubleSolenoid.Value.kReverse);
    phaseTwo.set(DoubleSolenoid.Value.kReverse);    
  }

  @Log.Dial(max=120)
  public double getPressureSensor(){
    //add math later
    return (pressureSensor.getVoltage() - 0.49) * (120/2.25);
  }

  /*
  public void setFrontClimb(){
    //If the arm is already extended, retract the arm
    if(frontClimb.get() ==  DoubleSolenoid.Value.kForward){
      frontClimb.set(DoubleSolenoid.Value.kReverse);
    }
    //If the arm is off or retracted, extend the arm
    else{
      frontClimb.set(DoubleSolenoid.Value.kForward);
    }
  }
*/
  public void setBackClimb(){
    /*
    //If the arm is already extended, retract the arm
    if(backClimb.get() ==  DoubleSolenoid.Value.kForward){
        backClimb.set(DoubleSolenoid.Value.kReverse);
    }
    //If the arm is off or retracted, extend the arm
    else{
      backClimb.set(DoubleSolenoid.Value.kForward);
    }
    */
  }
  

  public Enum getPhaseOne(){
    return phaseOne.get();
  }
  public Enum getPhaseTwo(){
    return phaseTwo.get();
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
