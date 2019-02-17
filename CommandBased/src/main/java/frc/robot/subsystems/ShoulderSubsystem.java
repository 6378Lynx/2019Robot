/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.CheesyPID;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class ShoulderSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public  Spark        shoulder         = new Spark(RobotMap.shoulderPort);
  private Encoder      encoder          = new Encoder(RobotMap.encoderPortA, RobotMap.encoderPortB, false, Encoder.EncodingType.k4X);
  private DigitalInput topLimitSwitch   = new DigitalInput(RobotMap.limitSwitch1Port);
  private DigitalInput bottomLimitSwitch  = new DigitalInput(RobotMap.limitSwitch2Port);
  private double kP         = RobotMap.kP;
  private double kI         = RobotMap.kI;
  private double kD         = RobotMap.kD;
  private double kF         = RobotMap.kF;
  private double currentTime;
  private double prevTime   = Double.NaN;
  //adjust first and fourth 0's to tune PID
  //CheesyPID = SynchronousPIDF from 254, renamed because it was confusing
  //Change 0's to tune PID
  public CheesyPID pid = new CheesyPID(kP,kI,kD,kF);




  public ShoulderSubsystem(){
    pid.setInputRange(0, 4096);
    pid.setOutputRange(-1, 1);
  }

  //Feedforward Calculation, Added onto PID output
  //Uses the Centre Of Mass only when arm is retracted fully, as you only rotate when the arm is retracted otherwise you break frame perimeter
  private double feedforward_voltage(){
    //mgcos(theta)*centreOfMass / gearRatio / stallTorque, convert encoder ticks to angle using scaleFactor
    
    //Current Angle in Radians -> current encoder position * 16/66(gear Ratio) * 2pi / total encoder ticks
    double currentAngle = (encoder.getRaw()*RobotMap.gearRatio_2*Math.PI*2)/4096;
    //Mechanical Torque on Arm -> M*G * Centre Of Mass * cos(currentAngle)
    double kf = RobotMap.armMass * 9.8 * Math.cos(currentAngle) * RobotMap.centreOfMass;
    //Calculate Feedforward Voltage -> Mechanical Torque / Stall Torque / gear ratio
    kf = (kf * RobotMap.gearRatio) / RobotMap.stallTorque;
    //Versaplanetary Gearboxes aren't as reliable, so tune it to a voltage where it keeps the arm in place
    return kf*2;
  }

  public void setDegrees(double ticks){
    currentTime = Timer.getFPGATimestamp();
    pid.setSetpoint(ticks);
    //calculate dT if its not the first loop, otherwise just use 0.02 (50Hz)
    shoulder.set(pid.calculate(encoder.getRaw(), prevTime == Double.NaN ? 0.02 : currentTime-prevTime) + feedforward_voltage() );
    prevTime = currentTime;
  }

  public boolean getTopLimit(){
    return topLimitSwitch.get();
  }
  public boolean getBottomLimit(){
    return bottomLimitSwitch.get();
  }

  public void reset(){
    shoulder.set(0);
  }

  public void resetEncoder(){
    encoder.reset();
  }

  public double getVel(){
    return encoder.getRate();
  }

  public double getPos(){
    return encoder.getRaw();
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
