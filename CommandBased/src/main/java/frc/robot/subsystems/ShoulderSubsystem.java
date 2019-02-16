/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ShoulderSubsystem extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private  Spark       shoulderMotor;
  private Encoder      encoder;
  private DigitalInput fwdLimitSwitch;
  private DigitalInput backLimitSwitch;

  private double calibratedDegPerTick;


  public ShoulderSubsystem(){
    super("Arm", RobotMap.armKP, RobotMap.armKI, RobotMap.armKD, RobotMap.armKD, RobotMap.armKf);
    shoulderMotor = new Spark(RobotMap.shoulderPort);
    fwdLimitSwitch = new DigitalInput(RobotMap.limitSwitch1Port);
    backLimitSwitch = new DigitalInput(RobotMap.limitSwitch2Port);
    encoder = new Encoder(RobotMap.encoderPortA, RobotMap.encoderPortB, false, Encoder.EncodingType.k4X);

    //TODO: the best way for you to find degrees/tick is to calculate the degrees and read how many ticks you are at.
    //Deg/tick = Degrees/(CurrentTicks-startingTick)


    super.setOutputRange(-1, 1);
    super.setInputRange(  0, 4096);
    encoder.setDistancePerPulse(15/RobotMap.ticks15deg);
    encoder.setPIDSourceType(PIDSourceType.kDisplacement);

    getPIDController().setContinuous(false);

  }

  //TODO:WARNING THIS WILL CHANGE WHEN YOU ARM IS EXTENDED OR NOT!!!
  private double calcKF(){
    //mgcos(theta)*centreOfMass / gearRatio / stallTorque, convert encoder ticks to angle using scaleFactor
    //Current Angle in Radians -> current encoder position * 16/66(gear Ratio) * 2pi / total encoder ticks
    double currentAngle = (returnPIDInput()*RobotMap.gearRatio_2*Math.PI*2)/4096;
    //Mechanical Torque on Arm -> M*G * Centre Of Mass * cos(currentAngle)
    double kf = RobotMap.armMass * 9.8 * Math.cos(currentAngle) * RobotMap.centreOfMass;
    //Calculate Feedforward Voltage -> Mechanical Torque / Stall Torque / gear ratio
    kf = (kf * RobotMap.gearRatio) / RobotMap.stallTorque;
    return kf;
  }

  //TODO: you may need to reverse which switch is witch...
  public boolean isBottomLimit(){
    return fwdLimitSwitch.get();
  }
  public boolean isTopLimit(){
    return backLimitSwitch.get();
  }

  public double getVel(){
    return encoder.getRate();
  }

  public double getPos(){
    return encoder.getDistance();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

  @Override
  protected double returnPIDInput() {
    return encoder.getDistance();
  }

  @Override
  protected void usePIDOutput(double output) {
    shoulderMotor.pidWrite(output + calcKF());
  }
}
