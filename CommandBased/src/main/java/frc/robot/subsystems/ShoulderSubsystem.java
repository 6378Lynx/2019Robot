/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Author: Abdur Javaid                                                       */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.CheesyPID;
import frc.robot.RobotMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class ShoulderSubsystem extends Subsystem implements Loggable {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Spark shoulder = new Spark(RobotMap.shoulderPort);
  private Encoder encoder = new Encoder(RobotMap.armEncoder_A, RobotMap.armEncoder_B, false, Encoder.EncodingType.k4X);

  private DigitalInput topLimitSwitch = new DigitalInput(RobotMap.limitSwitchTopPort);
  private DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.limitSwitchBotPort);

  private double kP;
  private double kI;
  private double kD;
  private double kF;

  private double prevTime = Double.NaN;
  private double currentTime;

  private double deltaV;
  private double deltaT;
  private double currentVel;
  private double lastVel = this.getVel();
  private double lastTime = Timer.getFPGATimestamp();

  private double angle;

  @Log
  private double accel;

  // adjust first and fourth 0's to tune PID
  // CheesyPID = SynchronousPIDF from 254, renamed because it was confusing
  // Change 0's to tune PID
  @Config.PIDController
  private CheesyPID pid = new CheesyPID(kP, kI, kD, kF);

  // CONSTRUCTOR -------------------------------------------------

  public ShoulderSubsystem() {
    encoder.setDistancePerPulse(RobotMap.armDistPerPulse);

    pid.setInputRange(0, 90);
    pid.setOutputRange(-1, 1);

  }

  // FEEDFORWARD CALCULATION -------------------------------------

  // Feedforward Calculation, Added onto PID output
  // Uses the Centre Of Mass only when arm is retracted fully, as you only rotate
  // when the arm is retracted otherwise you break frame perimeter
  private double feedforward_voltage() {
    // mgcos(theta)*centreOfMass / gearRatio / stallTorque, convert encoder ticks to
    // angle using scaleFactor
    // Current Angle in Radians -> current encoder position * 16/66(gear Ratio) *
    // 2pi / total encoder ticks

    // Mechanical Torque on Arm -> M*G * Centre Of Mass * cos(currentAngle)
    double kf = RobotMap.armMass * 9.8 * Math.cos(Math.toRadians(encoder.getDistance() + RobotMap.kDeg))
        * RobotMap.centreOfMass;
    // Calculate Feedforward Voltage -> Mechanical Torque / Stall Torque / gear
    // ratio
    kf = (kf * RobotMap.gearRatio) / RobotMap.stallTorque;
    return kf;
  }

  // SET DEGREES USING PID ----------------------------------------

  public void setDegrees(double angle, double motion_feedforward) {
    this.angle = angle;
    currentTime = Timer.getFPGATimestamp();
    pid.setSetpoint(angle);
    // calculate dT if its not the first loop, otherwise just use 0.02 (50Hz)
    shoulder.set(pid.calculate(this.getPos(), prevTime == Double.NaN ? 0.02 : currentTime - prevTime)
        + feedforward_voltage() + motion_feedforward);
    prevTime = currentTime;
  }

  public void holdPosition() {
    currentTime = Timer.getFPGATimestamp();
    pid.setSetpoint(this.angle);
    shoulder.set(pid.calculate(this.getPos(), prevTime == Double.NaN ? 0.02 : currentTime - prevTime + feedforward_voltage()));
    prevTime = currentTime;
  }

  // Setters - Getters
  public void setArm(double percent) {
    shoulder.set(percent);
  }

  @Log
  public boolean getTopLimit() {
    return topLimitSwitch.get();
  }

  @Log
  public boolean getBottomLimit() {
    return bottomLimitSwitch.get();
  }

  @Log
  public double getPos() {
    return encoder.getDistance() + RobotMap.kDeg;
  }

  @Log
  public double getVel() {
    return encoder.getRate();
  }

  public void reset() {
    shoulder.set(0);
  }

  public void resetEncoder() {
    encoder.reset();
  }

  @Override
  public void periodic() {
    currentVel = this.getVel();
    currentTime = Timer.getFPGATimestamp();

    deltaV = lastVel - currentVel;
    deltaT = lastTime - currentTime;
    accel = deltaV / deltaT;

    lastTime = currentTime;
    lastVel = currentVel;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
