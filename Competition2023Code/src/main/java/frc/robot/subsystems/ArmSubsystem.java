// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorIDConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkMax armMotor = new CANSparkMax(5, MotorType.kBrushless);

  private CANSparkMax elevatorMotorLeft = new CANSparkMax(MotorIDConstants.leftElevatorMotor, MotorType.kBrushless);
  private CANSparkMax elevatorMotorRight = new CANSparkMax(MotorIDConstants.rightElevatorMotor, MotorType.kBrushless);
  private MotorControllerGroup elevatorMotors = new MotorControllerGroup(elevatorMotorRight, elevatorMotorLeft);

  private CANSparkMax intakeMotor = new CANSparkMax(MotorIDConstants.intakeMotor, MotorType.kBrushless);

  private RelativeEncoder angleEncoder = armMotor.getEncoder();

  private RelativeEncoder elevatorEncoderRight = elevatorMotorRight.getEncoder();
  private RelativeEncoder elevatorEncoderLeft = elevatorMotorLeft.getEncoder();

  public ArmSubsystem() {

  }

  public void resetAngleEncoder() {
    angleEncoder.setPosition(0);
  }

  public void resetElevatorEncoders() {
    elevatorEncoderLeft.setPosition(0);
    elevatorEncoderRight.setPosition(0);
  }

  public double armLengthMeters() {
    return ((elevatorEncoderLeft.getPosition() + elevatorEncoderRight.getPosition()) / 2) * ArmConstants.armConversionFactor;
  }

  public double armAngleDegrees() {
    return (angleEncoder.getPosition() * 360);
  }

  public void angleArm(double speed){
    armMotor.set(speed);
  }

  public void extendArm(double speed){
    elevatorMotors.set(speed);
  }

  public void intake(double speed){
    intakeMotor.set(speed);
  }

  public void stopArm(){
    armMotor.set(0);
  }
  public void stopElevator(){
    elevatorMotors.set(0);
  }
  public void stopIntake(){
    intakeMotor.set(0);
  }
  
  public void stopAll(){
    elevatorMotors.set(0);
    intakeMotor.set(0);
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}