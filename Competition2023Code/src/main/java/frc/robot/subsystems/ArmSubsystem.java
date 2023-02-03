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
  private CANSparkMax elevatorMotor = new CANSparkMax(5, MotorType.kBrushless);

  private CANSparkMax armMotorLeft = new CANSparkMax(MotorIDConstants.leftElevatorMotor, MotorType.kBrushless);
  private CANSparkMax armMotorRight = new CANSparkMax(MotorIDConstants.rightElevatorMotor, MotorType.kBrushless);
  private MotorControllerGroup armMotors = new MotorControllerGroup(armMotorLeft, armMotorRight);

  private CANSparkMax intakeMotor = new CANSparkMax(MotorIDConstants.intakeMotor, MotorType.kBrushless);

  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private RelativeEncoder angleEncoderRight = armMotorRight.getEncoder();
  private RelativeEncoder angleEncoderLeft = armMotorLeft.getEncoder();

  private Encoder armEncoder = new Encoder(5,5);

  //Another encoder will be placed, it is not on the motor controllers and it is on the rotate arm part

  public ArmSubsystem() {
    armMotorLeft.setInverted(true);
  }

  public void resetAngleEncoder() {
    angleEncoderRight.setPosition(0);
    angleEncoderLeft.setPosition(0);
  }

  public void resetElevatorEncoders() {
    elevatorEncoder.setPosition(0);
  }

  public double armLengthMeters() {
    return (elevatorEncoder.getPosition()) * ArmConstants.armConversionFactor;
  }

  public double armAngleDegrees() {
    return (angleEncoderRight.getPosition() * angleEncoderLeft.getPosition() * 180);
  }

  public double getArmAngleDegrees(){
    return armEncoder.getDistance();
  }

  public void angleArm(double speed){
    armMotors.set(speed);
  }

  public void extendArm(double speed){
    elevatorMotor.set(speed);
  }

  public void intake(double speed){
    intakeMotor.set(speed);
  }

  public void stopArm(){
    armMotors.set(0);
  }
  public void stopElevator(){
    elevatorMotor.set(0);
  }
  public void stopIntake(){
    intakeMotor.set(0);
  }
  
  public void stopAll(){
    elevatorMotor.set(0);
    intakeMotor.set(0);
    armMotors.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}