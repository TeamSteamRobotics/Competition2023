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

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */


  CANSparkMax armMotors = new CANSparkMax(5, null);

  CANSparkMax elevatorMotor1 = new CANSparkMax(7, null);
  CANSparkMax elevatorMotor2 = new CANSparkMax(8, null);
  CANSparkMax intakeMotor = new CANSparkMax(9, null);
  MotorControllerGroup elevatorMotors = new MotorControllerGroup(elevatorMotor1, elevatorMotor2);

  RelativeEncoder angleEncoders = armMotors.getEncoder();

  RelativeEncoder encoder = elevatorMotor1.getEncoder();
  RelativeEncoder encoder2 = elevatorMotor2.getEncoder();

  public double armLengthMeters() {
    return (encoder.getPosition()+encoder2.getPosition()) / 2 * ArmConstants.armConversionFactor;
  }

  public double armAngleDegrees() {
    return (angleEncoders.getPosition() * 2 * 180);
  }

  public void rotateArm(double speed){
    armMotors.set(speed);
  }

  public void extendArm(double speed){
    elevatorMotors.set(speed);
  }

  public void stopArm(){
    armMotors.set(0);
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
    armMotors.set(0);
  }




  public void intake(Double speed){
    intakeMotor.set(speed);
  }

  public ArmSubsystem() {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}