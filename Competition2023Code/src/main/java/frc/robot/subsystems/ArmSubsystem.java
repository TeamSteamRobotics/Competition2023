// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  CANSparkMax elevatorMotors = new CANSparkMax(5, null);

  CANSparkMax pushyMotor1 = new CANSparkMax(7, null);
  CANSparkMax pushyMotor2 = new CANSparkMax(8, null);
  CANSparkMax intakeMotor = new CANSparkMax(9, null);
  MotorControllerGroup armMotors = new MotorControllerGroup(pushyMotor1, pushyMotor2);

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