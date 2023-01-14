// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  WPI_TalonSRX elevatorMotors = new WPI_TalonSRX(5);

  CANSparkMax pushyMotor1 = new CANSparkMax(7, null);
  CANSparkMax pushyMotor2 = new CANSparkMax(8, null);
  MotorControllerGroup armMotors = new MotorControllerGroup(pushyMotor1, pushyMotor2);

  public void rotateArm(double speed){
    armMotor.set(speed);
  }

  public void extendArm(double speed){
    elevatorMotors.set(speed);
  }

  public void stop(){
    armMotor.set(0);
    elevatorMotors.set(0);
  }

  public ArmSubsystem() {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}