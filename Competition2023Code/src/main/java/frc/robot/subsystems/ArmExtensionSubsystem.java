// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmExtensionSubsystem extends SubsystemBase {
  /** Creates a new ArmExtensionSubsystem. */
  private CANSparkMax elevatorMotor = new CANSparkMax(6, MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  public ArmExtensionSubsystem() {
    elevatorEncoder.setPosition(0);
  }

  public double armLengthMeters() {
    return (-1 * (elevatorEncoder.getPosition()) * ArmConstants.armConversionFactor)+ArmConstants.extendArmPIDoffset;
  }

  //Creates stopElevator method
  public void stopElevator(){
    elevatorMotor.set(0);
  }

  //extends arm by setting elevator speed
  public void extendArm(double speed){
    elevatorMotor.set(speed);
    System.out.println(armLengthMeters());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
