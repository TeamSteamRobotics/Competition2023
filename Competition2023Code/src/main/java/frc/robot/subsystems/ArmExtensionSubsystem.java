// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorIDConstants;

public class ArmExtensionSubsystem extends SubsystemBase {
  /** Creates a new ArmExtensionSubsystem. */
  private CANSparkMax elevatorMotor = new CANSparkMax(MotorIDConstants.elevatorMotor, MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private long nowTime = 0;

  public ArmExtensionSubsystem() {
    //elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //elevatorEncoder.setPosition(0);
  }

  public double armLengthMeters() {
    //System.out.println(elevatorEncoder.getPosition());
    double distance = (-1 * (elevatorEncoder.getPosition()) * ArmConstants.armConversionFactor);
    return distance;
  }
  
  public void resetExtensionLength(){
    long initTime = System.currentTimeMillis();
    while(nowTime < 250) {
      this.extendArm(.3);
      nowTime = System.currentTimeMillis() - initTime;
    }
  }

  public void resetEncoder() {
    elevatorEncoder.setPosition(0);
  }

  //Creates stopElevator method
  public void stopElevator(){
    elevatorMotor.set(0);
  }

  //extends arm by setting elevator speed
  public void extendArm(double speed){
    elevatorMotor.set(speed);
    //System.out.println(armLengthMeters());
  }

  @Override
  public void periodic() {
    System.out.println("Arm Length: " + armLengthMeters());

    // This method will be called once per scheduler run
  }
}
