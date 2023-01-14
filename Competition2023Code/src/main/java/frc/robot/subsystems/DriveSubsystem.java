// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  WPI_TalonSRX leftback = new WPI_TalonSRX(0);
  WPI_TalonSRX rightback = new WPI_TalonSRX(1);
  WPI_TalonSRX leftfront = new WPI_TalonSRX(2);
  WPI_TalonSRX rightfront = new WPI_TalonSRX(3);

  MotorControllerGroup left = new MotorControllerGroup(leftback, leftfront);
  MotorControllerGroup right = new MotorControllerGroup(rightback, rightfront);

  DifferentialDrive diffDrive = new DifferentialDrive(left, right);


  public DriveSubsystem() {

    right.setInverted(true);

  }

  public void drive(double speed, double rotation){

    diffDrive.arcadeDrive(speed, rotation);
    

  }

  public void stop(){

    diffDrive.arcadeDrive(0, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
