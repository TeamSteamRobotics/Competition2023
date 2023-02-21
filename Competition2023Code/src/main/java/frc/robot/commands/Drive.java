// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class Drive extends CommandBase {
  /** Creates a new Drive. */

  DriveSubsystem m_driveSubsystem;
  DoubleSupplier driveSpeed;
  DoubleSupplier rotationSpeed;
  AprilVisionSubsystem m_aprilVisionSubsystem;

  public Drive(DriveSubsystem driveSubsystem ,AprilVisionSubsystem vision, DoubleSupplier driveSpeed, DoubleSupplier rotationSpeed) {

    m_driveSubsystem = driveSubsystem;
    m_aprilVisionSubsystem = vision;
    this.driveSpeed = driveSpeed;
    this.rotationSpeed = rotationSpeed;

    addRequirements(driveSubsystem, vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(driveSpeed.getAsDouble(), rotationSpeed.getAsDouble());
 /*  float currentDistance = m_aprilVisionSubsystem.getCoordinates(6, true).z;
    float currentDistanceRotation = m_aprilVisionSubsystem.getCoordinates(6, true).x;
    System.out.println("ANGLE 30: " + Math.toDegrees(Math.atan2(Math.abs(currentDistanceRotation), currentDistance * Math.sqrt(3))));
    System.out.println("ANGLE 60: " + Math.toDegrees(Math.atan2(currentDistance * Math.sqrt(3), Math.abs(currentDistanceRotation))));
    */  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
