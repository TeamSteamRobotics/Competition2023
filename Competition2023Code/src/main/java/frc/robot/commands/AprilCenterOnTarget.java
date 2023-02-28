// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.Coordinate;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilCenterOnTarget extends SequentialCommandGroup {
  /** Creates a new AprilCenterOnTarget. */
  Coordinate targetCoordinate;
  DriveSubsystem m_driveSubsystem;
  AprilVisionSubsystem m_aprilVisionSubsystem;
  double theta1;
  double theta;
  double distance;
  double distanceWithOffset;
  double driveDistance;
  double theta2;

  public AprilCenterOnTarget(DriveSubsystem m_driveSubsystem, AprilVisionSubsystem m_aprilVisionSubsystem, int targetID, int distanceFromTarget) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_aprilVisionSubsystem = m_aprilVisionSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    targetCoordinate = m_aprilVisionSubsystem.getCoordinates(targetID, 0);
    distance = Math.sqrt((Math.pow(targetCoordinate.z, 2)) - (Math.pow(targetCoordinate.x, 2)));
    distanceWithOffset = distance - distanceFromTarget;
    theta1 = Math.atan2(distanceWithOffset, targetCoordinate.x);
    theta = targetCoordinate.rx - theta1;
    driveDistance = Math.sqrt((Math.pow(distanceWithOffset, 2)) + (Math.pow(targetCoordinate.x, 2)));
    theta2 = 90 - theta1;

    addCommands(
      //PID Commands may not end 
      //Check tolerance or make parallel
      new DriveAnglePID(m_driveSubsystem, theta),
      new WaitCommand(.5),
      new EncoderDriveDistance(driveDistance, m_driveSubsystem),
      new WaitCommand(.5),
      new DriveAnglePID(m_driveSubsystem, theta2)
    );
  }
}
