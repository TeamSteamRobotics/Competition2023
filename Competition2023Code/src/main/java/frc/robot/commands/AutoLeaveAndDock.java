// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLeaveAndDock extends CommandBase {
  /** Creates a new AutoLeaveAndDock. */
  SequentialCommandGroup FimblyShimbly;
  DriveSubsystem m_driveSubsystem;
  ArmSubsystem armSubsystem;
  
  public AutoLeaveAndDock(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
  
    this.m_driveSubsystem = driveSubsystem;
    this.armSubsystem = armSubsystem;
    addRequirements(m_driveSubsystem, armSubsystem);
   // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  FimblyShimbly = new SequentialCommandGroup(
    new EncoderDriveDistance(2, m_driveSubsystem),
    new EncoderDriveDistance(-4, m_driveSubsystem));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.stopAll();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
