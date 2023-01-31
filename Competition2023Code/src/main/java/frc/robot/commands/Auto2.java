// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class Auto2 extends CommandBase {
  /** Creates a new AutoDriveBackwardsDock. */

  EncoderDriveDistance grimbloFimblo;
  DriveSubsystem m_driveSubsystem;
  //ArmSubsystem armSubsystem;
 //ArmSubsystem armSubsystem
  
  public Auto2(DriveSubsystem driveSubsystem) {
   
    this.m_driveSubsystem = driveSubsystem;
    //this.armSubsystem = armSubsystem;
    addRequirements(m_driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(-.3, 0);
  //armSubsystem.stopAll();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (grimbloFimblo.isFinished() == true)
      return true;
    else 
        return false;
  }
}
