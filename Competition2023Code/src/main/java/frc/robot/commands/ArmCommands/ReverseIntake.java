// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntake extends CommandBase {
  // Creates a new ReverseIntake. 
  IntakeSubsystem m_intakeSubsystem;
  double speed;
  public ReverseIntake(IntakeSubsystem m_intakeSubsystem, double speed) {
    this.speed = speed;
    this.m_intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ReverseIntake(IntakeSubsystem m_intakeSubsystem) {
    speed = ArmConstants.intakeSpeed;
    this.m_intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.intake(speed * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
