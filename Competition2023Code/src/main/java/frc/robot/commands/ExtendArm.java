// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArm extends CommandBase {
  /** Creates a new ExtendArm. */

  ArmSubsystem m_ArmSubsystem;
  DoubleSupplier speed;


  public ExtendArm(ArmSubsystem m_ArmSubsystem, DoubleSupplier speed) {
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.speed = speed;  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.extendArm(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
