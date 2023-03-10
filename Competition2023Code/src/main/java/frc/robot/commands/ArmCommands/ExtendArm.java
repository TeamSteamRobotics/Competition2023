// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtensionSubsystem;

public class ExtendArm extends CommandBase {
  /** Creates a new ExtendArm. */

  ArmExtensionSubsystem armExtensionSubsystem;
  double speed;


  public ExtendArm(ArmExtensionSubsystem armExtensionSubsystem, double speed) {
    this.armExtensionSubsystem = armExtensionSubsystem;
    this.speed = speed;  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armExtensionSubsystem.armLengthMeters() < 0.2){
      armExtensionSubsystem.extendArm(-speed);
    } else {
      armExtensionSubsystem.extendArm(-speed * 2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armExtensionSubsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
