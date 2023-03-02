// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands.ArmCommands;

<<<<<<<< HEAD:Competition2023Code/src/main/java/frc/robot/commands/ArmCommands/DeployIntake.java
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class DeployIntake extends CommandBase {
  PneumaticsSubsystem pneumatics;
  // Creates a new DeployIntake. 
  public DeployIntake(PneumaticsSubsystem pneumatics) {
    this.pneumatics = pneumatics;
    addRequirements(pneumatics);
========
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class RotateArm extends CommandBase {
  /** Creates a new RotateArm. */

  ArmSubsystem m_ArmSubsystem;
  double speed;

  public RotateArm(ArmSubsystem armSubsystem, double speed) {
  this.speed = speed;
  m_ArmSubsystem = armSubsystem;

>>>>>>>> april-carpetsquare:Competition2023Code/src/main/java/frc/robot/commands/RotateArm.java
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:Competition2023Code/src/main/java/frc/robot/commands/ArmCommands/DeployIntake.java
    pneumatics.deployIntake();
========
    m_ArmSubsystem.angleArm(speed);
>>>>>>>> april-carpetsquare:Competition2023Code/src/main/java/frc/robot/commands/RotateArm.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pneumatics.getIsIntake();
  }
}
