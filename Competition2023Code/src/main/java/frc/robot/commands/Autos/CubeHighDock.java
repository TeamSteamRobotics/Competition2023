// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands.Intake;
import frc.robot.commands.ArmCommands.ReverseIntake;
import frc.robot.commands.ArmCommands.PositionCommands.HighArmPosition;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeHighDock extends SequentialCommandGroup {
  /** Creates a new CubeHighDock. */
  public CubeHighDock(DriveSubsystem drive, ArmSubsystem armRotation, ArmExtensionSubsystem armExtension, PneumaticsSubsystem pneumatics, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(      
    new ReverseIntake(intake).raceWith(new WaitCommand(1)),
    new ParallelCommandGroup(
      new HighArmPosition(armExtension, pneumatics, armRotation),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new Drive(drive, () -> -0.5, () -> 0).raceWith(new WaitCommand(1)),
        new Intake(intake).raceWith(new WaitCommand(1))
      )
    ).raceWith(new WaitCommand(4)),
    new ParallelCommandGroup(
      new HighArmPosition(armExtension, pneumatics, armRotation),
      new Drive(drive, () -> 0.5, () -> 0)).raceWith(new WaitCommand(6.35)).raceWith(new WaitCommand(6.35)),
      new InstantCommand(() -> drive.setBrakeMode(true), drive)
      );
  }
}
