// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands.PositionCommands.HighArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.ResetArmPosition;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Dock extends SequentialCommandGroup {
  /** Creates a new Dock. */
  public Dock(DriveSubsystem drive, ArmExtensionSubsystem extension, PneumaticsSubsystem pneumatics, ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new HighArmPosition(extension, pneumatics, arm),
        new Drive(drive, () -> 0.5, () -> 0)
      ).raceWith(new WaitCommand(6.35)),
      new InstantCommand(() -> drive.setBrakeMode(true), drive)
    );
    
  }

}
