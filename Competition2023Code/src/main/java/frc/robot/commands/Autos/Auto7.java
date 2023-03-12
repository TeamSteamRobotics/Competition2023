// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands.Intake;
import frc.robot.commands.ArmCommands.ReverseIntake;
import frc.robot.commands.ArmCommands.PositionCommands.HighArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.LowArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.MiddleArmPosition;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.DriveCommands.EncoderDriveDistance;
import frc.robot.commands.DriveCommands.GyroTurn;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto7 extends SequentialCommandGroup {
  /** Creates a new Auto7. */
  //Drives forwards, scores, drives back (leaves community), picks up piece
  public Auto7(DriveSubsystem drive, ArmSubsystem arm, PneumaticsSubsystem deployIntake, ArmExtensionSubsystem armExtension, IntakeSubsystem intake, PneumaticsSubsystem pneumatics, ArmSubsystem armRotation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ReverseIntake(intake).raceWith(new WaitCommand(1)),
      new ParallelCommandGroup(
        new MiddleArmPosition(armExtension, pneumatics, armRotation),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new Intake(intake).raceWith(new WaitCommand(1))
        )
      ).raceWith(new WaitCommand(4)),
      
      new Drive(drive, () -> 0.5, () -> 0).raceWith(new WaitCommand(7)),
      new GyroTurn(drive, 180).raceWith(new WaitCommand(2)),
      new ParallelCommandGroup(
        new LowArmPosition(armExtension, pneumatics, arm),
        new Drive(drive, () -> 0.5, () -> 0),
        new ReverseIntake(intake)
        ).raceWith(new WaitCommand(2))
      );
    /* new EncoderDriveDistance(.3, drive),
    new ParallelCommandGroup(
     new HighArmPosition(armExtension, deployIntake, arm),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ReverseIntake(intake))),
    new WaitCommand(0.5),
    new ParallelCommandGroup(
      new SequentialCommandGroup(
        new LowArmPosition(armExtension, deployIntake, arm),
        new EncoderDriveDistance(-5.65, drive)),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new Intake(intake)))
    //intake/parallel drive command group

  

    //open intake
    //rotate arm and drive back
    //align with piece and close intake*/
    
  }
}
