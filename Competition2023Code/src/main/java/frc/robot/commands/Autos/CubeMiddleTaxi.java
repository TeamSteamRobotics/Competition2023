// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCommands.ArmAnglePID;
import frc.robot.commands.ArmCommands.Intake;
import frc.robot.commands.ArmCommands.ReverseIntake;
import frc.robot.commands.ArmCommands.PositionCommands.HighArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.LowArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.MiddleArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.ResetArmPosition;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.DriveCommands.EncoderDriveDistance;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto11 extends SequentialCommandGroup {
  /** Creates a new Auto11. */
  //Drives forwards, scores cube, drives back
  public Auto11(DriveSubsystem drive, ArmExtensionSubsystem armExtension, ArmSubsystem armRotation, PneumaticsSubsystem pneumatics, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //23 inches for middle
    addCommands(
      new ReverseIntake(intake).raceWith(new WaitCommand(1)),
      new ParallelCommandGroup(
        new MiddleArmPosition(armExtension, pneumatics, armRotation),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new Intake(intake).raceWith(new WaitCommand(1))
        )
      ).raceWith(new WaitCommand(4)),
      new Drive(drive, () -> 0.5, () -> 0).raceWith(new WaitCommand(7))


    /* 
      new ParallelRaceGroup(
        new ReverseIntake(intake),
        new WaitCommand(.5)),
      
      new ParallelRaceGroup(
        new WaitCommand(2),
        new ReverseIntake(intake, .1),
        new MiddleArmPosition(armExtention, pneumatics, armRotation)
      ),
      
      new ParallelRaceGroup(
        new Intake(intake), 
        new WaitCommand(.5)),

      new ResetArmPosition(armExtention, pneumatics, armRotation),

      new ParallelRaceGroup(
        new Drive(drive, () -> -0.25, () -> 0),
        new WaitCommand(1)
      )
    */

    /* 
      new ParallelRaceGroup(
        new WaitCommand(2),
        new ReverseIntake(intake, .1)),
      
      new ParallelCommandGroup(
        new MiddleArmPosition(armExtention, pneumatics, armRotation),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new EncoderDriveDistance(.5, drive)),  
        new SequentialCommandGroup(
          new WaitCommand(1),
          new Intake(intake))), //3
      
      new WaitCommand(2),
      new EncoderDriveDistance(-.5, drive),
      new ResetArmPosition(armExtention, pneumatics, armRotation), 
      new EncoderDriveDistance(-3, drive)
    */
    );
  }
}
