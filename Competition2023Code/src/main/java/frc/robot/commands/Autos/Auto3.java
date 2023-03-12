// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands.ReverseIntake;
import frc.robot.commands.ArmCommands.PositionCommands.HighArmPosition;
import frc.robot.commands.DriveCommands.EncoderDriveDistance;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto3 extends SequentialCommandGroup {
  /** Creates a new AutoDriveForwardsScoreDriveBackwardsDock. */
  //Drives forwards, scores high, drives back and docks.
  public Auto3(DriveSubsystem drive, ArmSubsystem arm, PneumaticsSubsystem deployIntake, ArmExtensionSubsystem armExtension, IntakeSubsystem intake) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(

      new EncoderDriveDistance(5, drive),
      new ParallelCommandGroup(
        new HighArmPosition(armExtension, deployIntake, arm),
        new SequentialCommandGroup(
          new WaitCommand(.5),
          new ReverseIntake(intake)), 
        new SequentialCommandGroup(
          new WaitCommand(1),
          new EncoderDriveDistance(-2.33, drive))
        )

    );
  }
}
