// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmAnglePID;
import frc.robot.commands.ArmCommands.ReverseIntake;
import frc.robot.commands.DriveCommands.EncoderDriveDistance;
import frc.robot.commands.PositionCommands.HighArmPosition;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto4 extends SequentialCommandGroup {
  /** Creates a new AutoDriveForwardsScoreDriveBackwardsDock. */
  //Drives forwards and scores
  public Auto4(DriveSubsystem drive, ArmSubsystem arm, PneumaticsSubsystem deployIntake, ArmExtensionSubsystem armExtension, IntakeSubsystem intake) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(

      new EncoderDriveDistance(.3, drive),
      new HighArmPosition(armExtension, deployIntake, arm),
      new ReverseIntake(intake)

    );
  }
}
