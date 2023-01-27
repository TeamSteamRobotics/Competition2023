// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto7 extends SequentialCommandGroup {
  /** Creates a new Auto7. */
  public Auto7(DriveSubsystem drive, ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new EncoderDriveDistance(5, drive),
    new ArmAnglePID(arm, 90),
    //Reverse Intake

    new EncoderDriveDistance(0, drive),
    new ArmAnglePID(arm, 90),

    //intake/parallel drive command group

    new EncoderDriveDistance(5, drive),
    new ArmAnglePID(arm, 90)

    //open intake
    //rotate arm and drive back
    //align with piece and close intake
    );
  }
}
