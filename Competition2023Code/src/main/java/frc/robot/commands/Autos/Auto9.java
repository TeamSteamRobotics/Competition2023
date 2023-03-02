// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.EncoderDriveDistance;
import frc.robot.commands.DriveCommands.GyroTurn;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto9 extends SequentialCommandGroup {
  /** Creates a new Auto9. */
  //Drive forwards scores, drives out of community, picks up piece, drives forwards and
  public Auto9(DriveSubsystem drive, ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new EncoderDriveDistance(.3, drive),
    new EncoderDriveDistance(-5.65, drive),
    new GyroTurn(0,drive)
    






    );
  }
}
