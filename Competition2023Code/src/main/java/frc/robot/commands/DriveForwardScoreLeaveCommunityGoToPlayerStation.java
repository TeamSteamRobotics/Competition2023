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
public class DriveForwardScoreLeaveCommunityGoToPlayerStation extends SequentialCommandGroup {
  /** Creates a new DriveForwardScoreLeaveCommunityGoToPlayerStation. */
  public DriveForwardScoreLeaveCommunityGoToPlayerStation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new EncoderDriveDistance(5, null),
      // new ArmAnglePID(armSubsystem, 5),
      // new Intake(armSubsystem)
      new EncoderDriveDistance(-5, null),
      new EncoderDriveDistance(1, null),
      //                                            ^turn to 20 degrees but code mad for being dumb
      new EncoderDriveDistance(10, null)
      // turn -20 degrees toward player station

    );
  }
}
