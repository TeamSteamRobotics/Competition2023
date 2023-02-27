// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.BalancePID;
import frc.robot.commands.DriveCommands.EncoderDriveDistance;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto5 extends SequentialCommandGroup {
  /** Creates a new Auto5. */
  //Drives backwards outside of community, then drives forwards and docks.
  public Auto5(DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new EncoderDriveDistance(-3.5, drive),
    new EncoderDriveDistance(1.3, drive)
    
    );
  }
}
