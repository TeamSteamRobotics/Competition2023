// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmAnglePID;
import frc.robot.commands.EncoderDriveDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto10 extends SequentialCommandGroup {
  /** Creates a new Auto10. */
  public Auto10(DriveSubsystem drive, ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new EncoderDriveDistance(5, drive),
    new ArmAnglePID(arm, 90),

    //reverse intake

    new EncoderDriveDistance(5, drive),
    new ArmAnglePID(arm, 90),

    //intake
    
    new EncoderDriveDistance(5, drive)

    //balance PID
    
  


    );
  }
}
