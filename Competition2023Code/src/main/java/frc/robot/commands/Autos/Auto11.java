// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands.Intake;
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
  public Auto11(DriveSubsystem drive, ArmExtensionSubsystem armExtention, ArmSubsystem armRotation, PneumaticsSubsystem pneumatics, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new EncoderDriveDistance(1, drive),

      new ParallelCommandGroup(
        new MiddleArmPosition(armExtention, pneumatics, armRotation),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new EncoderDriveDistance(.5, drive)),  
        new SequentialCommandGroup(
          new WaitCommand(1),
          new Intake(intake))), //3
      
      new WaitCommand(2),
      new ResetArmPosition(armExtention, pneumatics, armRotation), 
      new EncoderDriveDistance(-3, drive)
    );
  }
}
