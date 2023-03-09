// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.Intake;
import frc.robot.commands.ArmCommands.PositionCommands.MiddleArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.ResetArmPosition;
import frc.robot.commands.DriveCommands.AprilCenterOnTarget;
import frc.robot.commands.DriveCommands.GyroDrive;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
//Uses vision systems to drive up to the april tag, scores, then drives backwards
public class Auto12 extends SequentialCommandGroup {
  /** Creates a new Auto12. */
  public Auto12(DriveSubsystem drive, ArmSubsystem armRotation, ArmExtensionSubsystem armExtension, PneumaticsSubsystem pneumatics, IntakeSubsystem intake, AprilVisionSubsystem aprilVision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //1.40716
    addCommands(
      new AprilCenterOnTarget(drive, aprilVision, 0, 1.15716),
      new MiddleArmPosition(armExtension, pneumatics, armRotation),
      new GyroDrive(drive, .25),
      new Intake(intake),
      new GyroDrive(drive, -.5),
      new ResetArmPosition(armExtension, pneumatics, armRotation),
      new GyroDrive(drive, -1.5)
    );
  }
}
