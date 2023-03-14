// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands.PositionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCommands.ArmAngleLowPID;
import frc.robot.commands.ArmCommands.ArmAnglePID;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.ExtendArmPID;
import frc.robot.commands.ArmCommands.RetractIntake;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetArmPosition extends ParallelCommandGroup {
  /** Creates a new ResetArmPosition. */
  public ResetArmPosition(ArmExtensionSubsystem m_armExtensionSubsystem, PneumaticsSubsystem m_pneumaticsSubsystem, ArmSubsystem m_armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_armSubsystem.setGoingLow(true);
    addCommands(
      new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.resetPositionLength),
      new SequentialCommandGroup(
        new WaitCommand(0.25),
        new RetractIntake(m_pneumaticsSubsystem)), 
      new SequentialCommandGroup(
        new WaitCommand(.75), 
        new ArmAngleLowPID(m_armSubsystem, ArmConstants.resetPosition))
    );
    
  }
}
