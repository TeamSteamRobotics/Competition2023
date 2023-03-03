// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PositionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCommands.ArmAnglePID;
import frc.robot.commands.ArmCommands.DeployIntake;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.ExtendArmPID;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowArmPosition extends ParallelCommandGroup {
 
  /** Creates a new LowArmPosition. */
  public LowArmPosition(ArmExtensionSubsystem m_armExtensionSubsystem, PneumaticsSubsystem m_pneumaticsSubsystem, ArmSubsystem m_armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ArmSubsystem.setGoingLow(true);
    addCommands(
      new SequentialCommandGroup(
        new WaitCommand(2),
        new ArmAnglePID(m_armSubsystem, ArmConstants.lowPosition)),
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new ExtendArm(m_armExtensionSubsystem, -0.2), 
          new WaitCommand(1)),
        new WaitCommand(1),
        new DeployIntake(m_pneumaticsSubsystem),
        new WaitCommand(0.5),
        new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.lowPositionLength))
    );
    
  }
}
