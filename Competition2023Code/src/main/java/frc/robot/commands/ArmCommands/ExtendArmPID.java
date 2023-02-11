// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendArmPID extends PIDCommand {
  /** Creates a new ExtendArmPID. */
  ArmExtensionSubsystem arm;
  public ExtendArmPID(ArmExtensionSubsystem arm, double length) {
    super(
        // The controller that the command will use
        new PIDController(ArmConstants.length_kP, ArmConstants.length_kI, ArmConstants.length_kD),
        // This should return the measurement
        () -> arm.armLengthMeters(),
        // This should return the setpoint (can also be a constant)
        length,
        // This uses the output
        output -> {
          arm.extendArm(-output);
          // Use the output here
        });
        addRequirements(arm);
        
        getController().setTolerance(ArmConstants.lengthPIDTolerance);
      this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint() || arm.armLengthMeters() >= ArmConstants.maxArmLengthMeters-.05;
  }
}
