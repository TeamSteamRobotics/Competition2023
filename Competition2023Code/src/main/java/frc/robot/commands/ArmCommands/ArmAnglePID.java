// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.security.AlgorithmConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html



public class ArmAnglePID extends PIDCommand {
  /** Creates a new ArmAnglePID. */
  
  public ArmAnglePID(ArmSubsystem arm, double angle) {
    super(
        // The controller that the command will use
        new PIDController(ArmConstants.angle_kP, ArmConstants.angle_kI, ArmConstants.angle_kD),
        // This should return the measurement
        () -> arm.getArmAngleDegrees(), //also increments index
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        output -> {
          arm.getArmAngleDegrees();
          arm.setArmSpeed(output);
          // Use the output here
        });
    addRequirements(arm);
    this.getController().setTolerance(ArmConstants.anglePIDTolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }
}