// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.GyroTurnConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GyroTurn extends PIDCommand {
  /** Creates a new GyroTurn. */
  public GyroTurn(double turnAngleDegrees, DriveSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(GyroTurnConstants.kP, GyroTurnConstants.kI, GyroTurnConstants.kD),
        // This should return the measurement
        () -> drive.gyroAngleDegrees(),
        // This should return the setpoint (can also be a constant)
        turnAngleDegrees,
        // This uses the output
        output -> {
          drive.drive(0, output);
        });
      addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(GyroTurnConstants.tolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
