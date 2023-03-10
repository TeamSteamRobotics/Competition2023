// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalancePID extends PIDCommand {
  /** Creates a new BalancePID. */
  public BalancePID(DriveSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(Constants.BalanceConstants.kP, Constants.BalanceConstants.kI, Constants.BalanceConstants.kD),
        // This should return the measurement
        () -> drive.gyroPitchDegrees() / 360,
        // This should return the setpoint (can also be a constant)
        0,
        output -> {
          drive.drive(output, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    getController().setSetpoint(Constants.BalanceConstants.tolerance); 
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
