// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.GyroDriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GyroDrive extends PIDCommand {
  /** Creates a new GyroDrive. */
  public GyroDrive(DriveSubsystem drive, double distance) {
    super(
        // The controller that the command will use
        new PIDController(GyroDriveConstants.kP, GyroDriveConstants.kI, GyroDriveConstants.kD),
        // This should return the measurement
        () -> drive.gyroDistance(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          drive.drive(output / 2, 0);
          // Use the output here
        });
        addRequirements(drive);
        this.getController().setTolerance(GyroDriveConstants.tolerance);
        //this.getController().setIntegratorRange(0, 1);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }
}
