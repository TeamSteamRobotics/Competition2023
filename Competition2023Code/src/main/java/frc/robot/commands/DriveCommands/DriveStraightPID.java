// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveStraightPIDConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraightPID extends PIDCommand {
  /** Creates a new DriveStraighPID. */
  public DriveStraightPID(DriveSubsystem drive, double speed) {
    super(
        // The controller that the command will use
        new PIDController(DriveStraightPIDConstants.kP, DriveStraightPIDConstants.kI, DriveStraightPIDConstants.kD),
        // This should return the measurement
        () -> drive.getEncoderDifference(),
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          drive.drive(speed, output);
          // Use the output here
        });
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }
}
