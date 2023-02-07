// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveRotationConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveRotationPID extends PIDCommand {
  /** Creates a new DriveRotationPID. */
  public DriveRotationPID(DriveSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(DriveRotationConstants.drive_kP, DriveRotationConstants.drive_kI, DriveRotationConstants.drive_kD),
        // This should return the measurement
        () -> drive.encoderDifference(),
        // This should return the setpoint (can also be a constant)
         0,
        // This uses the output
        output -> {
          drive.drive(0, output);
          // Use the output here
        });
        addRequirements(drive);
        getController().setTolerance(DriveRotationConstants.drive_tolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }
}
