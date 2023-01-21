// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.EncoderDriveDistanceConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EncoderDriveDistance extends PIDCommand {
  /** Creates a new EncoderDriveDistance. */
  public EncoderDriveDistance(double distanceMeters, DriveSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(EncoderDriveDistanceConstants.kP, EncoderDriveDistanceConstants.kI, EncoderDriveDistanceConstants.kD),
        // This should return the measurement
        () -> drive.getEncoderDistanceMeters(),
        // This should return the setpoint (can also be a constant)
        distanceMeters,
        // This uses the output
        output -> {
          /*if(output > 0.2)
            drive.drive(.2, 0);
          else*/
            drive.drive(-output / 2, 0);
          // Use the output here
        });
      
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(.1,1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
