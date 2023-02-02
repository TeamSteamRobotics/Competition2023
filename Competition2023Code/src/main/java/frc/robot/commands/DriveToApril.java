
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.EncoderDriveDistanceConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem;




public class DriveToApril {
  /** Creates a new EncoderDriveDistance. */
  public DriveToApril(DriveSubsystem drive, AprilVisionSubsystem vision) {
    while(vision.getCoordinates().x > 0.5){
      drive.drive(.2, 0);
    }
    /*super(
        // The controller that the command will use
        new PIDController(EncoderDriveDistanceConstants.kP, EncoderDriveDistanceConstants.kI, EncoderDriveDistanceConstants.kD),
        // This should return the measurement
        () -> drive.getEncoderDistanceMeters(),
        // This should return the setpoint (can also be a constant)
        vision.getCoordinates().z-1,
        // This uses the output
        output -> {
          /*if(output > 0.2)
            drive.drive(.2, 0);
          else

            drive.drive(-output / 2, 0);
          // Use the output here
    });
    */

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    System.out.println(vision.getCoordinates().z);
  }

 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

