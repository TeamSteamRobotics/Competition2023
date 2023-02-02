
package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.EncoderDriveDistanceConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem;





public class DriveToApril extends CommandBase {
  /** Creates a new EncoderDriveDistance. */
  AprilVisionSubsystem m_aprilVisionSubsystem;
  boolean aprilTagVisible;
  float aprilTagZCoord;
  DriveSubsystem m_driveSubsystem;
  boolean stopCalled = false;
  float robotSpeed;
  public DriveToApril(DriveSubsystem drive, AprilVisionSubsystem vision) {
    m_aprilVisionSubsystem = vision;
    m_driveSubsystem = drive;
    stopCalled = false;
    robotSpeed = 0.1f;
    addRequirements(vision, drive);
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
  }
  private void aprilDrive(float speed){
    do{
      updateVisibility(); //calls update visibility function
      updateZCoordinate(); //calls update z coord function
      m_driveSubsystem.drive(speed, 0); // makes the robot go forward to its AprilTag
    }while(aprilTagVisible && aprilTagZCoord > 0.1);
    if(!aprilTagVisible && !stopCalled){ //Calls the function that gives the robot existential dread
      pauseUntilVisible();
    }
  }
  private void updateVisibility(){
    aprilTagVisible = m_aprilVisionSubsystem.getCoordinates().aprilTagVisible; //Updates visibility status
  }
  private void updateZCoordinate(){
    aprilTagZCoord = m_aprilVisionSubsystem.getCoordinates().z[0]; //Updates zCoord
  }
  //Allows the robot to pause when it feels nervous, which is caused when it can't see its AprilTag!
  private void pauseUntilVisible(){
    do{
      //Updates status of visibility
      updateVisibility();
      System.out.println("APRILTAG NOT VISIBLE, TASK PAUSED");
    }while(!aprilTagVisible || stopCalled); // Determines if the robot is having an intervention, or if it has found its AprilTag and is now overreacting.
    aprilDrive(0.1f); //The big guns
  }
  @Override
  public void initialize(){
    stopCalled = false;
    robotSpeed = 0.1f;
    System.out.println("i cried making this command i hope you are aware of this fact, NOW PROCEEDING");    
  }
  @Override 
  public void execute(){
    System.out.println(m_aprilVisionSubsystem.getCoordinates().z[0]);
    updateVisibility();
    if(aprilTagVisible){
    aprilDrive(0.1f);
    }else{
      pauseUntilVisible();
    }
  }
  @Override
  //To stop the robot when it inevitably turns evil
  public void end(boolean interrupted) {
    stopCalled = true;
    m_driveSubsystem.stop();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}

