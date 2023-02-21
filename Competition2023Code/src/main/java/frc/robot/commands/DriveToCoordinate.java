
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToCoordinate extends CommandBase {
  
  private AprilVisionSubsystem m_aprilVisionSubsystem;
  private DriveSubsystem m_driveSubsystem;
  public float targetDistance;
  public float secondTargetDistance;
  private boolean aprilTagVisible;
  public float robotSpeed;
  private float currentDistance;
  private float currentDistanceRotation;
  private boolean commandFinished;
  private boolean inverted;
  private float tolerance;
  private float targetRotation;
  private float robotX;
  enum Step {
    ONE,
    TWO,
    THREE,
    FOUR
  }
  Step currentStep;
  public DriveToCoordinate(AprilVisionSubsystem vision, DriveSubsystem drive, float speed, float distance, float secondDistance, boolean invertDistance) {
    
    addRequirements(vision, drive);
    m_aprilVisionSubsystem = vision;
    m_driveSubsystem = drive;
    robotSpeed = speed;
    commandFinished = false;
  }
  @Override
  public void initialize(){
  }
  @Override
  public void execute(){

  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
    commandFinished = true;
  }
  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}