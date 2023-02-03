
package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem;

public class DriveToApril extends CommandBase {
  
  private AprilVisionSubsystem m_aprilVisionSubsystem;
  private DriveSubsystem m_driveSubsystem;
  public float targetDistance;
  private boolean aprilTagVisible;
  public float robotSpeed;
  private float currentDistance;
  private boolean commandFinished;

  public DriveToApril(AprilVisionSubsystem vision, DriveSubsystem drive) {
    addRequirements(vision, drive);
    m_aprilVisionSubsystem = vision;
    m_driveSubsystem = drive;
    
    robotSpeed = 0.5f;
    targetDistance = 2.0f;

    currentDistance = 0.0f;

    commandFinished = false;
  }
  @Override
  public void execute(){
    currentDistance = m_aprilVisionSubsystem.getCoordinates().z;
    aprilTagVisible = m_aprilVisionSubsystem.getCoordinates().aprilTagVisible;
    if(aprilTagVisible){
      if(currentDistance > targetDistance){
      System.out.println(currentDistance);
      m_driveSubsystem.drive(robotSpeed, 0);
      commandFinished = false;
      }else{
        commandFinished = true;
      }
    }else{
      commandFinished = false;
      System.out.println("NO APRILTAGS VISIBLE");
    }
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