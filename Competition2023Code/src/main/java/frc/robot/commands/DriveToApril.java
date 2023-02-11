
package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
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
  private float currentDistanceRotation;
  private boolean commandFinished;
  private boolean inverted;

  public DriveToApril(AprilVisionSubsystem vision, DriveSubsystem drive, float speed, float distance, boolean invertDistance) {
    
    addRequirements(vision, drive);
    m_aprilVisionSubsystem = vision;
    m_driveSubsystem = drive;
    
    robotSpeed = speed;
    targetDistance = distance;
    inverted = invertDistance;

    currentDistance = 0.0f;
    currentDistanceRotation = 0.0f;

    commandFinished = false;
  }
  @Override
  public void execute(){
    currentDistanceRotation = m_aprilVisionSubsystem.getCoordinates(6).x;
    currentDistance = m_aprilVisionSubsystem.getCoordinates(6).z;
    aprilTagVisible = m_aprilVisionSubsystem.getCoordinates(6).aprilTagVisible;

    forwardTarget();
  }
  private void forwardTarget(){
    if(aprilTagVisible){
      System.out.println("Current Distance: " + currentDistance);
      System.out.println("Current Rotation: " + currentDistanceRotation);
      if(currentDistance > targetDistance && !inverted){
        if(currentDistanceRotation > 0.45){
          m_driveSubsystem.drive(-robotSpeed, 0.35f);
        }else if(currentDistanceRotation < -0.45){
          m_driveSubsystem.drive(-robotSpeed, -0.35f);
        }else{
          m_driveSubsystem.drive(-robotSpeed, 0.0f);
        }
      commandFinished = false;
      }else if(currentDistance < targetDistance && inverted){
        System.out.println("Current Distance: " + currentDistance);
        m_driveSubsystem.drive(robotSpeed, 0);
        commandFinished = false;
      }else{
        commandFinished = false;
      }
    }else{
      if(!aprilTagVisible){
      commandFinished = false;
      }
    }
    //Thanks chatgpt for saving my ass like forty times
  }
 /*  private void centerTarget(boolean finalCenter){
    if(aprilTagVisible && currentDistance > targetDistance - 0.5){
      System.out.println("Current Rotation: " + currentDistanceRotation);
      if(currentDistanceRotation > 1.5){
        m_driveSubsystem.drive(0, 0.5f);
        commandFinished = false;
      }else if(currentDistanceRotation < 1){
        m_driveSubsystem.drive(0, -0.5f);
        commandFinished = false;
      }else if(!finalCenter){
       forwardTarget();
      }else{
        forwardTarget();
      }
    }
  }
  */
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