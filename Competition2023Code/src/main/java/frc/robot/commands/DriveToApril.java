
package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem;

import java.util.concurrent.CompletableFuture;

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

  enum Step {
    MAXTURN,
    FORWARD
  }
  Step currentStep;
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
  public void initialize(){
    currentStep = Step.MAXTURN;
  }
  @Override
  public void execute(){
    currentDistanceRotation = m_aprilVisionSubsystem.getCoordinates(6).x;
    currentDistance = m_aprilVisionSubsystem.getCoordinates(6).z;
    aprilTagVisible = m_aprilVisionSubsystem.getCoordinates(6).aprilTagVisible;
    asyncLog();

    switch(currentStep){
      case MAXTURN:
        maxTurn();
      break;
      case FORWARD:
        forward();
      break;

    }
  }
public void asyncLog(){
    CompletableFuture<Void> future = CompletableFuture.runAsync(() ->{
      System.out.println("Current X/Rotation: " + m_aprilVisionSubsystem.getCoordinates(6).x);
      //System.out.println("Current RX/Rotation of Apriltag in 3D space: " + m_aprilVisionSubsystem.getCoordinates(6).rx);
      System.out.println("Current Z/Distance: " + m_aprilVisionSubsystem.getCoordinates(6).z);
    });
  }

  private void maxTurn(){
    float lastSeenRotation = 0;
    float lastSeenDistance = 0;
    if(aprilTagVisible && currentDistance > 2.0f){
      lastSeenRotation = currentDistanceRotation;
      lastSeenDistance = currentDistance;
      System.out.println();
      m_driveSubsystem.drive(-robotSpeed, 0.35 * -Math.signum(currentDistanceRotation));
    }else if(!aprilTagVisible && lastSeenDistance > targetDistance){
      m_driveSubsystem.drive(0, 0.25 * Math.signum(lastSeenRotation));
    }else{
      currentStep = Step.FORWARD;
    }
  }
  private void forward(){
    if(aprilTagVisible){
      System.out.println("Current Distance: " + currentDistance);
      System.out.println("Current Rotation: " + currentDistanceRotation);
      if(currentDistance > targetDistance && !inverted){
        if(Math.abs(currentDistanceRotation) > 0.45){
          m_driveSubsystem.drive(0, Math.signum(currentDistanceRotation) * 0.35f);
        }else{
          m_driveSubsystem.drive(-robotSpeed, 0.0f);
        }
      commandFinished = false;
     }else if(currentDistance <= targetDistance){
      commandFinished = true;
      }
    }else if(!aprilTagVisible){
      commandFinished = true;
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