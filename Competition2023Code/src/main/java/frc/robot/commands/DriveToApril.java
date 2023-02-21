
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToApril extends CommandBase {
  
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
  public DriveToApril(AprilVisionSubsystem vision, DriveSubsystem drive, float speed, float distance, float secondDistance, boolean invertDistance) {
    
    addRequirements(vision, drive);
    m_aprilVisionSubsystem = vision;
    m_driveSubsystem = drive;
    robotSpeed = speed;
    targetDistance = distance;
    secondTargetDistance = secondDistance;
    inverted = invertDistance;
    tolerance = 0.5f;
    currentDistance = 0.0f;
    currentDistanceRotation = 0.0f;
    currentStep = Step.ONE;
    commandFinished = false;
  }
  @Override
  public void initialize(){
    targetRotation = m_aprilVisionSubsystem.getCoordinates(6, 0).x;
    currentStep = Step.ONE;
  }
  @Override
  public void execute(){
    aprilTagVisible = m_aprilVisionSubsystem.getCoordinates(6, 0).aprilTagVisible;
    if(aprilTagVisible){
    currentDistanceRotation = m_aprilVisionSubsystem.getCoordinates(6, 0).x;
    currentDistance = m_aprilVisionSubsystem.getCoordinates(6, 0).z;
    robotX = m_aprilVisionSubsystem.getCoordinates(6, 1).x;
    }
    Log();
   switch(currentStep){
      case ONE:
        doTurn();
      break;
      case TWO:
        doForward();
      break;

    }
  }
public void Log(){
   
      System.out.println("Current X/Rotation: " + m_aprilVisionSubsystem.getCoordinates(6, 0).x);
      //System.out.println("Current RX/Rotation of Apriltag in 3D space: " + m_aprilVisionSubsystem.getCoordinates(6).rx);
      System.out.println("Current Z/Distance: " + m_aprilVisionSubsystem.getCoordinates(6, 0).z);

  }
  private void doTurn(){
    if(aprilTagVisible && currentDistance > targetDistance){
        m_driveSubsystem.drive(-robotSpeed, 0.35 * -Math.signum(currentDistanceRotation));
      }else if(!aprilTagVisible && currentDistance > targetDistance){
        m_driveSubsystem.drive(0, 0.35 * Math.signum(currentDistanceRotation));
      }else{
        currentStep = Step.TWO;
      }
    }
  
  private void doForward(){
    if(aprilTagVisible){
      if(Math.abs(currentDistanceRotation) > 0.3){
        m_driveSubsystem.drive(0 , Math.signum(currentDistanceRotation) * 0.3);
      }else if(Math.abs(currentDistanceRotation) < 0.3 && currentDistance > secondTargetDistance){
        m_driveSubsystem.drive(-robotSpeed, 0);
      }else{
        commandFinished = true;
      }
    }
  }
//or
/*
 * if(currentDistance > secondTargetDistance){
 *  m_driveSubsytem.drive(-robotSpeed, currentDistanceRotation * 0.5);
 * }else{
 * commandFinished = true;
 * }
 */
  @Override
  public void end(boolean interrupted) {
    currentStep = Step.ONE;
    m_driveSubsystem.stop();
    commandFinished = true;
  }
  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}