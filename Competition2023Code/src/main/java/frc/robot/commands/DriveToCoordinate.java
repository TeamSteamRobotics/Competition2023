
package frc.robot.commands;

import java.util.Vector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.Coordinate;

public class DriveToCoordinate extends CommandBase {
  
  private AprilVisionSubsystem av;
  private DriveSubsystem ds;

  private Coordinate targetCoordinate;
  private Coordinate robotCoordinateInitial;
  private Coordinate robotCoordinateCurrent;

  private boolean aprilTagVisible;
  public double robotSpeed;
  private double targetAngle;
  private double initialDistance;
  private double initialAngle;
  private double currentDistance;
  private double currentRotation;
  private boolean commandFinished;

  enum Step {
    ONE,
    TWO,
  }
  Step currentStep;
  public DriveToCoordinate(AprilVisionSubsystem vision, DriveSubsystem drive, double speed, Coordinate targetCoordinate) {
    
    addRequirements(vision, drive);
    av = vision;
    ds = drive;
    robotSpeed = speed;
    commandFinished = false;
    this.targetCoordinate = targetCoordinate;
  }
  @Override
  public void initialize(){
    robotCoordinateInitial = av.getCoordinates(0, 2);
    initialAngle = robotCoordinateInitial.rx;
    initialDistance = robotCoordinateInitial.z;
    targetAngle = initialAngle - Math.atan2(robotCoordinateInitial.x, robotCoordinateInitial.z);
  }
  @Override
  public void execute(){
      robotCoordinateCurrent = av.getCoordinates(0, 2);
      aprilTagVisible = robotCoordinateCurrent.aprilTagVisible;
      if(aprilTagVisible){
        currentRotation = robotCoordinateCurrent.rx;
        currentDistance = robotCoordinateCurrent.z;

        System.out.println("CURRENT DISTANCE: " + distance(robotCoordinateCurrent, targetCoordinate));

        switch(currentStep){
          case ONE:
            turn();
            break;
          case TWO:
            forward();
            break; 
        }
      }
  }

  public double distance(Coordinate current, Coordinate target){
    double distance;
    distance = Math.sqrt(Math.pow(target.x - current.x, 2) + Math.pow(target.y - current.y, 2));
    return distance;
  }
  private void turn(){
    ds.drive(0, 0.35 * (currentRotation - targetAngle));
  }
  private void forward(){
    if(distance(robotCoordinateCurrent, targetCoordinate) > 0.5){
    ds.drive(-robotSpeed, 0.25 * (currentRotation - targetAngle));
    }else{
      commandFinished = true;
    }
  }
  @Override
  public void end(boolean interrupted) {
    ds.stop();
    commandFinished = true;
  }
  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}