
package frc.robot.commands.DriveCommands;


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
    System.out.println(targetCoordinate);
    currentStep = Step.ONE;
    robotCoordinateInitial = av.getCoordinates(0, 2);
    initialAngle = robotCoordinateInitial.rz;
    initialDistance = robotCoordinateInitial.z;
    targetAngle = initialAngle - Math.atan2(targetCoordinate.x - robotCoordinateInitial.x, targetCoordinate.z - robotCoordinateInitial.z);
  }
  @Override
  public void execute(){
    System.out.println(targetCoordinate.x);
      robotCoordinateCurrent = av.getCoordinates(0, 2);
      aprilTagVisible = robotCoordinateCurrent.aprilTagVisible;
      if(av.getCoordinates(0, 2).aprilTagVisible){
        currentRotation = robotCoordinateCurrent.rz;
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
    System.out.println("CURRENT ROTATION: " + currentRotation);
    System.out.println("TARGET ANGLE: " + targetAngle);

    System.out.println(targetAngle - currentRotation);
    System.out.println(currentRotation - targetAngle);
    if(Math.abs(targetAngle - currentRotation) > 2){
      ds.drive(0, -0.4);
      commandFinished = false;
    }else{
      commandFinished = true;
    }
  }
  private void forward(){
    if(distance(robotCoordinateCurrent, targetCoordinate) > 2){
    ds.drive(-robotSpeed, 0.25 * -Math.abs(currentRotation - targetAngle));
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