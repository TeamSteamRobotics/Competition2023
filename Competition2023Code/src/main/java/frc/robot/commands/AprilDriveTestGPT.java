package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AprilDriveTestGPT extends CommandBase{

    private AprilVisionSubsystem av;
    private DriveSubsystem ds;

    public int id;

    private boolean aprilTagVisible;
    private boolean commandFinished;

    double targetXDistance = 1.0; 
    double targetZDistance = 2.0; 

    double currentXDistance = 0.0;
    double currentZDistance = 0.0;

    double distanceThreshold = 0.1; 

    double forwardBackwardValue = 0.0;
    double maxForwardBackValue = 1.0;

    double rotationValue = 0.0;
    double maxRotationValue = 1.0;

    
    public AprilDriveTestGPT(AprilVisionSubsystem aprilVisionSubsystem, DriveSubsystem driveSubsystem, int targetId, double targetX, double targetZ, double threshold, double maxFb, double maxR){
        av = aprilVisionSubsystem;
        ds = driveSubsystem;
        id = targetId;
        targetXDistance = targetX;
        targetZDistance = targetZ;
        distanceThreshold = threshold;
        maxForwardBackValue = maxFb;
        maxRotationValue = maxR;
    }
    @Override
    public void initialize(){
        commandFinished = false;
    }
  
    @Override
    public void execute(){
        aprilTagVisible = av.getCoordinates(id, 0).aprilTagVisible;

        if(Math.abs(currentXDistance - targetXDistance) > distanceThreshold || Math.abs(currentZDistance - targetZDistance) > distanceThreshold && aprilTagVisible){
            commandFinished = false;
            currentXDistance = -av.getCoordinates(id, 0).x;
            currentZDistance = av.getCoordinates(id, 0).z;

            forwardBackwardValue = Math.max(-maxForwardBackValue, Math.min(maxForwardBackValue, (currentZDistance - targetZDistance)));
            rotationValue = Math.max(-maxRotationValue, Math.min(maxRotationValue, Math.atan2(-currentXDistance, currentZDistance)));

            ds.drive(forwardBackwardValue, rotationValue);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }else if(aprilTagVisible){
            commandFinished = true;
        }else{
            commandFinished = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
      commandFinished = true;

    }

    @Override
    public boolean isFinished() {
      return commandFinished;
    }
}


