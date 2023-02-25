package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AprilDriveTest extends CommandBase{

    //Objects for subsystems
    private AprilVisionSubsystem av;
    private DriveSubsystem ds;

    //Parameters from RobotContainer
    public double speed;

    //Visibility and command finished booleans
    private boolean aprilTagVisible;
    private boolean commandFinished;

    public int targetId = 2;
    //Filters and shit
    public int arraySize = 25;
    private double[] rotationFilter;
    private double rotationCaptured;
    public AprilDriveTest(AprilVisionSubsystem aprilVisionSubsystem, DriveSubsystem driveSubsystem, double robotSpeed){
        //Set up variables
        av = aprilVisionSubsystem;
        ds = driveSubsystem;
        speed = robotSpeed;


    }
    //Runs when the command is first run
    @Override
    public void initialize(){
        commandFinished = false;
        rotationFilter = new double[arraySize];
        initializeFilter(rotationFilter);
        //Make sure the filter is full
        for (int i = 0; i < arraySize; i++){
            rotationCaptured = av.getCoordinates(targetId, 0).rx;
            addToFilter(rotationFilter, rotationCaptured);
        }
        System.out.println("TEST");
        
    }
    //Runs while commandFinished = false

    //TODO put filter shit in seperate file??
    //TODO PUT MORE COMMENTS!!

    //filters
    @Override
    public void execute(){
        rotationCaptured = av.getCoordinates(targetId, 0).rx;
        addToFilter(rotationFilter, rotationCaptured);
        System.out.println("AVERAGE" + sumArray(rotationFilter));
    }
    public void initializeFilter(double[] filter){
        for (int i = 0; i < filter.length; i++) {
            filter[i] = 0.0;
        }
    }
    public void addToFilter(double[] filter, double value){
        for (int i = 0; i < filter.length - 1; i++) {
            filter[i] = filter[i + 1];
        }
        filter[filter.length - 1] = value;
    }
    public double sumArray(double[] filter){
        double average = 0;
        for (int i = 0; i < filter.length; i++) {
            average = average + filter[i];
        }
        average = average / filter.length;
        return average;
    }
    //Ends the command early if the enter key(or another interruption) is triggered
    @Override
    public void end(boolean interrupted) {
      commandFinished = true;

    }
    //Determines whether or not the command should continue running based on commandFinished
    @Override
    public boolean isFinished() {
      return commandFinished;
    }
}
