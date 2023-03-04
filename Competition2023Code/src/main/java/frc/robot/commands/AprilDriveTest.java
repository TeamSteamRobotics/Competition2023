package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class AprilDriveTest extends CommandBase{

    //Objects for subsystems
    private AprilVisionSubsystem av;
    private DriveSubsystem ds;

    //Parameters from RobotContainer
    public double speed;
    public int id;
    public double offset;
    private double error = 5;
    //public double targetDistance;

    //Visibility and command finished booleans
    private boolean aprilTagVisible;
    private boolean commandFinished;

    //CURRENT VALUES
    private double x;
    private double z;
    private double hyp;
    private double angle;
    private double gyroAngle;

    //VALUES CAPTURED DURING STEP ONE
    private double zInitial;
    private double xInitial;

    private double targetAngle;
    private double capturedAngle;
    private double capturedHyp;
    AHRS ahrs;

    //Basically a fancier version of an integer(as in the enum can only have three possible values).
    enum Step{
        ONE,
        TWO,
        THREE,
        FOUR
    }
   Step stepCurrent;
    public AprilDriveTest(AprilVisionSubsystem aprilVisionSubsystem, DriveSubsystem driveSubsystem, double robotSpeed, int targetId, double targetOffset){
        //Set up variables
        av = aprilVisionSubsystem;
        ds = driveSubsystem;
        speed = robotSpeed;
        id = targetId;
        offset = targetOffset;

    }
    
    
    //Runs when the command is first run
    @Override
    public void initialize(){
        //Makes sure the command is ready to be run.
        stepCurrent = Step.ONE;
        commandFinished = false;
        ahrs = new AHRS(SPI.Port.kMXP);
    }


    //TODO PUT MORE COMMENTS!!
    //TODO liam's balls. **DO NOT REMOVE THIS IS ESSENTIAL TO OPERATION**

    //Runs while commandFinished = false
    @Override
    public void execute(){
        //Detects whether or not the targeted april tag is visible, WILL NOT RUN CODE IF THE APRILTAG IS NOT VISIBLE
        aprilTagVisible = av.getCoordinates(id, 0).aprilTagVisible;
        gyroAngle = ahrs.getYaw();
        if(aprilTagVisible){
        //Updates x and z values constantly(as long as apriltag is visible)
        x = av.getCoordinates(id, 0).x;
        //Z with offset(distance from target).
        z = av.getCoordinates(id, 0).z - offset;

        //because fuck sequential command groups
        switch(stepCurrent){
            case ONE:
                //Initial values are captured when it is first run(not put it in initial because the apriltag being seen is not guarenteed on startup).
                //Turns the robot by the amount captured in the "capturedAngle" variable(double).
                zInitial = av.getCoordinates(id, 0).z - offset;
                //xInitial only exists because of my OCD.
                xInitial = av.getCoordinates(id, 0).x;
                capturedAngle = Math.atan2(x, zInitial);
                //capturedHyp is basically useless :/
                capturedHyp = Math.sqrt(Math.pow(xInitial, 2) + Math.pow(zInitial, 2));
                targetAngle = gyroAngle + (capturedAngle * Math.signum(xInitial));
                stepCurrent = Step.TWO;
            break;
            case TWO:
            System.out.println(ahrs.getYaw());
            System.out.println(targetAngle);
            if (Math.abs(ahrs.getYaw() - targetAngle) > error) {
                System.out.println("GET YAW");
                //Rotate the robot at speed until within error
                ds.drive(0, 0.5);
              } else {
                System.out.println("TO STEP THREE");
                //Stop rotating
                stepCurrent = Step.THREE;
              }
            break;
            case THREE:
                
                //Gets current value for hypotenuse, NOTE: INCLUDES THE OFFSET FOR TARGET(1 meter).
                hyp = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));

                //If the current hypotenuse is greater than 0.5(the robot's distance from the target offset) then drive forward. Could also possibly use the current z value instead.
                //TODO: Make 0.5 value a variable(called tolerance?).
                if(hyp > offset){
                    ds.drive(-speed, 0);
                }else{
                    stepCurrent = Step.FOUR;
                }
                //alternative code:
                /*
                 * if(z > tolerance){ //Could also get raw z value and compare it to the offset instead of using tolerance.
                 *      ds.drive(-speed, 0);
                 * }else{
                 *      stepCurrent = Step.THREE;
                 * }
                 */
            break;
            case FOUR:
                    commandFinished = true;
            }
        }else{
            //PRINTS WHEN APRILTAG IS NOT VISIBLE
            System.out.println("APRILTAG NOT VISIBLE!");
        }
    



        
        
        //Some logging
        System.out.println("ANGLE: " + angle);
        System.out.println("HYPOTENUSE: " + hyp);
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
