// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorIDConstants;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


//Creates DriveSubsystem class
public class DriveSubsystem extends SubsystemBase {
  
  //Creates motor objects from WPI_TalonFX
  private WPI_TalonFX leftback = new WPI_TalonFX(MotorIDConstants.leftBackDrive);
  private WPI_TalonFX leftfront = new WPI_TalonFX(MotorIDConstants.leftFrontDrive);
  private WPI_TalonFX rightback = new WPI_TalonFX(MotorIDConstants.rightBackDrive);
  private WPI_TalonFX rightfront = new WPI_TalonFX(MotorIDConstants.rightFrontDrive);
  
  //Creates left and right MotorControllerGroups
  private MotorControllerGroup left = new MotorControllerGroup(leftback, leftfront);
  private MotorControllerGroup right = new MotorControllerGroup(rightback, rightfront);
  
  //Creates diffDrive from DifferentialDrive for left and right MotorControllerGroups
  private DifferentialDrive diffDrive = new DifferentialDrive(left, right);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27));

  //private AHRS gyro = new AHRS();
  //private AHRS navX = new AHRS(SerialPort.Port.kMXP);
  
  //Inverts right MotorControllerGroup
  public DriveSubsystem() {
    right.setInverted(true);
  }

  //Assigns arcadeDrive speed and rotation
  public void drive(double speed, double rotation){
    //System.out.println("leftfront" + leftfront.get());
    //System.out.println("leftback" + leftback.get());
    //System.out.println("rightfront" + rightfront.get());
    //System.out.println("rightback" + rightback.get());
    diffDrive.arcadeDrive(speed, -rotation);
  }

  public double encoderDifference() {
    //System.out.println(leftfront.getSelectedSensorPosition() - rightfront.getSelectedSensorPosition());
    return leftfront.getSelectedSensorPosition() - rightfront.getSelectedSensorPosition();
  }

  //sets arcadeDrive to 0 rotation and 0 speed
  public void stop(){
    diffDrive.arcadeDrive(0, 0);
  }

  // returns average of leftfront and rightfront motor positions
  public double getEncoderPosRAW() {
    return (leftfront.getSelectedSensorPosition() + rightfront.getSelectedSensorPosition()) / 2;
  }

  // returns the difference between the leftfront and rightfront motor positions
  public double getEncoderDiffernce() {
    return leftfront.getSelectedSensorPosition() - rightfront.getSelectedSensorPosition();
  }

  // prints and returns distance driven
  public double getEncoderDistanceMeters() {
    double dist = leftfront.getSelectedSensorPosition() / 4096 * 2 * Math.PI * DriveConstants.wheelRadiusMeters; //* 2*Math.PI*DriveConstants.wheelRadiusMeters);
    System.out.println(dist);
    return dist;
  }

  /**
   * Calculates velocity of robot based on information from wheel encoders
   * @return m/s speed of robot
   */
  public double getRobotVelocityEncoders() {
    double STUVelocity = (leftfront.getSelectedSensorVelocity() + rightfront.getSelectedSensorVelocity()) / 2;
    double rotationsPerSecond = (STUVelocity / 4096) * 10;
    return rotationsPerSecond * DriveConstants.wheelRadiusMeters;
  }

  // resets position
  public void resetEncoders() {
    leftfront.setSelectedSensorPosition(0);
    rightfront.setSelectedSensorPosition(0);
  }

  // resets gyro rotation 
  public void resetGyro() {
    //navX.reset();
  }

  
  public double gyroAngleDegrees() {
    //return navX.getAngle();
    return 0;
  }

  public double gyroPitchDegrees() {
    return 0;
  }

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.DriveStraightPIDConstants.kP, Constants.DriveStraightPIDConstants.kD, Constants.DriveStraightPIDConstants.kI),
            this.kinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            this::outputVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
  }

  // Overrides code
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
