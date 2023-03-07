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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


//Creates DriveSubsystem class
public class DriveSubsystem extends SubsystemBase {
  
  //Creates motor objects from WPI_TalonFX
  private WPI_TalonSRX leftback = new WPI_TalonSRX(MotorIDConstants.leftBackDrive);
  private WPI_VictorSPX leftfront = new WPI_VictorSPX(MotorIDConstants.leftFrontDrive);
  private WPI_TalonSRX rightback = new WPI_TalonSRX(MotorIDConstants.rightBackDrive);
  private WPI_VictorSPX rightfront = new WPI_VictorSPX(MotorIDConstants.rightFrontDrive);
  
  //Creates left and right MotorControllerGroups
  private MotorControllerGroup left = new MotorControllerGroup(leftback, leftfront);
  private MotorControllerGroup right = new MotorControllerGroup(rightback, rightfront);
  
  //Creates diffDrive from DifferentialDrive for left and right MotorControllerGroups
  private DifferentialDrive diffDrive = new DifferentialDrive(left, right);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27));
  private DifferentialDriveOdometry m_odometry;//

  //private AHRS m_gyro = new AHRS();
  //private AHRS navX = new AHRS();
  AHRS navX = new AHRS(SPI.Port.kMXP);
  //= new AHRS();
  //SerialPort.Port.kMXP

  //Inverts right MotorControllerGroup
  public DriveSubsystem() { 
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyroAngleDegrees()), getLeftEncoderDistance(), getRightEncoderDistance());
    right.setInverted(true);
    resetEncoders();
    resetGyro();
  }

  //Assigns arcadeDrive speed and rotation
  public void drive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, -rotation);
  }

  //sets arcadeDrive to 0 rotation and 0 speed
  public void stop(){
    diffDrive.arcadeDrive(0, 0);
  }

  public void diffDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public double encoderDifference() {
    return leftfront.getSelectedSensorPosition() - rightfront.getSelectedSensorPosition();
  }

  // returns average of leftfront and rightfront motor positions
  public double getEncoderPosRAW() {
    return (leftfront.getSelectedSensorPosition() + rightfront.getSelectedSensorPosition()) / 2;
  }

  // returns the difference between the leftfront and rightfront motor positions
  public double getEncoderDifference() {
    return leftfront.getSelectedSensorPosition() - rightfront.getSelectedSensorPosition();
  }

  public double getLeftEncoderDistance() {
    return leftfront.getSelectedSensorPosition() / 4096 * 2 * Math.PI * DriveConstants.wheelRadiusMeters; //* 2*Math.PI*DriveConstants.wheelRadiusMeters);
  }

  public double getRightEncoderDistance() {
    return rightfront.getSelectedSensorPosition() / 4096 * 2 * Math.PI * DriveConstants.wheelRadiusMeters; //* 2*Math.PI*DriveConstants.wheelRadiusMeters);
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftVelocity = leftfront.getSelectedSensorVelocity() / 4096 * 10 * DriveConstants.wheelRadiusMeters;
    double rightVelocity = rightfront.getSelectedSensorVelocity() / 4096 * 10 * DriveConstants.wheelRadiusMeters;
    return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
  }

  // resets position
  public void resetEncoders() {
    leftfront.setSelectedSensorPosition(0);
    rightfront.setSelectedSensorPosition(0);
  }

  // resets gyro rotation 
  public void resetGyro() {
    navX.reset();
  }

  public double gyroAngleDegrees() {
    return navX.getAngle();
  }

  public double gyroPitchDegrees() {
    return 0;
  }

  //0 degrees = 0 encoder difference
  //90 degrees = -5864
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        navX.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance(), pose);
  }

  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics; 
  }

  // Overrides code
  @Override
  public void periodic() {
    //System.out.println(getEncoderDifference());
    // This method will be called once per scheduler run
    m_odometry.update(
      navX.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
    System.out.println(navX.getAngle());
  }

}
