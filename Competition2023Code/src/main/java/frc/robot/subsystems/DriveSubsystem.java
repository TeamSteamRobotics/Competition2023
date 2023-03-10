// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SerialPort;

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
  private DifferentialDriveOdometry m_odometry;//
  private boolean isSlow = false;

  AHRS navX = new AHRS(SPI.Port.kMXP);

  private boolean halfSpeed = false;
  private SlewRateLimiter rateLimitVelocity = new SlewRateLimiter(2);


  //Inverts right MotorControllerGroup
  public DriveSubsystem() { 
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyroAngleDegrees()), getLeftEncoderDistance(), getRightEncoderDistance());
    right.setInverted(true);
    resetEncoders();
    resetGyro();
  }

  public void setHalfSpeedTrue() {
    halfSpeed = true;
  }

  public void setHalfSpeedFalse() {
    halfSpeed = false;
  }

  public void setBrakeMode(boolean brakeMode) {
    if(brakeMode) {
      leftback.setNeutralMode(NeutralMode.Brake);
      leftfront.setNeutralMode(NeutralMode.Brake);
      rightback.setNeutralMode(NeutralMode.Brake);
      rightfront.setNeutralMode(NeutralMode.Brake);
    } else {
      leftback.setNeutralMode(NeutralMode.Coast);
      leftfront.setNeutralMode(NeutralMode.Coast);
      rightback.setNeutralMode(NeutralMode.Coast);
      rightfront.setNeutralMode(NeutralMode.Coast);
    }
  }

  //Assigns arcadeDrive speed and rotation
  public void drive(double speed, double rotation){
    if(halfSpeed) {
      diffDrive.arcadeDrive(speed / 2, -rotation / 2);
    } else {
      diffDrive.arcadeDrive(speed, -rotation);
    }
  }


  public void curveDrive(double speed, double rotation) {
    if(!halfSpeed) {
      diffDrive.curvatureDrive(rateLimitVelocity.calculate(speed), -rotation, true);
    } else {
      diffDrive.curvatureDrive(speed / 4, -rotation / 4, true);
    }
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

  public double gyroDistance() {
    return navX.getDisplacementX();
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
    double dist = (leftfront.getSelectedSensorPosition() / 4096) * 10.75 * (2 * Math.PI * DriveConstants.wheelRadiusMeters); //* 2*Math.PI*DriveConstants.wheelRadiusMeters);
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
    navX.resetDisplacement();
    navX.reset();
  }

  public double gyroAngleDegrees() {
    double angle = navX.getAngle() % 360; 
    if (angle < 0) {
      return 360 + angle; 
    } else {
      return angle; 
    }
  }

  public double gyroPitchDegrees() {
    return navX.getPitch();
  }

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
    //System.out.println(gyroAngleDegrees());
    //System.out.println(getEncoderDifference());
    System.out.println(getEncoderDistanceMeters());
    // This method will be called once per scheduler run
    m_odometry.update(
      navX.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
  }
}
