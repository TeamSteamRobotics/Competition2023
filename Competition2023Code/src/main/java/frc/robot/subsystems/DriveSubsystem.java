// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorIDConstants;
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

  //private AHRS gyro = new AHRS();

  //private AHRS navX = new AHRS(SerialPort.Port.kMXP);
  
  //Inverts right MotorControllerGroup
  public DriveSubsystem() {
    right.setInverted(true);
    resetEncoders();
  }

  //Assigns arcadeDrive speed and rotation
  public void drive(double speed, double rotation){
    //System.out.println("leftfront" + leftfront.get());
    //System.out.println("leftback" + leftback.get());
    //System.out.println("rightfront" + rightfront.get());
    //System.out.println("rightback" + rightback.get());
    diffDrive.arcadeDrive(speed, -rotation);
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
  public double getEncoderDifference() {
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
    //eturn navX.getAngle();
    return 0;
  }

  public double gyroPitchDegrees() {
    return 0;
  }

  //0 degrees = 0 encoder difference
  //90 degrees = -5864

  // Overrides code
  @Override
  public void periodic() {
    //System.out.println(getEncoderDifference());
    // This method will be called once per scheduler run
  }
}
