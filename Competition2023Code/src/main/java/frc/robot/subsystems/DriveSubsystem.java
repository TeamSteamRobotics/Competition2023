// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorIDConstants;
import edu.wpi.first.wpilibj.SerialPort;

public class DriveSubsystem extends SubsystemBase {
  
  private WPI_VictorSPX leftback = new WPI_VictorSPX(MotorIDConstants.leftBackDrive);
  private WPI_TalonSRX leftfront = new WPI_TalonSRX(MotorIDConstants.leftFrontDrive);
  private WPI_VictorSPX rightback = new WPI_VictorSPX(MotorIDConstants.rightBackDrive);
  private WPI_TalonSRX rightfront = new WPI_TalonSRX(MotorIDConstants.rightFrontDrive);
  
  private MotorControllerGroup left = new MotorControllerGroup(leftback, leftfront);
  private MotorControllerGroup right = new MotorControllerGroup(rightback, rightfront);

  private DifferentialDrive diffDrive = new DifferentialDrive(left, right);

  //private AHRS navX = new AHRS(SerialPort.Port.kMXP);

  public DriveSubsystem() {
    right.setInverted(true);
  }

  public void drive(double speed, double rotation){
    diffDrive.arcadeDrive(-speed, rotation);
  }

  public double encoderDifference() {
    //System.out.println(leftfront.getSelectedSensorPosition() - rightfront.getSelectedSensorPosition());
    return leftfront.getSelectedSensorPosition() - rightfront.getSelectedSensorPosition();
  }

  public void stop(){
    diffDrive.arcadeDrive(0, 0);
  }

  public double getEncoderPosRAW() {
    return (leftfront.getSelectedSensorPosition() + rightfront.getSelectedSensorPosition()) / 2;
  }

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

  public void resetEncoders() {
    leftfront.setSelectedSensorPosition(0);
    rightfront.setSelectedSensorPosition(0);
  }

  /*public void resetGyro() {
    navX.reset();
  }*/

  public double gyroAngleDegrees() {
    //return navX.getAngle();
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
