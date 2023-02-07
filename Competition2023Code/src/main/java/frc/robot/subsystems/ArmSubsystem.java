// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;
import javax.swing.SingleSelectionModel;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorIDConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private CANSparkMax elevatorMotor = new CANSparkMax(6, MotorType.kBrushless);

  private CANSparkMax armMotorLeft = new CANSparkMax(MotorIDConstants.leftElevatorMotor, MotorType.kBrushless);
  private CANSparkMax armMotorRight = new CANSparkMax(MotorIDConstants.rightElevatorMotor, MotorType.kBrushless);
  private MotorControllerGroup armMotors = new MotorControllerGroup(armMotorLeft, armMotorRight);

  private CANSparkMax intakeMotor = new CANSparkMax(MotorIDConstants.intakeMotor, MotorType.kBrushless);

  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);

  private double dutyCycleOffset = 0.158333;

  private int index = 0; 

  private Solenoid intakeSolenoid =  new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  //Another encoder will be placed, it is not on the motor controllers and it is on the rotate arm part

  public ArmSubsystem() {
    armEncoder.setDistancePerRotation(2 * Math.PI);
    armEncoder.setPositionOffset(dutyCycleOffset);
    elevatorEncoder.setPosition(0);
    //armMotorRight.setInverted(false);
    //armMotorLeft.setInverted(false);
    //armMotorRight.follow(armMotorLeft);
  }

  public void increaseIndex() {
    if (index > 2) {
      index = 0;
    } else {
      index += 1; 
    }
  }
  public void decreaseIndex() {
    if (index == 0) {
      index = 2;
    } else {
      index -= 1; 
    }
  }
  public int getIndex() {
    return index; 
  }

  /*public void resetAngleEncoder() {
    angleEncoderRight.setPosition(0);
    angleEncoderLeft.setPosition(0);
  }*/

  public void resetElevatorEncoders() {
    //elevatorEncoder.setPosition(0);
  }

  public double armLengthMeters() {
    return -1 * (elevatorEncoder.getPosition()) * ArmConstants.armConversionFactor;
  }

  /*public double armAngleDegrees() {
    return (angleEncoderRight.getPosition() * angleEncoderLeft.getPosition() * 180);
  }*/

  public double getArmAngleDegrees(){
    //return 0.0;
    System.out.println(armEncoder.getDistance());
    return armEncoder.getDistance();
  }
  
  //angleArm sets armMotors to input speed
  public void angleArm(double speed){
    armMotors.set(speed);
  }

  public void zachRotateArm(double speed){
    if(armEncoder.getDistance() > 1.8){
      armMotorLeft.set(0);
      armMotorRight.set(0);
    } else if(armEncoder.getDistance() < .44){
      armMotorLeft.set(0);
      armMotorRight.set(0);
    } else{
      armMotorLeft.set(speed);
      armMotorRight.set(speed);
    }
   
  }

  public void angleRightMotor(double speed) {
    armMotorRight.set(speed);
  }
// angleLeftMotor sets armMotorLeft to input speed
  public void angleLeftMotor(double speed) {
    armMotorLeft.set(speed);
  }
//Creates extendArm method with speed input
  public void extendArm(double speed){
    elevatorMotor.set(speed);
    System.out.println(armLengthMeters());
  }
// intake sets intakeMotor to input speed
  public void intake(double speed){
    intakeMotor.set(speed);
  }
//stopArm sets armMotors to 0
  public void stopArm(){
    armMotors.set(0);

  }
//Creates stopElevator method
  public void stopElevator(){
    elevatorMotor.set(0);
  }

  // stopIntake sets intakeMotor to 0
  public void stopIntake(){
    intakeMotor.set(0);
  }
  
  // stopAll sets intakeMotor and armMotor to 0
  public void stopAll(){
    //elevatorMotor.set(0);
    intakeMotor.set(0);
    armMotors.set(0);
  }
//Overrides code
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void retractIntake(){
    intakeSolenoid.set(false);
  }

  public void deployIntake(){
    intakeSolenoid.set(true);
  }
}
