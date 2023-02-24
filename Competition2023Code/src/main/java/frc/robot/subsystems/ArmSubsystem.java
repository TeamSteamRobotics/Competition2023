// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;
import javax.swing.SingleSelectionModel;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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



  private CANSparkMax armMotorLeft = new CANSparkMax(MotorIDConstants.leftShoulderMotor, MotorType.kBrushless);
  private CANSparkMax armMotorRight = new CANSparkMax(MotorIDConstants.rightShoulerMotor, MotorType.kBrushless);
  private MotorControllerGroup armMotors = new MotorControllerGroup(armMotorLeft, armMotorRight);

  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);

  private double dutyCycleOffset = 0.3046; //0.0805; //0.2017; //0.618333;
  // 0 - 1 to 0 - 6.283: 1.2673
  private static int rotationIndex = 0; 
  //need to somehow do 2pi - the encoder thingy
  //Another encoder will be placed, it is not on the motor controllers and it is on the rotate arm part
  
  //2.42 - 0.5061
  
  //29
  //0.5061 radians
  //.71 = 90 degrees
  //2.85 = PI/2
  
  public ArmSubsystem() {
    armEncoder.setDistancePerRotation(2 * Math.PI);
    //armEncoder.setPositionOffset(dutyCycleOffset);
    
    //armMotorRight.setInverted(false);
    //armMotorLeft.setInverted(false);
    //armMotorRight.follow(armMotorLeft);
  }

  public void increaseRotationIndex(){
    rotationIndex += 1;
    System.out.println(rotationIndex);
  }

  public void decreaseRotationIndex(){
    rotationIndex -= 1;
    System.out.println(rotationIndex);
  }

  public int getRotationIndex() {
    return rotationIndex % 4; 
  }

  public void resetElevatorEncoders() {
    //elevatorEncoder.setPosition(0);
  }


  public double getArmAngleDegrees(){
    //System.out.println(armEncoder.getDistance());
    return (Math.PI * 2) - armEncoder.getDistance() - 1.9139;
  }
  
  public void setArmSpeed(double speed){
    if(getArmAngleDegrees() > 2 && getArmAngleDegrees() < 2.2){
      armMotorLeft.set(0);
      armMotorRight.set(0);
    }
    else if(getArmAngleDegrees() > .25 && getArmAngleDegrees() < .35){
      armMotorLeft.set(0);
      armMotorRight.set(0);
    }
    else if(getArmAngleDegrees() > 4){
      //System.out.println("First if");
      armEncoder.reset();
      armEncoder.setPositionOffset(dutyCycleOffset);
    } else if(getArmAngleDegrees() < 0){
      //System.out.println("Second if");
      armEncoder.reset();
      armEncoder.setPositionOffset(dutyCycleOffset);
    } else{
      //System.out.println("Else statement");
      armMotorLeft.set(speed);
      armMotorRight.set(speed);
    }
   
  }

  //sets individual arm motor 
  public void setRightMotor(double speed) {
    armMotorRight.set(speed);
  }
  public void setLeftMotor(double speed) {
    armMotorLeft.set(speed);
  }
  
//stopArm sets armMotors to 0
  public void stopArm(){
    armMotors.set(0);

  }
//2.838 - 1.5707 = 1.2673
//Overrides code
  @Override
  public void periodic() {
    System.out.println(this.getArmAngleDegrees());
    // This method will be called once per scheduler run
  }
  
}  