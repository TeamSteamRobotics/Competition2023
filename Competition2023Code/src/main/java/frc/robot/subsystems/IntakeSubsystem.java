// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax intakeMotor = new CANSparkMax(MotorIDConstants.intakeMotor, MotorType.kBrushless);
  //private Solenoid intake = new Solenoid(10, PneumaticsModuleType.CTREPCM, 0);

  private static int index;

  public IntakeSubsystem() {
    index = 0;
  }

  public void intake(double speed){
    intakeMotor.set(speed);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public int getIntakeIndex(){
    return index;
  }

  public void increaseIndex(){
    index++;
  }

  public void decreaseIndex(){
    index--;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
