// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax intakeMotor = new CANSparkMax(MotorIDConstants.intakeMotor, MotorType.kBrushless);
  //private Solenoid intake = new Solenoid(10, PneumaticsModuleType.CTREPCM, 0);

  private static int coneIndex;
  private static int cubeIndex;
  private static boolean isCone;

  public IntakeSubsystem() {
    coneIndex = 0;
    cubeIndex = 3;
  }

  public void intake(double speed){
    intakeMotor.set(speed);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public int getIntakeIndex(){
    if(isCone){
      return getConeIntakeIndex();
    } else {
      return getCubeIntakeIndex();
    }
  }

  public int getConeIntakeIndex(){
    if(coneIndex >= 3){
      coneIndex = 0;
      return coneIndex;
    }
    else if(coneIndex <= -1){
      coneIndex = 2;
      return coneIndex;
    }
    return coneIndex;
  }

  public void increaseConeIndex(){
    isCone = true;
    coneIndex++;
  }

  public void decreaseConeIndex(){
    isCone = true;
    coneIndex--;
  }

  public int getCubeIntakeIndex(){
    if(cubeIndex >= 6){
      cubeIndex = 3;
      return cubeIndex;
    }
    else if(cubeIndex <= 2){
      cubeIndex = 5;
      return cubeIndex;
    }
    return cubeIndex;
  }

  public void increaseCubeIndex(){
    isCone = false;
    cubeIndex++;
  }

  public void decreaseCubeIndex(){
    isCone = false;
    cubeIndex++;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
