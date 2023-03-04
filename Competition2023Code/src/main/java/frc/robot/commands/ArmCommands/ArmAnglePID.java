// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html



public class ArmAnglePID extends PIDCommand {
  /** Creates a new ArmAnglePID. */
  
  public ArmAnglePID(ArmSubsystem arm, double angle) {

    super(
        // The controller that the command will use
        new PIDController(ArmConstants.angle_kP, ArmConstants.angle_kI, ArmConstants.angle_kD),
        // This should return the measurement
        () -> arm.getArmAngleDegrees(), //also increments index
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        output -> {
          //System.out.println("reached ARM ANGLE PID");
          //System.out.println("Angle Output: " + output);

          System.out.println();
          arm.setArmSpeed(output);
          // Use the output here
        });
    addRequirements(arm);
    
 
    this.getController().setTolerance(ArmConstants.anglePIDTolerance);
    this.getController().setIntegratorRange(-0.4/ArmConstants.angle_kI, 0.4/ArmConstants.angle_kI);
    
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  //arm code to test out
  /* 
  @Override
  public void execute() {
    if(ArmSubsystem.goingLow != true){
      this.getController().setP(ArmConstants.angle_kP);
      this.getController().setI(ArmConstants.angle_kI);
      this.getController().setD(ArmConstants.angle_kD);
      super.execute();
    } else {
      this.getController().setP(ArmConstants.angle_kP * 0.65);
      this.getController().setI(ArmConstants.angle_kI * 0.65);
      this.getController().setD(ArmConstants.angle_kD * 0.65);
      super.execute();
    }
  }
  */

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //this.getController().atSetpoint();
  }
}