// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmAngleLowPID extends PIDCommand {
  /** Creates a new ArmAngleLowPID. */
  public ArmAngleLowPID(ArmSubsystem arm, double angle) {
    super(
        // The controller that the command will use
        new PIDController(ArmConstants.low_angle_kP, ArmConstants.low_angle_kI, ArmConstants.low_angle_kD),
        // This should return the measurement
        () -> arm.getArmAngleDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          // Use the output here
          arm.setArmSpeed(output);
        });
      addRequirements(arm);
      this.getController().setTolerance(ArmConstants.anglePIDTolerance);
      this.getController().setIntegratorRange(-0.4/ArmConstants.angle_kI, 0.4/ArmConstants.angle_kI);
    
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
