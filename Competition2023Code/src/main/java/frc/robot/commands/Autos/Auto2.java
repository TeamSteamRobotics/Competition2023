// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands.EncoderDriveDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2 extends SequentialCommandGroup {
  /** Creates a new Auto2. */
  
  //test values
  double KS = .5;
  double KV = .5;
  double KA = .5; 

  public Auto2(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, PathPlannerTrajectory traj, Boolean isFirstPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
          new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(traj.getInitialPose());
            }
          }),
          new PPRamseteCommand(
              traj, 
              this::getPose, // Pose supplier
              new RamseteController(),
              new SimpleMotorFeedforward(KS,KV,KA),
              this.kinematics, // DifferentialDriveKinematics
              this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
              new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
              this::outputVolts, // Voltage biconsumer
              true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
              this // Requires this drive subsystem
          )
      )
    );
  }
}
