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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowTrajectory extends SequentialCommandGroup {
  /** Creates a new Auto2. */
  
  public FollowTrajectory(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, PathPlannerTrajectory traj, Boolean isFirstPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
          new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                driveSubsystem.resetOdometry(traj.getInitialPose());
            }
            System.out.println("started"); 
          }),
          new PPRamseteCommand(
              traj, 
              driveSubsystem::getPose, // Pose supplier
              new RamseteController(),
              new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,Constants.DriveConstants.ksVoltSecondsPerMeter,Constants.DriveConstants.kvVoltSecondsSquaredPerMeter),
              driveSubsystem.getKinematics(), // DifferentialDriveKinematics
              driveSubsystem::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
              new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0), // Right controller (usually the same values as left controller)
              driveSubsystem::diffDriveVolts, // Voltage biconsumer
              true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
              driveSubsystem // Requires this drive subsystem
          )
      )
    );
  }
}
