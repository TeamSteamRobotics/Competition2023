// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AprilDriveTest;
import frc.robot.commands.Drive;


import frc.robot.commands.DriveToApril;
import frc.robot.commands.DriveToCoordinate;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.AprilVisionSubsystem; 

import frc.robot.subsystems.AprilVisionSubsystem.Coordinate;

import java.nio.file.attribute.PosixFilePermissions;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final AprilVisionSubsystem m_aprilVisionSubsystem = new AprilVisionSubsystem(); 

  private final Joystick joystick = new Joystick(0);
  private final Trigger testButton = new JoystickButton(joystick, 4);
  private final Trigger testButtonAlternate = new JoystickButton(joystick, 3);
  private final Trigger intakeTest = new JoystickButton(joystick, 6);
  private final Trigger unintakeTest = new JoystickButton(joystick, 7);
  private final Trigger button = new JoystickButton(joystick, 5);
  private final Trigger driveToApril = new JoystickButton(joystick, 9);
  private final Trigger driveToAprilInverted = new JoystickButton(joystick, 10); 
  
  Coordinate targetCoordinate = m_aprilVisionSubsystem.new Coordinate();
 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, m_aprilVisionSubsystem, () -> joystick.getY(), () -> joystick.getX()));
    targetCoordinate.x = 1;
    targetCoordinate.z = 1;

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driveToApril.onTrue(
      //new ParallelDeadlineGroup(
        new AprilDriveTest(m_aprilVisionSubsystem, m_driveSubsystem, 0.5)
    );
   // driveToAprilInverted.onTrue(
      //new ParallelDeadlineGroup(
        //new DriveToApril(m_aprilVisionSubsystem, m_driveSubsystem, 0.5f, 3.5f, true)
   // );
      
  
    
    /*testButtonAlternate.onTrue( new ParallelDeadlineGroup (
      new WaitCommand(2),
      new RotateArm(m_armSubsystem, -0.1)
      ) 
    );
    */
 
  

    //new InstantCommand(() -> m_visionSubsystem.visionDistanceTest(), m_visionSubsystem));
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
