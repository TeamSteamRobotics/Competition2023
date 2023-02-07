// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmAnglePID;
import frc.robot.commands.Auto.Auto10;
import frc.robot.commands.Auto.Auto11;
import frc.robot.commands.Auto.Auto2;
import frc.robot.commands.Auto.Auto3;
import frc.robot.commands.Auto.Auto4;
import frc.robot.commands.Auto.Auto6;
import frc.robot.commands.Auto.Auto7;
import frc.robot.commands.Auto.Auto8;
import frc.robot.commands.Auto.Auto9;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveRotationPID;
import frc.robot.commands.EncoderDriveDistance;
import frc.robot.commands.Intake;
import frc.robot.commands.RotateArm;
import frc.robot.commands.Auto.Auto1;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.DriveToApril; 
import frc.robot.commands.ArmTogglePID; 
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem; 

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
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final AprilVisionSubsystem m_aprilVisionSubsystem = new AprilVisionSubsystem(); 

  private final Joystick joystick = new Joystick(0);
  private final Trigger driveToTarget = new JoystickButton(joystick, 9);
  private final Trigger toggleArmUp = new JoystickButton(joystick, 4);
  private final Trigger toggleArmDown = new JoystickButton(joystick, 3);
  private final Trigger intakeTest = new JoystickButton(joystick, 6);
  private final Trigger unintakeTest = new JoystickButton(joystick, 7);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, () -> joystick.getY(), () -> joystick.getX()));

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
    driveToTarget.onTrue(
     new SequentialCommandGroup(
        new InstantCommand(m_driveSubsystem::resetEncoders), 
        new EncoderDriveDistance(m_visionSubsystem.visionDistanceTest(), m_driveSubsystem))
      
    );
    
    toggleArmUp.onTrue( 
      new ArmTogglePID(m_armSubsystem, true)
    );
    toggleArmDown.onTrue( 
      new ArmTogglePID(m_armSubsystem, false)
    );

    
    /*testButtonAlternate.onTrue( new ParallelDeadlineGroup (
      new WaitCommand(2),
      new RotateArm(m_armSubsystem, -0.1)
      ) 
    );
    */
  intakeTest.whileTrue(new Intake(m_armSubsystem));
  

    //new InstantCommand(() -> m_visionSubsystem.visionDistanceTest(), m_visionSubsystem));
  }
  public Command ChooseAuto(AutoType type) {
    switch(type){
        case do_nothing:
          return new Auto1(m_driveSubsystem, m_armSubsystem);
        case drive_backwards_dock:
          return new Auto2();
        case drive_forwards_score_drive_back_dock:
          return new Auto3(m_driveSubsystem, m_armSubsystem);
        case drive_forwards_score:
          return new Auto4(m_driveSubsystem, m_armSubsystem);
        case drive_backwards_drive_forwards_dock:
          //return Auto5
          break;
        case drive_backwards_dock_engage:
          return new Auto6(m_driveSubsystem, m_armSubsystem);
        case drive_forwards_score_drive_back_pick_up_piece:
          return new Auto7(m_driveSubsystem, m_armSubsystem);
        case drive_back_move_pieces_to_our_side:
          return new Auto8(m_driveSubsystem, m_armSubsystem);
        case drive_forwards_score_leave_community_pickup_piece_score:
          return new Auto9(m_driveSubsystem, m_armSubsystem);
        case drive_forwards_score_leave_community_dock_engage:
          return new Auto10(m_driveSubsystem, m_armSubsystem);
        case drive_forwards_score_leave_community_go_to_plaer_station:
          return new Auto11(m_driveSubsystem, m_armSubsystem);
        default:
            return null;
    }
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new ParallelRaceGroup(new Drive(m_driveSubsystem, () -> 0.5, () -> 0), new WaitCommand(5) );
    // An example command will be run in autonomous
    return ChooseAuto(AutoType.drive_forwards_score);//new AutoDriveBackwardsDockAndEngage(m_driveSubsystem, m_armSubsystem);
    
    //new SequentialCommandGroup(

      //new InstantCommand(m_driveSubsystem::resetEncoders),
      //new EncoderDriveDistance(5, m_driveSubsystem));
  }
}
