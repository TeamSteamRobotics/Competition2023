// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands.ArmAnglePID;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.ExtendArmPID;
//import frc.robot.commands.ArmCommands.Intake;
import frc.robot.commands.ArmCommands.RotateArm;
import frc.robot.commands.Autos.Auto1;
import frc.robot.commands.Autos.Auto10;
import frc.robot.commands.Autos.Auto11;
import frc.robot.commands.Autos.Auto2;
import frc.robot.commands.Autos.Auto3;
import frc.robot.commands.Autos.Auto4;
import frc.robot.commands.Autos.Auto6;
import frc.robot.commands.Autos.Auto7;
import frc.robot.commands.Autos.Auto8;
import frc.robot.commands.Autos.Auto9;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.DriveCommands.DriveRotationPID;
import frc.robot.commands.DriveCommands.DriveToApril;
import frc.robot.commands.DriveCommands.EncoderDriveDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;

import java.nio.file.attribute.PosixFilePermissions;
import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
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
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ArmExtensionSubsystem m_armExtensionSubsystem = new ArmExtensionSubsystem();


  private final Joystick joystick = new Joystick(0);
  private final Trigger unIntake = new JoystickButton(joystick, 1);
  private final Trigger intake = new JoystickButton(joystick, 2);
  private final Trigger toggleArmUp = new JoystickButton(joystick, 3);
  private final Trigger toggleArmDown = new JoystickButton(joystick, 4);
  //private final Trigger arm90 = new JoystickButton(joystick, 5);
  private final Trigger arm45 = new JoystickButton(joystick, 6);
  private final Trigger deployIntake = new JoystickButton(joystick, 7);
  private final Trigger unDeployIntake = new JoystickButton(joystick, 8);
  private final Trigger extendArmManual = new JoystickButton(joystick, 9);
  private final Trigger retractArmManual = new JoystickButton(joystick, 10);
  private final Trigger extendArmPID = new JoystickButton(joystick, 11);
  private final Trigger driveToTarget = new JoystickButton(joystick, 12);
  private final Trigger extendLiftArmTest = new JoystickButton(joystick, 5);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, () -> joystick.getY(), () -> joystick.getX()));

    //m_armSubsystem.setDefaultCommand(positionCommand);
  }
  
  private final Command positionCommand = 
    new SelectCommand(
      Map.ofEntries(
          Map.entry(0, new ArmAnglePID(m_armSubsystem, ArmConstants.resetPosition)),
          Map.entry(1, new ArmAnglePID(m_armSubsystem, ArmConstants.lowPosition)),
          Map.entry(2, new ArmAnglePID(m_armSubsystem, ArmConstants.middlePosition)),
          Map.entry(3, new ArmAnglePID(m_armSubsystem, ArmConstants.highPosition))),
          m_armSubsystem::getIndex);

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
    intake.whileTrue(
      new Intake(m_ArmSubsystem)
    );
    unIntake.whileTrue(
      new ReverseIntake(m_ArmSubsystem)
    );
    deployIntake.onTrue(
      new DeployIntake(m_ArmSubsystem)
    );
    unDeployIntake.onTrue(
      new RetractIntake(m_ArmSubsystem)

    );

    

    driveToTarget.onTrue( 
     new SequentialCommandGroup(
        new InstantCommand(m_driveSubsystem::resetEncoders), 
        new EncoderDriveDistance(m_visionSubsystem.visionDistanceTest(), m_driveSubsystem))
    ); //9
    
    extendArmPID.onTrue(new ExtendArmPID(m_armExtensionSubsystem, .20)); //11
    extendLiftArmTest.onTrue(new SequentialCommandGroup(new ArmAnglePID(m_armSubsystem, Math.PI / 4) , new ExtendArmPID(m_armExtensionSubsystem, .2)));
    extendArmManual.whileTrue(new ExtendArm(m_armExtensionSubsystem, .2)); //9
    retractArmManual.whileTrue(new ExtendArm(m_armExtensionSubsystem, -.2)); //10
    
    //arm toggles 
    toggleArmUp.whileTrue(new RotateArm(m_armSubsystem, 0.2)); //3
    toggleArmDown.whileTrue(new RotateArm(m_armSubsystem, -0.2)); //4

    //arm position sets
    //arm90.onTrue(new ArmAnglePID(m_armSubsystem, Math.PI / 2)); //5
    //arm45.onTrue(new ArmAnglePID(m_armSubsystem, Math.PI/4)); //6

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
