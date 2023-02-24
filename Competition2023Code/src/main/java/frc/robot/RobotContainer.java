// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands.DeployIntake;
import frc.robot.commands.ArmCommands.RetractIntake;
import frc.robot.commands.ArmCommands.ArmAnglePID;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.ExtendArmPID;
import frc.robot.commands.ArmCommands.Intake;
import frc.robot.commands.ArmCommands.ReverseIntake;
import frc.robot.commands.ArmCommands.RotateArm;
import frc.robot.commands.Autos.Auto1;
import frc.robot.commands.Autos.Auto10;
import frc.robot.commands.Autos.Auto2;
import frc.robot.commands.Autos.Auto3;
import frc.robot.commands.Autos.Auto4;
import frc.robot.commands.Autos.Auto6;
import frc.robot.commands.Autos.Auto7;
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

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
  public final ArmExtensionSubsystem m_armExtensionSubsystem = new ArmExtensionSubsystem();


  private final Joystick joystick = new Joystick(0);
  private final XboxController xbox = new XboxController(1);
  private final Trigger unIntake = new JoystickButton(joystick, 1);
  private final Trigger intake = new JoystickButton(joystick, 2);
  private final Trigger rotateArmToggleUp = new JoystickButton(joystick, 5);
  private final Trigger rotateArmToggleDown = new JoystickButton(joystick, 3);
  private final Trigger deployIntake = new JoystickButton(joystick, 7);
  private final Trigger retractIntake = new JoystickButton(joystick, 8);
  private final Trigger extendArmToggleUp = new JoystickButton(joystick, 6);
  private final Trigger extendArmToggleDown = new JoystickButton(joystick, 4);

  PathPlannerTrajectory examplePath = PathPlanner.loadPath("TestPath", new PathConstraints(4, 3));

  int armIndex = 0;
  int intakeIndex = 0;
  boolean isIncreasing = false; 



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, () -> joystick.getY(), () -> joystick.getX()));

    //m_armSubsystem.setDefaultCommand(positionCommand);
    //m_armExtensionSubsystem.setDefaultCommand(extentionCommand);
    
  }
  

public int GetArmIndex(){
  return armIndex;
}

public int GetIntakeIndex(){
  return intakeIndex;
}

 
public Command getArmCommand(){
  System.out.println("Reached GetArmCommand");
  if(isIncreasing){
    if(armIndex >= 3){
      armIndex = 3;
    }
    System.out.println("INDEX: " + armIndex);
    return new SelectCommand(
      Map.ofEntries(
        Map.entry(0,
          new ParallelCommandGroup(
            new ArmAnglePID(m_armSubsystem, ArmConstants.resetPosition),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.resetPositionLength)))),
        Map.entry(1, 
          new ParallelCommandGroup(
            new ArmAnglePID(m_armSubsystem, ArmConstants.lowPosition),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.lowPositionLength)))),
        Map.entry(2, 
          new ParallelCommandGroup(
            new ArmAnglePID(m_armSubsystem, ArmConstants.middlePosition),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.middlePositionLength)))),
        Map.entry(3, 
          new ParallelCommandGroup(
            new ArmAnglePID(m_armSubsystem, ArmConstants.highPosition),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.highPositionLength))))),
      this::GetArmIndex);
      } 
  else {
    if(armIndex <= 0){
      armIndex = 0;
    }
    System.out.println("INDEX: " + armIndex);
    return new SelectCommand(
      Map.ofEntries(
        Map.entry(0,
          new ParallelCommandGroup(
            new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.resetPositionLength),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new ArmAnglePID(m_armSubsystem, ArmConstants.resetPosition)))),
        Map.entry(1, 
          new ParallelCommandGroup(
            new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.lowPositionLength),  
            new SequentialCommandGroup(
              new WaitCommand(1),
              new ArmAnglePID(m_armSubsystem, ArmConstants.lowPosition)))),
        Map.entry(2, 
          new ParallelCommandGroup(
            new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.middlePositionLength),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new ArmAnglePID(m_armSubsystem, ArmConstants.middlePosition)))),
        Map.entry(3, 
          new ParallelCommandGroup(
            new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.highPositionLength),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new ArmAnglePID(m_armSubsystem, ArmConstants.highPosition))))),
      this::GetArmIndex);
  }
} 

  private void configureBindings() {
    intake.whileTrue(new Intake(m_intakeSubsystem, ArmConstants.intakeSpeed)); //2

    unIntake.whileTrue(new ReverseIntake(m_intakeSubsystem)); //1

    deployIntake.onTrue(new DeployIntake(m_intakeSubsystem)); //7

    retractIntake.onTrue(new RetractIntake(m_intakeSubsystem)); //8

    //5
    rotateArmToggleUp.onTrue(
    new ParallelCommandGroup(
      new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.resetPositionLength),
      
      //new ParallelRaceGroup(
      //    new ExtendArm(m_armExtensionSubsystem, -.5), 
      //    new WaitCommand(1)),
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new RotateArm(m_armSubsystem, .02),
          new WaitCommand(1)), 
        new ArmAnglePID(m_armSubsystem, ArmConstants.resetPosition))
      )
    );
    
  
    
    extendArmToggleUp.onTrue(
    new ParallelCommandGroup(
      new SequentialCommandGroup(
        new WaitCommand(1),
        new ArmAnglePID(m_armSubsystem, ArmConstants.lowPosition)),
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new ExtendArm(m_armExtensionSubsystem, -0.2), 
          new WaitCommand(1)),
        new WaitCommand(1),
        new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.lowPositionLength))));
    

    //3
    rotateArmToggleDown.onTrue(new ParallelCommandGroup(
        new ArmAnglePID(m_armSubsystem, ArmConstants.middlePosition),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.middlePositionLength))));

    //4
    extendArmToggleDown.onTrue(new ParallelCommandGroup(
        new ArmAnglePID(m_armSubsystem, ArmConstants.highPosition),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new ExtendArmPID(m_armExtensionSubsystem, ArmConstants.highPositionLength))));

  
  }

  public Command ChooseAuto(AutoType type) {
    switch(type){
        case do_nothing:
          return new Auto1(m_driveSubsystem, m_armSubsystem, examplePath);
        case path_planner:
          return new Auto2(m_driveSubsystem, m_armSubsystem, examplePath, true);
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
        case drive_forwards_score_leave_community_pickup_piece_score:
          return new Auto9(m_driveSubsystem, m_armSubsystem);
        case drive_forwards_score_leave_community_dock_engage:
          return new Auto10(m_driveSubsystem, m_armSubsystem);

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
    return ChooseAuto(AutoType.path_planner);
  }
}
