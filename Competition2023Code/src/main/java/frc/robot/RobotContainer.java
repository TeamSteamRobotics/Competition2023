// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCommands.DeployIntake;
import frc.robot.commands.ArmCommands.RetractIntake;
import frc.robot.commands.ArmCommands.Intake;
import frc.robot.commands.ArmCommands.ReverseIntake;
import frc.robot.commands.Autos.Auto1;
import frc.robot.commands.Autos.Auto10;
import frc.robot.commands.Autos.Auto3;
import frc.robot.commands.Autos.Auto4;
import frc.robot.commands.Autos.Auto5;
import frc.robot.commands.Autos.Auto6;
import frc.robot.commands.Autos.Auto7;
import frc.robot.commands.Autos.Auto9;
import frc.robot.commands.Autos.FollowTrajectory;
import frc.robot.commands.DriveCommands.CurveDrive;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.PositionCommands.HighArmPosition;
import frc.robot.commands.PositionCommands.LowArmPosition;
import frc.robot.commands.PositionCommands.MiddleArmPosition;
import frc.robot.commands.PositionCommands.ResetArmPosition;
import frc.robot.commands.DriveCommands.CurvatureDrive;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.AprilDriveTest;
import frc.robot.commands.AprilDriveTestGPT;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;

import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();


  //Driver's controls
  private final Joystick joystick = new Joystick(1);
  private final Trigger halveSpeed = new JoystickButton(joystick, 1);
  private final Trigger normalSpeed = new JoystickButton(joystick, 2);
  private final Trigger resetArmButton = new JoystickButton(joystick, 5);
  private final Trigger middleArmButton = new JoystickButton(joystick, 3);
  private final Trigger deployIntake = new JoystickButton(joystick, 7);
  private final Trigger retractIntake = new JoystickButton(joystick, 8);
  private final Trigger lowArmButton = new JoystickButton(joystick, 6);
  private final Trigger highArmButton = new JoystickButton(joystick, 4);
  private final Trigger intakeToggleTest = new JoystickButton(joystick, 9);
  private final Trigger reverseIntakeToggleTest = new JoystickButton(joystick, 10);

  //Operator's controls
  private final CommandXboxController xbox = new CommandXboxController(0);
  private final Trigger resetPosition = xbox.a();
  private final Trigger lowPosition = xbox.x();
  private final Trigger middlePosition = xbox.y();
  private final Trigger highPosition = xbox.b();
  private final Trigger intakeToggle = xbox.rightBumper();
  private final Trigger reverseIntakeToggle = xbox.leftBumper();
  private final Trigger resetIntakeToggles = xbox.povUp();
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final Trigger unIntake = operatorController.leftBumper();
  private final Trigger intake = operatorController.rightBumper();
  private final Trigger rotateArmToggleUp = operatorController.a();
  private final Trigger rotateArmToggleDown = operatorController.y();
  private final Trigger deployIntake = operatorController.povUp();
  private final Trigger retractIntake = operatorController.povDown();
  private final Trigger extendArmToggleUp = operatorController.x();
  private final Trigger extendArmToggleDown = operatorController.b();
  private final Trigger intakeToggleTest = operatorController.rightTrigger();
  private final Trigger reverseIntakeToggleTest = operatorController.leftTrigger();

  int armIndex = 0;
  boolean isIncreasing = false; 


  PathPlannerTrajectory examplePath = PathPlanner.loadPath("TestPath", new PathConstraints(4, 3));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, () -> joystick.getY(), () -> joystick.getX()));
    //m_driveSubsystem.setDefaultCommand(new CurveDrive(m_driveSubsystem, driverXbox::getRightY, driverXbox::getLeftX));
    m_intakeSubsystem.setDefaultCommand(intakeCommand);
    m_driveSubsystem.setDefaultCommand(new CurvatureDrive(driverController::getLeftY, driverController::getRightX, m_driveSubsystem));
    //m_intakeSubsystem.setDefaultCommand(intakeCommand);

    //m_armSubsystem.setDefaultCommand(positionCommand);
    //m_armExtensionSubsystem.setDefaultCommand(extentionCommand);
    
  }

  private final Command intakeCommand = 
  new SelectCommand(
    Map.ofEntries(
        Map.entry(0, new Intake(m_intakeSubsystem, 0)),
        Map.entry(1, new Intake(m_intakeSubsystem)),
        Map.entry(2, new Intake(m_intakeSubsystem, .1)),
        Map.entry(3, new ReverseIntake(m_intakeSubsystem, 0)),
        Map.entry(4, new ReverseIntake(m_intakeSubsystem)),
        Map.entry(5, new ReverseIntake(m_intakeSubsystem, 0.1))),
        m_intakeSubsystem::getIntakeIndex);
  
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
    //Driver's Commands
    //intake.whileTrue(new Intake(m_intakeSubsystem, ArmConstants.intakeSpeed)); //2
    //unIntake.whileTrue(new ReverseIntake(m_intakeSubsystem)); //1

    halveSpeed.onTrue(new InstantCommand(m_driveSubsystem::setHalfSpeedTrue, m_driveSubsystem));
    normalSpeed.onTrue(new InstantCommand(m_driveSubsystem::setHalfSpeedFalse, m_driveSubsystem));

    deployIntake.onTrue(new DeployIntake(m_pneumaticsSubsystem)); //7
    retractIntake.onTrue(new RetractIntake(m_pneumaticsSubsystem)); //8

    intakeToggleTest.onTrue(new InstantCommand(m_intakeSubsystem::increaseConeIndex, m_intakeSubsystem)); //9
    reverseIntakeToggleTest.onTrue(new InstantCommand(m_intakeSubsystem::increaseCubeIndex, m_intakeSubsystem)); //10

    resetArmButton.onTrue(new ResetArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem)); //5
    lowArmButton.onTrue(new LowArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem)); //6
    middleArmButton.onTrue(new MiddleArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem)); //3
    highArmButton.onTrue(new HighArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem)); //4

    //Operator's Commands
    resetPosition.onTrue(new ResetArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem));
    lowPosition.onTrue(new LowArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem));
    middlePosition.onTrue(new MiddleArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem));
    highPosition.onTrue(new HighArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem));

    intakeToggle.onTrue(new InstantCommand(m_intakeSubsystem::increaseConeIndex, m_intakeSubsystem));
    reverseIntakeToggle.onTrue(new InstantCommand(m_intakeSubsystem::increaseCubeIndex, m_intakeSubsystem));
    resetIntakeToggles.onTrue(new InstantCommand(m_intakeSubsystem::resetIndexes, m_intakeSubsystem));

  }

  public Command ChooseAuto(AutoType type) {
    switch(type){
        case do_nothing:
          return new Auto1(m_driveSubsystem, m_armSubsystem, examplePath);
        case drive_forwards_score_drive_back_dock:
          return new Auto3(m_driveSubsystem, m_armSubsystem, m_pneumaticsSubsystem, m_armExtensionSubsystem, m_intakeSubsystem);
        case drive_forwards_score:
          return new Auto4(m_driveSubsystem, m_armSubsystem, m_pneumaticsSubsystem, m_armExtensionSubsystem, m_intakeSubsystem);
        case drive_backwards_drive_forwards_dock:
          return new Auto5(m_driveSubsystem);
        case drive_backwards_dock_engage:
          return new Auto6(m_driveSubsystem, m_armSubsystem, m_driveSubsystem);
        case drive_forwards_score_drive_back_pick_up_piece:
          return new Auto7(m_driveSubsystem, m_armSubsystem, m_pneumaticsSubsystem, m_armExtensionSubsystem, m_intakeSubsystem);
        case drive_forwards_score_leave_community_pickup_piece_score:
          return new Auto9(m_driveSubsystem, m_armSubsystem);
        case drive_forwards_score_leave_community_dock_engage:
          return new Auto10(m_driveSubsystem, m_armSubsystem);
        case path_planner:
          return new FollowTrajectory(m_driveSubsystem, m_armSubsystem, examplePath, true);
        default:
            return null;
    }
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return ChooseAuto(AutoType.path_planner);
  }
}
