// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmCommands.DeployIntake;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.Intake;
import frc.robot.commands.ArmCommands.RetractIntake;
import frc.robot.commands.ArmCommands.ReverseIntake;
import frc.robot.commands.ArmCommands.RotateArm;
import frc.robot.commands.ArmCommands.PositionCommands.HighArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.HumanPlayerStationPosition;
import frc.robot.commands.ArmCommands.PositionCommands.LowArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.MiddleArmPosition;
import frc.robot.commands.ArmCommands.PositionCommands.ResetArmPosition;
import frc.robot.commands.Autos.Auto1;
import frc.robot.commands.Autos.Auto10;
import frc.robot.commands.Autos.CubeMiddleTaxi;
import frc.robot.commands.Autos.Dock;
import frc.robot.commands.Autos.CubeHighTaxi;
import frc.robot.commands.Autos.ConeHighTaxi;
import frc.robot.commands.Autos.Auto3;
import frc.robot.commands.Autos.Auto4;
import frc.robot.commands.Autos.Auto5;
import frc.robot.commands.Autos.Auto6;
import frc.robot.commands.Autos.Auto7;
import frc.robot.commands.Autos.Auto9;
import frc.robot.commands.Autos.FollowTrajectory;
import frc.robot.commands.DriveCommands.BalancePID;
import frc.robot.commands.DriveCommands.CurvatureDrive;
import frc.robot.commands.DriveCommands.GyroDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;

import java.util.Map;

import javax.management.InstanceNotFoundException;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
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
  private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();

  //Driver Controller
  private final Joystick joystick = new Joystick(2);
  private final Trigger gyroDrive = new JoystickButton(joystick, 12);
  //private final Trigger halfSpeed = new JoystickButton(joystick, 1);
  //private final Trigger fullSpeed = new JoystickButton(joystick, 2);
  private final CommandXboxController driverController = new CommandXboxController(1);
  private final Trigger halfSpeed = driverController.rightTrigger();
  private final Trigger fullSpeed = driverController.leftTrigger();
  private final Trigger brakeModeOn = driverController.rightBumper();
  private final Trigger balanceBeam = driverController.leftBumper();
  private final Trigger coastMode = driverController.a();

  // 55.4 inches
  //Operator Controller
  private final CommandXboxController operatorController = new CommandXboxController(0);
  private final Trigger resetPosition = operatorController.a();
  private final Trigger lowPosition = operatorController.x();
  private final Trigger middlePosition = operatorController.y();
  private final Trigger highPosition = operatorController.b();
  private final Trigger humanPlayerStation = operatorController.x().and(operatorController.b());
  private final Trigger intakeToggle = operatorController.rightBumper();
  private final Trigger reverseIntakeToggle = operatorController.leftBumper();
  private final Trigger resetIntakeToggles = operatorController.povUp();
  private final Trigger manualArmUp = operatorController.povRight();
  private final Trigger manualArmDown = operatorController.povLeft();

  private final Trigger manualDeployIntake = operatorController.leftTrigger();
  private final Trigger manualRetractIntake = operatorController.rightTrigger();

  //private final Trigger manualExtendArm = operatorController.leftBumper();
  //private final Trigger manualRetractArm = operatorController.rightBumper();
  

  int armIndex = 0;
  boolean isIncreasing = false; 

  PathPlannerTrajectory examplePath = PathPlanner.loadPath("TestPath", new PathConstraints(4, 3));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    //m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, () -> joystick.getY(), () -> joystick.getX()));
    //m_driveSubsystem.setDefaultCommand(new CurveDrive(m_driveSubsystem, driverXbox::getRightY, driverXbox::getLeftX));
    m_intakeSubsystem.setDefaultCommand(intakeCommand);
    //m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, joystick::getY, joystick::getX));
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
    //Operator's Commands
    resetPosition.onTrue(new ResetArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem));
    lowPosition.onTrue(new LowArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem));
    middlePosition.onTrue(new MiddleArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem));
    highPosition.onTrue(new HighArmPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem));
    humanPlayerStation.onTrue(new HumanPlayerStationPosition(m_armExtensionSubsystem, m_pneumaticsSubsystem, m_armSubsystem));
    intakeToggle.onTrue(new InstantCommand(m_intakeSubsystem::increaseConeIndex, m_intakeSubsystem));
    reverseIntakeToggle.onTrue(new InstantCommand(m_intakeSubsystem::increaseCubeIndex, m_intakeSubsystem));
    resetIntakeToggles.onTrue(new InstantCommand(m_intakeSubsystem::resetIndexes, m_intakeSubsystem));

    gyroDrive.onTrue(new GyroDrive(m_driveSubsystem, 1));
    //Operator Manual
    manualArmUp.whileTrue(new RotateArm(m_armSubsystem, 0.2));
    manualArmDown.whileTrue(new RotateArm(m_armSubsystem, -0.2));
    
    //manualExtendArm.whileTrue(new ExtendArm(m_armExtensionSubsystem, 0.2));
    //manualRetractArm.whileTrue(new ExtendArm(m_armExtensionSubsystem, -0.2));
    manualDeployIntake.onTrue(new DeployIntake(m_pneumaticsSubsystem));
    manualRetractIntake.onTrue(new RetractIntake(m_pneumaticsSubsystem));

    //Driver Buttons
    halfSpeed.onTrue(new InstantCommand(m_driveSubsystem::setHalfSpeedTrue, m_driveSubsystem));
    fullSpeed.onTrue(new InstantCommand(m_driveSubsystem::setHalfSpeedFalse, m_driveSubsystem));
    brakeModeOn.whileTrue(
      new InstantCommand(() -> m_driveSubsystem.setBrakeMode(true), m_driveSubsystem))
      .onFalse(
        new InstantCommand(() -> m_driveSubsystem.setBrakeMode(false), m_driveSubsystem));
    balanceBeam.onTrue(new BalancePID(m_driveSubsystem)); 
    coastMode.onTrue(new InstantCommand(() -> m_driveSubsystem.setBrakeMode(false), m_driveSubsystem));
   
  }

  //2, 7, 11, 13 are ones that might work
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
          return new Auto7(m_driveSubsystem, m_armSubsystem, m_pneumaticsSubsystem, m_armExtensionSubsystem, m_intakeSubsystem, m_pneumaticsSubsystem, m_armSubsystem);
        case drive_forwards_score_leave_community_pickup_piece_score:
          return new Auto9(m_driveSubsystem, m_armSubsystem);
        case drive_forwards_score_leave_community_dock_engage:
          return new Auto10(m_driveSubsystem, m_armSubsystem, m_armExtensionSubsystem, m_intakeSubsystem, m_pneumaticsSubsystem);
        case drive_forwards_score_cube_community:
          return new CubeMiddleTaxi(m_driveSubsystem, m_armExtensionSubsystem, m_armSubsystem, m_pneumaticsSubsystem, m_intakeSubsystem);
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
    // An example command will be run in autonomousw
    return new Dock(m_driveSubsystem);
  }
}
