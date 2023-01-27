// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.google.gson.Gson;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;

  double[] defaultValue = new double[0];

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry t_t6t_rs = table.getEntry("json");
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    String jsonString = t_t6t_rs.getString("");
    //System.out.println(jsonString);
    Gson gson = new Gson();
    limelightjson thirteenthReason = gson.fromJson(jsonString, limelightjson.class);

    x = thirteenthReason.Results.Fiducial[0].t6t_rs[0];
    y = thirteenthReason.Results.Fiducial[0].t6t_rs[1];
    z = thirteenthReason.Results.Fiducial[0].t6t_rs[2];
    rx = thirteenthReason.Results.Fiducial[0].t6t_rs[3];
    ry = thirteenthReason.Results.Fiducial[0].t6t_rs[4];
    rz = thirteenthReason.Results.Fiducial[0].t6t_rs[5];

    System.out.println("X: " + x);
    System.out.println("Y: " + y);
    System.out.println("Z: " + z);
    System.out.println("RX: " + rx);
    System.out.println("RY: " + ry);
    System.out.println("RZ: " + rz);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
class limelightjson{
    public ResultJson Results;
}

class ResultJson
{
    public FiducialJson[] Fiducial;
    public int pID;
    public float tl;
    public float ts;
    public int v;
}

class FiducialJson
{
    public int fID;
    public String fam;
    public float[] t6t_rs;
}
