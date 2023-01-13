// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = limelight.getEntry("tv");
  private NetworkTableEntry ta = limelight.getEntry("ta");
  private NetworkTableEntry tx = limelight.getEntry("tx");
  private NetworkTableEntry ty = limelight.getEntry("ty");


  public VisionSubsystem() {}

  public boolean hasTargets() {
    return tv.getDouble(0) == 1;
  }

  public double getTx() {
    return tx.getDouble(0);
  }

  public double getTy() {
    return ty.getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
