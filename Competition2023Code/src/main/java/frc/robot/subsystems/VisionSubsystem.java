// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TreeMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PipelineType;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry pipeline = limelight.getEntry("pipeline");
  private NetworkTableEntry tv = limelight.getEntry("tv");
  private NetworkTableEntry ta = limelight.getEntry("ta");
  private NetworkTableEntry tx = limelight.getEntry("tx");
  private NetworkTableEntry ty = limelight.getEntry("ty");


  public VisionSubsystem() {}

  public PipelineType getCurrentPipeline() {
    switch(pipeline.getNumber(0).intValue()) {
      case 0:
        return PipelineType.APRILTAG;
      case 1:
        return PipelineType.RETRO_REFLECTIVE_TAPE;
      default:
        System.out.println("The pipeline was unknown. Number: " + pipeline.getDouble(0));
        return PipelineType.ERROR_UNKNOWN_PIPELINE;
    }
  }

  public void selectPipeline(PipelineType type) {
    switch (type) {
      case APRILTAG:
        pipeline.setNumber(0);
        break;
      case RETRO_REFLECTIVE_TAPE:
        pipeline.setNumber(1);
        break;
      default:
        break;
    }
  }

  public boolean hasTargets() {
    return tv.getDouble(0) == 1;
  }

  public double getTa() {
    return ta.getDouble(0);
  }

  public double getTx() {
    return tx.getDouble(0);
  }

  public double getTy() {
    return ty.getDouble(0);
  }

  /**
   * Calculates the distance to the target based on angle of target to camera and physical constants of the robot and field
   * @return distance in meters to the middle target
   */
  public double distanceToMiddleConeNode() {
    double angleToTargetRadians = ((VisionConstants.kLimelightMountingAngle + ty.getDouble(0)) * (Math.PI / 180));
    double distance = (FieldConstants.kMiddleConeNodeVisionTapeHeightMeters - VisionConstants.kLimelightHeightMeters) / Math.tan(angleToTargetRadians);
    return distance;
  }

  /**
   * Calculates the distance to the target based on angle of target to camera and physical constants of the robot and field
   * @return distance in meters to the top target
   */
  public double distanceToTopConeNode() {
    double angleToTargetRadians = ((VisionConstants.kLimelightMountingAngle + ty.getDouble(0)) * (Math.PI / 180));
    double distance = (FieldConstants.kTopConeNodeVisionTapeHeightMeters - VisionConstants.kLimelightHeightMeters) / Math.tan(angleToTargetRadians);
    return distance;
  }

  public double distanceToGridAprilTag() {
    double angleToTagRadians = ((VisionConstants.kLimelightMountingAngle + ty.getDouble(0)) * (Math.PI / 180));
    double distance = (0.23622) / Math.tan(angleToTagRadians);
    System.out.println("Distance: " + distance);
    return distance;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
