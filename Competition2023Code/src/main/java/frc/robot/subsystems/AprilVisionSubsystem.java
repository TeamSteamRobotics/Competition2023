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

import com.google.gson.Gson;

public class AprilVisionSubsystem extends SubsystemBase {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;

    double[] defaultValue = new double[0];

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry t_t6t_rs = table.getEntry("json");

    public AprilVisionSubsystem() {}
    String jsonString = t_t6t_rs.getString("");
    //System.out.println(jsonString);
    Gson gson = new Gson();
    limelightjson thirteenthReason = gson.fromJson(jsonString, limelightjson.class);

    public Coordinate getCoordinates() {
        Coordinate coordinate = new Coordinate(); 
        if (thirteenthReason.Results.Fiducial.length != 0) {
            coordinate.x = thirteenthReason.Results.Fiducial[0].t6t_rs[0];
            coordinate.y = thirteenthReason.Results.Fiducial[0].t6t_rs[1];
            coordinate.z = thirteenthReason.Results.Fiducial[0].t6t_rs[2];
            coordinate.rx = thirteenthReason.Results.Fiducial[0].t6t_rs[3];
            coordinate.ry = thirteenthReason.Results.Fiducial[0].t6t_rs[4];
            coordinate.rz = thirteenthReason.Results.Fiducial[0].t6t_rs[5];
        } else {
            coordinate.x = 0;
            coordinate.y = 0;
            coordinate.z = 0;
            coordinate.rx = 0;
            coordinate.ry = 0;
            coordinate.rz = 0;
        }
        return coordinate; 
    }


class Coordinate {
    public float x;
    public float y;
    public float z;
    public float rx;
    public float ry;
    public float rz;
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
}


