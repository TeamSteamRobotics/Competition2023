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
    Coordinate coordinate = new Coordinate();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry t_t6t_rs = table.getEntry("json");
    float distanceMultiplier = 0.754f;
    int fidLocation;
    boolean fidLocFound;
    public AprilVisionSubsystem() {}
    Gson gson = new Gson();

    public Coordinate getCoordinates(int targetId) {
        updateCoordinates(targetId);
        return coordinate;
    }

    private void updateCoordinates(int targetId) {
        String jsonString = t_t6t_rs.getString("");
        limelightjson thirteenthReason = gson.fromJson(jsonString, limelightjson.class);
        if (thirteenthReason.Results.Fiducial.length != 0) {
             for (int i = 0; i < thirteenthReason.Results.Fiducial.length; i++) {
                if(thirteenthReason.Results.Fiducial[i].fID == targetId){
                    fidLocation = i;
                    fidLocFound = true;
                    break;
                }else{
                    fidLocFound = false;
                }
            }if(!fidLocFound){
                System.out.println("TARGET FIDUCIAL NOT FOUND!");
                coordinate.aprilTagVisible = false;
            }else{
                coordinate.x = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[0];
                coordinate.y = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[1];
                coordinate.z = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[2] * 0.754f - 0.045f;
                coordinate.rx = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[3];
                coordinate.ry = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[4];
                coordinate.rz = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[5];
                coordinate.aprilTagVisible = true;
            } 
        }else{
            coordinate.aprilTagVisible = false;
            System.out.println("NO FIDUCIALS IN VIEW!");
        }
    }

public class Coordinate {
    public float x;
    public float y;
    public float z;
    public float rx;
    public float ry;
    public float rz;
    public boolean aprilTagVisible;
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

