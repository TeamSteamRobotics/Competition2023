package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.google.gson.Gson;

public class AprilVisionSubsystem extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry t_t6t_rs = table.getEntry("json");
  private LimelightJson thirteenthReason;
  
  public AprilVisionSubsystem() {
    String jsonString = t_t6t_rs.getString("");
    Gson gson = new Gson();
    thirteenthReason = gson.fromJson(jsonString, LimelightJson.class);
  }
  
  public Coordinate getCoordinates() {
    Coordinate coordinate = new Coordinate(); 
    if (thirteenthReason.Results.Fiducial.length != 0) {
      coordinate.aprilTagVisible = true;
      coordinate.x = thirteenthReason.Results.Fiducial[0].t6t_rs[0];
      coordinate.y = thirteenthReason.Results.Fiducial[0].t6t_rs[1];
      coordinate.z = thirteenthReason.Results.Fiducial[0].t6t_rs[2];
      coordinate.rx = thirteenthReason.Results.Fiducial[0].t6t_rs[4];
      coordinate.ry = thirteenthReason.Results.Fiducial[0].t6t_rs[5];
      coordinate.rz = thirteenthReason.Results.Fiducial[0].t6t_rs[6];
    
    } else {
      coordinate.aprilTagVisible = false;
    }
    return coordinate; 
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

  class LimelightJson{
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
