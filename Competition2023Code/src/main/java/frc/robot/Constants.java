// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0; 
  }

  public static class DriveStraightPIDConstants {
    public static final double kP = .5;
    public static final double kI = 0;
    public static final double kD = .1;
  }

  public static class VisionConstants {
    public static final double kLimelightMountingAngle = 0.0; // angle of limelight to being perfectly vertical in degrees
    public static final double kLimelightHeightMeters = 0.0; // distance from middle of lense of limelight to ground in meters    
  }

  public static class FieldConstants {
    public static final double kMiddleConeNodeVisionTapeHeightMeters = 0.56; // 56 centimeters to bottom
    public static final double kTopConeNodeVisionTapeHeightMeters = 1.06; // 106 centimeters to bottom
    public static final double kGridAprilTagHeightMeters = 0.59; // 59 centimeters to bottom
  }
  public static class DriveConstants {
    public static final double driveConversionFactor = 0;
    public static final double encoderTicksToMeters = 0; // Will be filled out when more info arrives
    public static final double wheelRadiusMeters = 0.0762;

    //test constants, need to determine for real
    public static final double ksVolts = .2;
    public static final double ksVoltSecondsPerMeter = 3.3;
    public static final double kvVoltSecondsSquaredPerMeter = 0.67;
    public static final double kPDriveVel = 4.6; 

  }
  public static class DriveRotationConstants {
    public static final double drive_kP = 0;
    public static final double drive_kI = 0;
    public static final double drive_kD = 0;
    public static final double drive_tolerance = 3; 

    public static int rightFrontMotorID;
    public static int rightBackMotorID;
    public static int leftFrontMotorID;
    public static int leftBackMotorID;
  }

  public static class GyroTurnConstants {
    public static final double kP = 0.3;
    public static final double kI = 0;
    public static final double kD = 0.1;

    public static final double tolerance = 5; //in degrees
  }

  public static class BalanceConstants {
    public static final double kP = .3; 
    public static final double kI = 0;
    public static final double kD = 0; 

    public static final double tolerance = .01; 
  }

  public static class EncoderDriveDistanceConstants {
    public static final double kP = 0.2;
    public static final double kI = 0.1;
    public static final double kD = 0.00001;
  }

  public static class GyroDriveConstants{
    public static final double kP = 0.2;
    public static final double kI = 0.05;
    public static final double kD = 0.1;

    public static final double tolerance = 0.01;
  }

  public static class ArmConstants {
    public static final double angle_kP = 1.3; //1.9
    public static final double angle_kI = 0.35; //.7
    public static final double angle_kD = 0.05; // 67.6 / 100; //.3

    public static final double low_angle_kP = 1.1; //1.9
    public static final double low_angle_kI = 0.2; //.7
    public static final double low_angle_kD = 0.05; // 67.6 / 100; //.3

    public final static double[] positions = {Math.PI / 4, Math.PI / 3, Math.PI /2 }; 

    public static final double length_kP = 3.5;
    public static final double length_kI = 0.35;
    public static final double length_kD = 0.3;

    public static final double anglePIDTolerance = .05;
    public static final double lengthPIDTolerance = 0.000001;

    public static final double extendArmPIDoffset= 0.1739;

    public static final double armConversionFactor = 0.0652149; //Meters per rotation
    public static final double maxArmLengthMeters = 0.6;

    public static final double intakeSpeed = 0.65 * 0.7;

    public static final double resetPosition = 0.53677233; //0.67;
    public static final double lowPosition = 0.82; //.85; //0.7119198;
    public static final double middlePosition = 1.52; //1.52 //1.62316;
    public static final double highPosition = 1.62;//1.72;//1.65; //1.8; //constants not finished

    public static final double resetPositionLength = 0.01;
    public static final double lowPositionLength = 0.35; //0.355; //0.25; 
    public static final double middlePositionLength = .1475;//0.172;
    public static final double highPositionLength = .5869;//.5119; //.45;//0.43;

    

  }

  public final class MotorIDConstants {
    public static final short leftBackDrive = 3;
    public static final short leftFrontDrive = 2;
    public static final short rightBackDrive = 9;
    public static final short rightFrontDrive = 8;

    public static final short intakeMotor = 5;
    public static final short leftShoulderMotor = 4;
    public static final short rightShoulerMotor = 7;

    public static final short elevatorMotor = 6;
  }
}
