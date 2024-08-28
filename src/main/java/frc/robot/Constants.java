// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final double kDeadbandLeftStick = 0.15;
    public static final double kDeadbandRightStick = 0.19;
  }

  public static class DriveConstants {
    // SwerveKinematics
    public static final double kTrackWidth = Units.inchesToMeters(21.75); // add actual measurement
    public static final double kWheelBase = Units.inchesToMeters(27); // add actual measurement

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATIC = new SwerveDriveKinematics( 
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), // fl
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // fr
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // bl
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // br
    );

    //
    public static final int kFLDriveMotorID = 53; // check ids
    public static final int kFLRotationMotorID = 55;
    //
    public static final int kFRDriveMotorID = 51;
    public static final int kFRRotationMotorID = 57;
    //
    public static final int kBLDriveMotorID = 52;
    public static final int kBLRotationMotorID = 54;
    //
    public static final int kBRDriveMotorID = 50;
    public static final int kBRRotationMotorID = 56;
    //
    public static final int kFLCancoderID = 10; // add actual ids
    public static final int kFRCancoderID = 11; //
    public static final int kBLCancoderID = 13; //
    public static final int kBRCancoderID = 12; //
    // 
    public static final double kFLOffset = -0.121338;
    public static final double kFROffset = 0.440186;
    public static final double kBLOffset = 0.469238;
    public static final double kBROffset = 0.053467;
    //


    public static final double kMaxSpeedMetersPerSec = 1.0; 
  }

  public static class SwerveConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // add real measurement
    public static final double kDriveMotorGearRatio = 1 / 6.75; // add real measurement
    public static final double kRotationMotorGearRatio = 1 / 21.4285714286; // add real measurement
    public static final double kDriveEncoderRotToMeter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kRotationEncoderRotToRad = kRotationMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPMToMetersPerSec = kDriveEncoderRotToMeter / 60;
    public static final double kRotationEncoderRPMToRadsPerSec = kRotationEncoderRotToRad / 60;
    public static final double KPTurning = 0.1; // tune value
    //public static final double KPTurning = 0.05;
  }

}
