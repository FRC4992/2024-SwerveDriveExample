// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {

  public SwerveModule FLModule = new SwerveModule(
    DriveConstants.kFLDriveMotorID, 
    DriveConstants.kFLRotationMotorID, 
    DriveConstants.kFLCancoderID, 
    false, 
    true, 
    DriveConstants.kFLOffset, // abs enc offset, add to constants, paramter here
    false); // abs enc reversed, add to constants, parameter here

  public SwerveModule FRModule = new SwerveModule(
    DriveConstants.kFRDriveMotorID, 
    DriveConstants.kFRRotationMotorID, 
    DriveConstants.kFRCancoderID, 
    true, 
    true, 
    DriveConstants.kFROffset, // abs enc offset, add to constants, paramter here
    false); // abs enc reversed, add to constants, parameter here

  public SwerveModule BLModule = new SwerveModule(
    DriveConstants.kBLDriveMotorID, 
    DriveConstants.kBLRotationMotorID, 
    DriveConstants.kBLCancoderID, 
    false, 
    true, 
    DriveConstants.kBLOffset, // abs enc offset, add to constants, paramter here
    false); // abs enc reversed, add to constants, parameter here

  public SwerveModule BRModule = new SwerveModule(
    DriveConstants.kBRDriveMotorID, 
    DriveConstants.kBRRotationMotorID, 
    DriveConstants.kBRCancoderID, 
    true, 
    true, 
    DriveConstants.kBROffset, // abs enc offset, add to constants, paramter here
    false); // abs enc reversed, add to constants, parameter here

  public AHRS navx = new AHRS(SPI.Port.kMXP);

  //SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DriveConstants.SWERVE_DRIVE_KINEMATIC, getRotation2d(), getPositions());
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DriveConstants.SWERVE_DRIVE_KINEMATIC, new Rotation2d(0), getPositions());
  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.SWERVE_DRIVE_KINEMATIC, 
    new Rotation2d(0), 
    getPositions(), 
    new Pose2d());

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    setBrakeMode();

    // for field-oriented swerve, resets navx on robot init
    new Thread(() -> { // new thread, doesn't interfere with anything else
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {}
    }).start();
  }

  public void zeroHeading() {
    navx.reset();
  }

  public double getHeading() {
    //return Math.IEEEremainder(navx.getYaw(), 360); // clamps value within -180 to 180 deg
    return -navx.getYaw();
  } // from getAngle()

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading()); // (1e) changed from negative
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = FLModule.getPosition();
    positions[1] = FRModule.getPosition();
    positions[2] = BLModule.getPosition();
    positions[3] = BRModule.getPosition();
    return positions;
  }

  // public Pose2d getPose2d() {
  //   return odometry.getPoseMeters();
  // }

  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  // public void resetPose(Pose2d pose) {
  //   odometry.resetPosition(getRotation2d(), getPositions(), pose);
  // }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    ChassisSpeeds chassisSpeeds = Constants.DriveConstants.SWERVE_DRIVE_KINEMATIC.toChassisSpeeds(
      getStates()
    );
    return chassisSpeeds;
  }

  public void setStates(ChassisSpeeds chassisSpeeds) {
    RobotContainer.swerve.setModuleStates(Constants.DriveConstants.SWERVE_DRIVE_KINEMATIC.toSwerveModuleStates(chassisSpeeds));
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = {
      FLModule.getState(),
      FRModule.getState(),
      BLModule.getState(),
      BRModule.getState()
    };
    return states;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //odometry.update(getRotation2d(), getPositions());
    
    poseEstimator.update(getRotation2d(), getPositions());
    
    // update by adding vision measurement

    // SmartDashboard.putNumber("Heading", getHeading());
    // SmartDashboard.putString("Robot Location", getPose2d().getTranslation().toString());
    SmartDashboard.putNumber("[DRIVE] Output Current [FL]", FLModule.driveMotor.getOutputCurrent());
    SmartDashboard.putNumber("[DRIVE] Output Current [FR]", FRModule.driveMotor.getOutputCurrent());
    SmartDashboard.putNumber("[DRIVE] Output Current [BL]", BLModule.driveMotor.getOutputCurrent());
    SmartDashboard.putNumber("[DRIVE] Output Current [BR]", BRModule.driveMotor.getOutputCurrent());
    SmartDashboard.putNumber("[ROTATION] Output Current [FL]", FLModule.driveMotor.getOutputCurrent());
    SmartDashboard.putNumber("[ROTATION] Output Current [FR]", FRModule.driveMotor.getOutputCurrent());
    SmartDashboard.putNumber("[ROTATION] Output Current [BL]", BLModule.driveMotor.getOutputCurrent());
    SmartDashboard.putNumber("[ROTATION] Output Current [BR]", BRModule.driveMotor.getOutputCurrent());
  
  }

  public void resetEncoders() {
    FLModule.resetEncoders();
    FRModule.resetEncoders();
    BLModule.resetEncoders();
    BRModule.resetEncoders();
  }

  public void stopModules() {
    FLModule.stop();
    FRModule.stop();
    BLModule.stop();
    BRModule.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSec);
    FLModule.setDesiredState(desiredStates[0]);
    FRModule.setDesiredState(desiredStates[1]);
    BLModule.setDesiredState(desiredStates[2]);
    BRModule.setDesiredState(desiredStates[3]);
  }

  public void setBrakeMode() {
    FLModule.setBrakeMode();
    FRModule.setBrakeMode();
    BLModule.setBrakeMode();
    BRModule.setBrakeMode();
  }

}
