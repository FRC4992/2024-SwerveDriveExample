// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

  CANSparkMax driveMotor;
  CANSparkMax rotationMotor;
  
  RelativeEncoder distanceEncoder;
  RelativeEncoder rotationEncoder;
  public CANcoder absoluteEncoder;
  CANcoderConfiguration canCoderConfig;

  boolean absoluteEncoderReversed;
  double absoluteEncoderOffsetRaw;
  double absoluteEncoderOffsetDeg;
  double absoluteEncoderOffsetRad;

  PIDController rotationPIDController;

  SwerveModuleState currentState;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int rotationMotorID, int canCoderID, boolean driveMotorReversed, 
                      boolean rotationMotorReversed, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

                        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);
    
    driveMotor.setInverted(driveMotorReversed);
    rotationMotor.setInverted(rotationMotorReversed);

    distanceEncoder = driveMotor.getEncoder();
    rotationEncoder = rotationMotor.getEncoder();
    absoluteEncoder = new CANcoder(canCoderID);

    distanceEncoder.setPositionConversionFactor(SwerveConstants.kDriveEncoderRotToMeter);
    distanceEncoder.setVelocityConversionFactor(SwerveConstants.kDriveEncoderRPMToMetersPerSec);
    rotationEncoder.setPositionConversionFactor(SwerveConstants.kRotationEncoderRotToRad);
    rotationEncoder.setVelocityConversionFactor(SwerveConstants.kRotationEncoderRPMToRadsPerSec);

    rotationPIDController = new PIDController(SwerveConstants.KPTurning, 0, 0);
    rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);


    this.absoluteEncoderOffsetRaw = absoluteEncoderOffset;
    this.absoluteEncoderOffsetDeg = this.absoluteEncoderOffsetRaw * 360;
    this.absoluteEncoderOffsetRad = this.absoluteEncoderOffsetRaw * 2 * Math.PI;

    driveMotor.setSmartCurrentLimit(40);
    rotationMotor.setSmartCurrentLimit(40);
    driveMotor.burnFlash();
    rotationMotor.burnFlash();

    resetEncoders();
  }

  public double getDistancePosition() {
    return distanceEncoder.getPosition();
  }

  public double getRotationPosition() {
    return rotationEncoder.getPosition();
  }

  public double getDistanceVelocity() {
    return distanceEncoder.getVelocity();
  }

  public double getRotationVelocity() {
    return rotationEncoder.getVelocity();
  }
  
  public StatusSignal<Double> getAbsoluteEncoderPositionRaw() { 
    return absoluteEncoder.getAbsolutePosition(); // find out what units this position is in (rotations)
  }

  public double getAbsoluteEncoderDeg() { // rotations as double, convert to rad or deg
    return getAbsoluteEncoderPositionRaw().getValue() * 360; // gets value as double
  }

  public double getAbsoluteEncoderRad() {  
    // check: (changes to radians, add/subtract absolute offset)
    if(absoluteEncoderReversed) {
      return -Math.toRadians(getAbsoluteEncoderDeg()) - this.absoluteEncoderOffsetRad;
    } else {
      return Math.toRadians(getAbsoluteEncoderDeg()) - this.absoluteEncoderOffsetRad;
    }
  }

  public void resetEncoders() {
    distanceEncoder.setPosition(0);
    rotationEncoder.setPosition(getAbsoluteEncoderRad()); // check units
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDistanceVelocity(), new Rotation2d(getRotationPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistancePosition(), new Rotation2d(getRotationPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if(Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    currentState = state;
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSec); // change to increase speed
    rotationMotor.set(rotationPIDController.calculate(getRotationPosition(), state.angle.getRadians()));
    
    // SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());

  }

  public void stop() {
    driveMotor.set(0);
    rotationMotor.set(0);
  }

  public void setBrakeMode() {
    driveMotor.setIdleMode(IdleMode.kBrake);
    rotationMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
