// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;


public class SwerveModual extends SubsystemBase {
  private final SparkFlex driveMotor;
  private final SparkMax steerMotor;

  private final AbsoluteEncoder turningEncoder;
  private final RelativeEncoder driveEncoder;

  private final SparkClosedLoopController drivePID;
  private final SparkClosedLoopController steerPID;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
  /** Creates a new SwerveModual. */
  public SwerveModual(int driveMotorID, int steerMotorID, double chassisAngularOffset) {
    this.driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
    this.steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);

    this.driveEncoder = driveMotor.getEncoder();
    this.turningEncoder = steerMotor.getAbsoluteEncoder();

    this.drivePID = driveMotor.getClosedLoopController();
    this.steerPID = steerMotor.getClosedLoopController();

    SparkFlexConfig driveConfig = new SparkFlexConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();
    double drivingFactor = ModuleConstants.DRIVE_ENCODER_POS_FACTOR;
    double drivingVelocityFeedForward = ModuleConstants.DRIVE_FF;
    double turningFactor = 2 * Math.PI;
    driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .encoder
                  .positionConversionFactor(drivingFactor)
                  .velocityConversionFactor(drivingFactor/60);
    driveConfig
            .closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);
    turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .absoluteEncoder
                      .inverted(ModuleConstants.TURN_ENCODER_INVERTED)
                      .positionConversionFactor(turningFactor)
                      .velocityConversionFactor(turningFactor/60);
    turningConfig
            .closedLoop
                      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                      .pid(1, 0, 0)
                      .outputRange(-1, 1)
                      .positionWrappingEnabled(true)
                      .positionWrappingInputRange(0, turningFactor);
    


    

    this.driveMotor.configure(
    driveConfig, 
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    this.steerMotor.configure(
    turningConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    SwerveModuleState correctDesiredState = new SwerveModuleState();
    correctDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    correctDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

    drivePID.setReference(correctDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    steerPID.setReference(correctDesiredState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = desiredState;
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
