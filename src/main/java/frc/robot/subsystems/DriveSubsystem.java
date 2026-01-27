// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private boolean robotCentered = false;

  private final SwerveModual frontLeft = new SwerveModual(DriveConstants.FrontLeftDriveMotorPort, 
  DriveConstants.FrontLeftTurningMotorPort, 
  DriveConstants.FL_CHASSIS_OFFSET);

  private final SwerveModual frontRight = new SwerveModual(DriveConstants.FrontRightDriveMotorPort,
  DriveConstants.FrontRightTurningMotorPort,
  DriveConstants.FR_CHASSIS_OFFSET);

  private final SwerveModual rearLeft = new SwerveModual(DriveConstants.RearLeftDriveMotorPort,
  DriveConstants.RearLeftTurningMotorPort,
  DriveConstants.BL_CHASSIS_OFFSET);

  private final SwerveModual rearRight = new SwerveModual(DriveConstants.RearRightDriveMotorPort,
  DriveConstants.RearRightTurningMotorPort,
  DriveConstants.BR_CHASSIS_OFFSET);

  private final Pigeon2 gyro = new Pigeon2(9);

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(gyro.getRotation2d().getDegrees()),
    new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      rearLeft.getPosition(),
      rearRight.getPosition()
    }
    );
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      Rotation2d.fromDegrees(gyro.getRotation2d().getDegrees()),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
      });
      
      
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(Rotation2d.fromDegrees(gyro.getRotation2d().getDegrees()), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      rearLeft.getPosition(),
      rearRight.getPosition()
    }, pose);
  }

  PIDController headingController = new PIDController(1.5,0,0);
  
  public double getHeadingAdj(double desiredHeading){
    headingController.enableContinuousInput(-Math.PI,Math.PI);
   return headingController.calculate(getHeading(), desiredHeading);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
    double xSpeedDelivered = Math.round(xSpeed*50.0)/50.0;//adds deadband
    double ySpeedDelivered = Math.round(ySpeed*50.0)/50.0;
    //double rotDelivered = Math.round(rot*-50.0)/50.0;
    //double rotDelivered = getHeading()-rot;
    double rotDelivered = getHeadingAdj(rot);
    SmartDashboard.putNumber("test", rot);

    SwerveModuleState[] swerveModualStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      !robotCentered
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getRotation2d().getDegrees()))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModualStates,
      DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModualStates[0]);
    frontRight.setDesiredState(swerveModualStates[1]);
    rearLeft.setDesiredState(swerveModualStates[2]);
    rearRight.setDesiredState(swerveModualStates[3]);
  }

  public void setX(){
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders(){
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    rearLeft.resetEncoders();
    rearRight.resetEncoders();
  }

  public void zeroHeading(){
    gyro.reset();
    //gyro.setYaw(Rotation2d.fromDegrees(90).getMeasure());
  }

  public double getHeading(){
    return Rotation2d.fromDegrees(gyro.getRotation2d().getDegrees()).getRadians();
  }

  public double getTurnRate(){
    return gyro.getRotation2d().getDegrees() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
