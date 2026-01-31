// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Kraken;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final Kraken hoodMotor;
  private final Kraken shooterMotor;
  public ShooterSubsystem() {
    hoodMotor = new Kraken(Constants.MotorConstants.hoodMotorID);
    hoodMotor.setBrakeMode();
    shooterMotor = new Kraken(Constants.MotorConstants.shooterMotorID);
    shooterMotor.setCoastMode();
  }

  public void setHoodSpeed(double speed) {
    hoodMotor.setMotorSpeed(speed); //change to pid when you get the chance
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.setMotorSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
