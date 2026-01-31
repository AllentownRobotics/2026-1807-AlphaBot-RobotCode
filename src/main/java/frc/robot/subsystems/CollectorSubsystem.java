// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Kraken;

public class CollectorSubsystem extends SubsystemBase {
  /** Creates a new CollectorSubsystem. */
  private final Kraken collectorMotor;
  public CollectorSubsystem() {
    collectorMotor = new Kraken(Constants.MotorConstants.collectorMotorID);
    collectorMotor.setBrakeMode();
  }

  public void setCollectorSpeed(double speed) {
    collectorMotor.setMotorSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
