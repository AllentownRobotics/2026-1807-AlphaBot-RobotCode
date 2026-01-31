// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Kraken;

public class Kicker extends SubsystemBase {
  private Kraken kickerMotors;
  /** Creates a new Kicker. */
  public Kicker() {
    kickerMotors = new Kraken(Constants.KickerConstants.MotorIDConstants);
    kickerMotors.setBrakeMode();  

  }

  public void spinKicker(double speed) {
    kickerMotors.setMotorSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
