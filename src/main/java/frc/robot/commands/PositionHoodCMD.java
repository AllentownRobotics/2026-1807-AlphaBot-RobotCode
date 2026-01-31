// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionHoodCMD extends Command {
  /** Creates a new PositionHoodCMD. */
  private ShooterSubsystem hoodMotor;
  public PositionHoodCMD(ShooterSubsystem m_ShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hoodMotor = hoodMotor;
    addRequirements(hoodMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hoodMotor.setHoodSpeed(Constants.ShooterConstants.hoodMotorSpeed); //this should be changed to a PID controller, for now this is a placeholder
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //sets hoodMotor's speed to the value given in Constants
    hoodMotor.setHoodSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
