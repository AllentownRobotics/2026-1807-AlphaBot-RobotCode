// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectCMD extends Command {
  /** Creates a new CollectCMD. */
  private CollectorSubsystem collectorMotor;
  public CollectCMD(CollectorSubsystem collectorMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collectorMotor = collectorMotor;
    addRequirements(collectorMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //sets collectorMotor's speed to the value given in Constants
    collectorMotor.setCollectorSpeed(Constants.CollectorConstants.collectorMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //sets collectorMotor's speed to 0
    collectorMotor.setCollectorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
