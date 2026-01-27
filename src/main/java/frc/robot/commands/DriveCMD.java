// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCMD extends Command {
  DriveSubsystem driveSubsystem;
  CommandXboxController controller;
  Rotation2d rotStored;
  /** Creates a new DriveCMD. */
  public DriveCMD(DriveSubsystem driveSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    addRequirements(driveSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //driveSubsystem.drive((controller.getLeftX()/-3), controller.getLeftY()/3, controller.getRightX(), true);

    Translation2d rotPoint = new Translation2d(-controller.getRightX(), controller.getRightY());
    Rotation2d rotation = rotPoint.getAngle();
    if(rotPoint.getDistance(new Translation2d(0,0))<0.75){
      rotation = Rotation2d.fromRadians(driveSubsystem.getHeading());
    }
    /*if (rotPoint.getDistance(new Translation2d(0, 0)) > 0.75) {
        rotation = rotPoint.getAngle();
        rotStored = rotation;
    } else {
        rotation = rotStored;
    }*/

    /*driveSubsystem.drive(controller.getLeftX()/-3, controller.getLeftY()/3, 
    (controller.getRightX()>=0)//when the right stick is positive on the x axis
    ?(Math.atan(controller.getRightY()/controller.getRightX()))//use regular arctangent to get the andgle
    :(Math.atan(controller.getRightY()/controller.getRightX())+3.141592654) % (2*Math.PI),//otherwise add pi to the angle to get the correct angle

    true);*/
    driveSubsystem.drive(
    controller.getLeftX()/-1, 
    controller.getLeftY()/1, 
    rotation.getRadians(),
    true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
