// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCMD;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final CollectorSubsystem m_CollectorSubsystem = new CollectorSubsystem();
  private final Twindexer m_twindexer = new Twindexer();
  private final Kicker m_kicker = new Kicker();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CollectCMD m_CollectCMD = new CollectCMD(m_CollectorSubsystem);
  private final PositionHoodCMD m_PositionHoodCMD = new PositionHoodCMD(m_ShooterSubsystem);
  private final ShootCMD m_ShootCMD = new ShootCMD(m_ShooterSubsystem, false);
  private final ShootCMD m_SlowShootCMD = new ShootCMD(m_ShooterSubsystem, true);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_DriveSubsystem.setDefaultCommand(new DriveCMD(m_DriveSubsystem, m_driverController));
    m_driverController.start().onTrue(Commands.runOnce(m_DriveSubsystem::zeroHeading));// press start to reset gyro
    m_driverController.x().whileTrue(Commands.run(m_DriveSubsystem::setX));
    
    m_driverController.a().toggleOnTrue(new SetTwindexerSpeed(m_twindexer));
    m_driverController.x().whileTrue(new SetKickerSpeed(m_kicker));

    m_driverController.b().toggleOnTrue(m_CollectCMD);
    //m_driverController.y().whileTrue(m_PositionHoodCMD); there is no hood motor on the alpha bot, so this is just hypothetical code.
    m_driverController.rightTrigger(0.1).and(m_driverController.rightTrigger(0.5).negate()).whileTrue(m_SlowShootCMD);
    m_driverController.rightTrigger(0.5).whileTrue(m_ShootCMD);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
