// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */




public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final int FrontLeftDriveMotorPort = 2;
    public static final int FrontLeftTurningMotorPort = 1;
    

    public static final int FrontRightDriveMotorPort = 8;
    public static final int FrontRightTurningMotorPort = 7;
    

    public static final int RearLeftDriveMotorPort = 4;
    public static final int RearLeftTurningMotorPort = 3;
    

    public static final int RearRightDriveMotorPort = 6;
    public static final int RearRightTurningMotorPort = 5;
    
    public static final double trackWidth = Units.inchesToMeters(26); // Distance between centers of right and left
                                                                      // wheels on robot
    public static final double wheelBase = Units.inchesToMeters(26); // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =  new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2));;
    public static final double kMaxAngularSpeed = 2.5;
    public static final double kSpeedAt12Volts = 0.5;
    public static final double kMaxSpeedMetersPerSecond = 2.5;
    public static final boolean kGyroReversed = false;

    public static final double FL_CHASSIS_OFFSET = -Math.PI / 2;
    public static final double FR_CHASSIS_OFFSET = 0.0; 
    public static final double BL_CHASSIS_OFFSET = Math.PI; 
    public static final double BR_CHASSIS_OFFSET = Math.PI / 2;
  }
  public static final class NeoMotorConstants {
      public static final double NEO_FREE_SPEED = 5676;
    }
  
    public static final class ModuleConstants{
      /*pinion gear teeth */    
      public static final int DRIVE_MOTOR_TEETH = 14;  
  
      public static final boolean TURN_ENCODER_INVERTED = true;
  
  //Calculations for drive motor conversion factors and feed forwards
      public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.NEO_FREE_SPEED / 60;
      public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(2.85);
      public static final double WHEEL_CIRCUMFRENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
      public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_TEETH * 15);
      public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFRENCE_METERS) / DRIVE_MOTOR_REDUCTION;
  
      public static final double DRIVE_ENCODER_POS_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
      / DRIVE_MOTOR_REDUCTION; // meters
      public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
      / DRIVE_MOTOR_REDUCTION) / 60.0; // meters per second
  
      public static final double TURN_ENCODER_POS_FACTOR = (2 * Math.PI); // radians
      public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second
  
      public static final double TURN_ENCODER_POS_MIN_INPUT = 0; // radians
      public static final double TURN_ENCODER_POS_MAX_INPUT = TURN_ENCODER_POS_FACTOR; // radians
  
      //drive motor PID
      public static final double DRIVE_P = 0.06;
      public static final double DRIVE_I = 0.0001;
      public static final double DRIVE_D = 0;
      public static final double DRIVE_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
      public static final double DRIVE_MIN_OUTPUT = -1;
      public static final double DRIVE_MAX_OUTPUT = 1;
  
      //turn motor PID
      public static final double TURN_P = 1;
      public static final double TURN_I = 0;
      public static final double TURN_D = 0;
      public static final double TURN_FF = 0;
      public static final double TURN_MIN_OUTPUT = -1;
      public static final double TURN_MAX_OUTPUT = 1;
  
      public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kBrake;
      public static final IdleMode TURN_MOTOR_IDLE_MODE = IdleMode.kBrake;
  
      public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50; // amps
      public static final int TURN_MOTOR_CURRENT_LIMIT = 20; // amps
  }
}
