// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_MOTOR_1_PORT = 2;
    public static final int LEFT_MOTOR_2_PORT = 1;
    public static final int RIGHT_MOTOR_1_PORT = 3;
    public static final int RIGHT_MOTOR_2_PORT = 4;
    public static final boolean LEFT_MOTORS_INVERSED = true;
    public static final boolean RIGHT_MOTORS_INVERSED = true;

    public static final boolean LEFT_ENCODER_REVERSED = true;
    public static final boolean RIGHT_ENCODER_REVERSED = true;
    public static final double TRACKWIDTH_METERS = 1.2387716;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
        new DifferentialDriveKinematics(TRACKWIDTH_METERS);

    public static final int ENCODER_CPR = 1024;
    public static final double POSITION_CONVERSION_FACTOR = 100;
    public static final double VELOCITY_CONVERSION_FACTOR = .001;
    public static final double WHEEL_DIAMETER_METERS = 0.15;
    public static final double ENCODER_DISTANCE_PER_PULSE =
        // Assumes the encoders are directly mounted on the wheel shafts
        (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR * POSITION_CONVERSION_FACTOR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double KS_VOLTS = 2.85;
    public static final double KV_VOLT_SECONDS_PER_METER = 0.197;
    public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.998;

    // Example value only - as above, this must be tuned for your drive!
    public static final double KP_DRIVE_VEL = 0.00206;
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class AutoConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = .5;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = .05;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_CONTROLLER_PORT = 8;
  }

  public static final class IndexConstants {
    public static final int FRONT_INDEX_CONTROLLER_PORT = 12;
    public static final int TOWER_INDEX_CONTROLLER_PORT = 13;
  }

  public static final class ShooterConstants {
    public static final int LEFT_SHOOTER_CONTROLLER_PORT = 17;
    public static final int RIGHT_SHOOTER_CONTROLLER_PORT = 18;
    public static final int KICKER_CONTROLLER_PORT = 19;
    public static final int HOOD_CONTROLLER_PORT = 20;

    public static final int ENCODER_DISTANCE_PER_PULSE = 0;
    public static final int VELOCITY_CONVERSION_FACTOR = 0;
  }
}
