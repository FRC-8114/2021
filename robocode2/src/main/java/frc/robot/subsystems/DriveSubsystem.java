// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  final CANSparkMax leftMotorLeader = new CANSparkMax(Constants.DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  final CANSparkMax leftMotorFollower = new CANSparkMax(Constants.DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

  // The motors on the right side of the drive.
  final CANSparkMax rightMotorLeader = new CANSparkMax(Constants.DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  final CANSparkMax rightMotorFollower = new CANSparkMax(Constants.DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

  // The left-side drive encoder
  final CANEncoder leftLeaderEncoder = leftMotorLeader.getEncoder();

  // The right-side drive encoder
  final CANEncoder rightLeaderEncoder = rightMotorLeader.getEncoder();

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private double maxOutput;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Initialize the moroters
    // Left Leader
    leftMotorLeader.restoreFactoryDefaults(); 
    leftMotorLeader.setIdleMode(IdleMode.kBrake);  
    leftMotorLeader.setInverted(Constants.DriveConstants.kLeftMotorsInversed);

    // Left Follower
    leftMotorFollower.restoreFactoryDefaults(); 
    leftMotorFollower.setIdleMode(IdleMode.kBrake);  
    leftMotorFollower.setInverted(Constants.DriveConstants.kLeftMotorsInversed);
    leftMotorFollower.follow(leftMotorLeader, false);

    // Right Leader
    rightMotorLeader.restoreFactoryDefaults(); 
    rightMotorLeader.setIdleMode(IdleMode.kBrake);  
    rightMotorLeader.setInverted(Constants.DriveConstants.kRightMotorsInversed);

    // Right Follower
    rightMotorFollower.restoreFactoryDefaults(); 
    rightMotorFollower.setIdleMode(IdleMode.kBrake);  
    rightMotorFollower.setInverted(Constants.DriveConstants.kRightMotorsInversed);
    rightMotorFollower.follow(rightMotorLeader, false);

    // Sets the distance per pulse for the encoders
    leftLeaderEncoder.setPositionConversionFactor(Constants.DriveConstants.kEncoderDistancePerPulse);
    rightLeaderEncoder.setPositionConversionFactor(Constants.DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    maxOutput = 1.0;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), leftLeaderEncoder.getPosition(), rightLeaderEncoder.getPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLeaderEncoder.getVelocity(), rightLeaderEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorLeader.setVoltage(leftVolts);
    rightMotorLeader.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftLeaderEncoder.setPosition(0);
    rightLeaderEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Gets the left encoder's distance
   * 
   * @return the left encoder's distance 
   */
  public double getLeftEncoderDistance() {
    return leftLeaderEncoder.getPosition() * (DriveConstants.kLeftEncoderReversed? -1:1);
  }

  /**
   * Gets the left encoder's velocity
   * 
   * @return the left encoder's velocity
   */
  public double getLeftEncoderVelocity() {
    return leftLeaderEncoder.getVelocity() * (DriveConstants.kLeftEncoderReversed? -1:1);
  }

  /**
   * Gets the right encoder's distance
   * 
   * @return the right encoder's distance 
   */
  public double getRightEncoderDistance() {
    return rightLeaderEncoder.getPosition() * (DriveConstants.kRightEncoderReversed? -1:1);
  }

  /**
   * Gets the right encoder's velocity
   * 
   * @return the right encoder's velocity
   */
  public double getRightEncoderVelocity() {
    return rightLeaderEncoder.getVelocity() * (DriveConstants.kRightEncoderReversed? -1:1);
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return leftLeaderEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return rightLeaderEncoder;
  }

  /**
   * Increments maxOutput by 0.05 and updates the differential drive's
   * max output, maxing at 1.0.
   * 
   * @param maxOutput
   */
  public void incMaxSpeed() {
    maxOutput = Math.min(maxOutput+0.05, 1.0);
    setMaxOutput();
  }

  /**
   * Decreases maxOutput by 0.05 and updates the differential drive's
   * max output, stopping at 0.1.
   * 
   * @param maxOutput
   */
  public void decMaxSpeed() {
    maxOutput = Math.max(maxOutput-0.05, 0.1);
    setMaxOutput();
  }

  /**
   * Sets the max output of the drive to the value of maxOutput;
   */
  public void setMaxOutput() {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  /**
   * Sends encoder positions and velocities to SmartDashboard
   */
  public void sendEncodersToShuffleboard() {
    SmartDashboard.putNumber("left_encoder_position", getLeftEncoderDistance());
    SmartDashboard.putNumber("left_encoder_velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("right_encoder_position", getRightEncoderDistance());
    SmartDashboard.putNumber("right_encoder_velocity", getRightEncoderVelocity());
  }
}
