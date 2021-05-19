// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Timer;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  final static CANSparkMax leftMotorLeader = new CANSparkMax(DriveConstants.LEFT_MOTOR_1_PORT, MotorType.kBrushless);
  final CANSparkMax leftMotorFollower = new CANSparkMax(DriveConstants.LEFT_MOTOR_2_PORT, MotorType.kBrushless);

  // The motors on the right side of the drive.
  final static CANSparkMax rightMotorLeader = new CANSparkMax(DriveConstants.RIGHT_MOTOR_1_PORT, MotorType.kBrushless);
  final CANSparkMax rightMotorFollower = new CANSparkMax(DriveConstants.RIGHT_MOTOR_2_PORT, MotorType.kBrushless);

  // The robot's drive
  private static final DifferentialDrive m_drive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

  // The left-side drive encoder
  static final CANEncoder leftLeaderEncoder = leftMotorLeader.getEncoder();

  // The right-side drive encoder
  static final CANEncoder rightLeaderEncoder = rightMotorLeader.getEncoder();

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private double maxVelocity, curvatureMaxCurvature, arcadeMaxCurvature;
  private double[] currentSpeeds = new double[2];

  // NetworkTable Entries for debugging mimicking
  private final NetworkTableEntry speedEntry, curvatureEntry, isArcadeEntry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    speedEntry = NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Drive_Speed");
    curvatureEntry = NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Drive_Quick_Turn");
    isArcadeEntry = NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Drive_Arcade?");

    // Initialize the drivetrain motors

    // Left
    leftMotorLeader.restoreFactoryDefaults();
    leftMotorLeader.setIdleMode(IdleMode.kCoast);  
    leftMotorLeader.setInverted(DriveConstants.LEFT_MOTORS_INVERSED);

    // Left Follower
    leftMotorFollower.restoreFactoryDefaults();
    leftMotorFollower.setIdleMode(IdleMode.kCoast);  
    leftMotorFollower.setInverted(DriveConstants.LEFT_MOTORS_INVERSED);
    leftMotorFollower.follow(leftMotorLeader, false);

    // Right Leader
    rightMotorLeader.restoreFactoryDefaults();
    rightMotorLeader.setIdleMode(IdleMode.kCoast);  
    rightMotorLeader.setInverted(DriveConstants.RIGHT_MOTORS_INVERSED);

    // Right Follower
    rightMotorFollower.restoreFactoryDefaults();
    rightMotorFollower.setIdleMode(IdleMode.kCoast);  
    rightMotorFollower.setInverted(DriveConstants.RIGHT_MOTORS_INVERSED);
    rightMotorFollower.follow(rightMotorLeader, false);

    // Sets the distance per pulse for the encoders
    leftLeaderEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    rightLeaderEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);

    m_gyro.calibrate();

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    maxVelocity = DriveConstants.INITIAL_MAX_VELOCITY;
    curvatureMaxCurvature = DriveConstants.INITIAL_CURVATURE_MAX_CURVATURE;
    arcadeMaxCurvature = DriveConstants.INITIAL_ARCADE_MAX_CURVATURE;
    setMaxOutput();
    setRampRate(DriveConstants.INITIAL_RAMP_RATE);

    Shuffleboard.getTab("Driving").add("Speed Control", DriveConstants.INITIAL_MAX_VELOCITY)
        .withWidget(BuiltInWidgets.kNumberSlider).getEntry()
        .addListener(event -> {
          maxVelocity = event.value.getDouble();
          setMaxOutput();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    Shuffleboard.getTab("Driving").add("Curvature Max Curvature", DriveConstants.INITIAL_CURVATURE_MAX_CURVATURE)
        .withWidget(BuiltInWidgets.kNumberSlider).getEntry()
        .addListener(event -> {
          curvatureMaxCurvature = event.value.getDouble();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    Shuffleboard.getTab("Driving").add("Arcade Max Curvature", DriveConstants.INITIAL_ARCADE_MAX_CURVATURE)
        .withWidget(BuiltInWidgets.kNumberSlider).getEntry()
        .addListener(event -> {
          arcadeMaxCurvature = event.value.getDouble();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    Shuffleboard.getTab("Driving").add("Ramp Rate", DriveConstants.INITIAL_RAMP_RATE)
        .withWidget(BuiltInWidgets.kTextView).getEntry()
        .addListener(event -> {
          setRampRate(event.value.getDouble());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), leftLeaderEncoder.getPosition(), rightLeaderEncoder.getPosition());
  }

  public void emergencyStop(double time) {
    double startLeft = -currentSpeeds[0]/5, startRight = -currentSpeeds[1]/5;
    //double signLeft = startLeft/Math.abs(startLeft), signRight = startRight/Math.abs(startRight);
    
    tankDrive(0, 0);
    Timer timer = new Timer();
    timer.start();
    Timer.delay(.15);

    for ( ; timer.get() < time ; ) {
      tankDrive(startLeft, startRight);
    }

    tankDrive(0,0);
  }

  /**
   * Drives the robot using cheesy drive controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void cheesyDrive(double speed, double curvature, boolean isArcade) {
    if(!isArcade) {
      // Applies a maximum curvature to curvature mode, limiting the minimum turn radius
      m_drive.curvatureDrive(speed, curvature * curvatureMaxCurvature, isArcade);
    } else {
      m_drive.curvatureDrive(speed, curvature * arcadeMaxCurvature, isArcade);
    }

    speedEntry.forceSetDouble(speed);
    curvatureEntry.forceSetDouble(curvature);
    isArcadeEntry.forceSetBoolean(isArcade);
  }

  /**
   * Drives the robot using tank drive controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public static void tankDrive(double leftSpeed, double rightSpeed) {
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

  /**                              **/
  /**     Shuffleboard Methods     **/
  /**                              **/

  /**
   * Sends encoder positions and velocities to SmartDashboard
   */
  public void sendOdometryToShuffleboard() {
    SmartDashboard.putNumber("left_encoder_position", leftLeaderEncoder.getPosition());
    SmartDashboard.putNumber("left_encoder_velocity", leftLeaderEncoder.getVelocity());
    SmartDashboard.putNumber("right_encoder_position", rightLeaderEncoder.getPosition());
    SmartDashboard.putNumber("right_encoder_velocity", rightLeaderEncoder.getVelocity());

    SmartDashboard.putNumber("gyro_angle", m_gyro.getAngle());
    SmartDashboard.putNumber("gyro_angle_rate", m_gyro.getRate());
  }

  /**                 **/
  /**     Setters     **/
  /**                 **/

  /**
   * Sets the max output of the drive to the value of maxOutput;
   */
  public void setMaxOutput() {
    m_drive.setMaxOutput(maxVelocity);
  }

  public void setRampRate(double rampRate) {
    rightMotorLeader.setOpenLoopRampRate(rampRate);
    rightMotorFollower.setOpenLoopRampRate(rampRate);
    leftMotorLeader.setOpenLoopRampRate(rampRate);
    leftMotorFollower.setOpenLoopRampRate(rampRate);
  }

  /**
   * Increments maxOutput by 0.05 and updates the differential drive's
   * max output, maxing at 1.0.
   * 
   * @param maxVelocity
   */
  public void incMaxSpeed() {
    maxVelocity = Math.min(maxVelocity+0.05, 1.0);
    setMaxOutput();
  }

  /**
   * Decreases maxOutput by 0.05 and updates the differential drive's
   * max output, stopping at 0.1.
   * 
   * @param maxVelocity
   */
  public void decMaxSpeed() {
    maxVelocity = Math.max(maxVelocity-0.05, 0.1);
    setMaxOutput();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftLeaderEncoder.setPosition(0);
    rightLeaderEncoder.setPosition(0);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Sets the value of the right motor's inverted boolean to the input value
   */
  public void setRightMotorsInverted(boolean inverted) {
    rightMotorLeader.setInverted(inverted);
    rightMotorFollower.setInverted(inverted);
  }

  /**
   * Sets the value of the right motor's inverted boolean to the input value
   */
  public void setLeftMotorsInverted(boolean inverted) {
    leftMotorLeader.setInverted(inverted);
    leftMotorFollower.setInverted(inverted);
  }

  /**                 **/
  /**     Getters     **/
  /**                 **/

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLeaderEncoder.getVelocity(), rightLeaderEncoder.getVelocity());
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
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public static double getAverageEncoderDistance() {
    return (leftLeaderEncoder.getPosition() + rightLeaderEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the average velocity of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderVelocity() {
    return (leftLeaderEncoder.getVelocity() + rightLeaderEncoder.getVelocity()) / 2.0;
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
}
