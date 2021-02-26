// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.driveSubsystem.*;
import frc.robot.commands.searchSystem.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final SearchSystem searchSystem = new SearchSystem();

  private Trajectory exampleTrajectory;
  private int index;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.tankDrive(
                    m_driverController.getY(GenericHID.Hand.kLeft),
                    m_driverController.getY(GenericHID.Hand.kRight)),
            m_robotDrive));

    setupTrajectory();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Increment drive speed when the right bumper is pressed
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenPressed(() -> m_robotDrive.incMaxSpeed());
    // Decrement drive speed when the left bumper is pressed
    new JoystickButton(m_driverController, Button.kBumperLeft.value)
        .whenPressed(() -> m_robotDrive.decMaxSpeed());
    // Reset odometry when Y is pressed
    new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(new ResetOdometry(m_robotDrive));
    
    // Adds the GetAveragedistance command to SmartDashboard
    SmartDashboard.putData(new GetAverageDistance(searchSystem, 3));
    SmartDashboard.putData("incIndex", new IncTrajectoryStateIndex(this));
    SmartDashboard.putData("decIndex", new DecTrajectoryStateIndex(this)));
  }

  /**
   * Creates the proper objects to initialize the trajectory so the trajectory can be ready before
   * the autonomous command is requested
   */
  public void setupTrajectory() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.KS_VOLTS,
                DriveConstants.KV_VOLT_SECONDS_PER_METER,
                DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            DriveConstants.DRIVE_KINEMATICS,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                DriveConstants.KS_VOLTS,
                DriveConstants.KV_VOLT_SECONDS_PER_METER,
                DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            DriveConstants.DRIVE_KINEMATICS,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

  /**
   * Increments the index from which the Trajectory.State is derived, constrained within the number
   * of Trajectory.States avalable
   */
  public void incIndex() {
      index++;
      index %= exampleTrajectory.getStates().size();
  }

  /**
   * Increments the index from which the Trajectory.State is derived, constrained within the number
   * of Trajectory.States avalable
   */
  public void decIndex() {
    index--;
    index %= exampleTrajectory.getStates().size();
  }

  /**
   * Returns the autonomous trajectory
   * 
   * @return the autonomous trajectory
   */
  public Trajectory getTrajectory() {
      return exampleTrajectory;
  }

  /**
   * Returns a specific trajectory state of the autonomous trajectory
   * 
   * @param index the index of the state to get from Trajectory.getStates
   * @return the state at the specified index
   */
  public Trajectory.State getTrajectoryState(int index) {
      return exampleTrajectory.getStates().get(index%exampleTrajectory.getStates().size());
  }

  /**
   * Returns the trajectory state of the autonomous trajectory at index
   * 
   * @return the state at index
   */
  public Trajectory.State getTrajectoryStateAtIndex() {
      return exampleTrajectory.getStates().get(index);
  }

  /**
   * Returns the drive subsystem object used to communicate with the 
   * drivetrain
   * 
   * @return the robots drive subsystem
   */
  public DriveSubsystem getDriveSystem() {
      return m_robotDrive;
  }

  /**
   * Returns the search subsystem object used to calculate the distance
   * to the power cell
   * 
   * @return the search system
   */
  public SearchSystem getSearchSystem() {
      return searchSystem;
  }

  /**
   * Returns the index of the trajectory from which the state is given
   * 
   * @return the index
   */
  public int getIndex() {
      return index;
  }
}
