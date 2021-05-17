// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.driveSubsystem.*;
import frc.robot.commands.indexSubsystem.IndexRun;
import frc.robot.commands.indexSubsystem.StopIndex;
import frc.robot.commands.searchSystem.GetAverageDistance;
import frc.robot.commands.shooterSubsystem.*;
import frc.robot.commands.shooterSubsystem.KickerRun;
import frc.robot.commands.shooterSubsystem.SetHoodPosition;
import frc.robot.commands.shooterSubsystem.ShooterRun;
import frc.robot.commands.shooterSubsystem.ShootingRoutine;
import frc.robot.commands.*;
import frc.robot.commands.auto.ShootMoveBack;
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
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final SearchSystem searchSystem = new SearchSystem();
  private final IndexSubsystem indexSubsystem = new IndexSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
  private final Limelight limelightSubsystem = new Limelight("limelight-eleven");

  private Trajectory exampleTrajectory;
  public boolean isQuickTurn = false;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    //setupTrajectory();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Increment drive speed when the right bumper is pressed
    // new JoystickButton(m_driverController, Button.kBumperRight.value)
    //     .whenPressed(() -> m_robotDrive.incMaxSpeed());

    // // Decrement drive speed when the left bumper is pressed
    // new JoystickButton(m_driverController, Button.kBumperLeft.value)
    //     .whenPressed(() -> m_robotDrive.decMaxSpeed());

    // Run the intake when Y is pressed
    new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(() -> shooterSubsystem.IncreaseHoodPosition(0.3))
        .whenReleased(() -> shooterSubsystem.StopHood());

    // B Button
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(() -> shooterSubsystem.LowerHoodPosition(0.3))
        .whenReleased(() -> shooterSubsystem.StopHood());

    // X Button
    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new AutoCenter(m_robotDrive, limelightSubsystem));
        
    //A Button
    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new SetHoodPosition(shooterSubsystem, 30));

    // Right Bumper
    new JoystickButton(m_driverController, 6)
        .whenPressed(() -> kickerSubsystem.KickerRun(0.85))
        .whenReleased(() -> kickerSubsystem.KickerStop());

    // Left Bumper
    new JoystickButton(m_driverController, 5)
        .whenPressed(() -> intakeSubsystem.IntakeRun(0.65))
        .whenReleased(() -> intakeSubsystem.IntakeStop());

    // Start Button
    new JoystickButton(m_driverController, Button.kStart.value)
        .whenPressed(new SetHoodPosition(shooterSubsystem, shooterSubsystem.CalculateAutoAngle(limelightSubsystem.approximateDistance(), 96, ShooterConstants.BALL_VELOCITY*shooterSubsystem.shooterDesiredSpeed)));

    // Right Joystick Button
    new JoystickButton(m_driverController, Button.kStickRight.value)
      .whenPressed(() -> isQuickTurn = !isQuickTurn);      

    
    // Adds the GetAveragedistance command to SmartDashboard
    SmartDashboard.putData(new GetAverageDistance(searchSystem, 3));
  }

  public void periodic() {
    shooterSubsystem.periodic();
    limelightSubsystem.periodic();

    // Left Trigger
    if(m_driverController.getTriggerAxis(Hand.kLeft) == 1) {
        indexSubsystem.AllIndexRun(.8);
    }
    else if(m_driverController.getTriggerAxis(Hand.kLeft) != 1)
        indexSubsystem.AllIndexStop();

    // Right Trigger
    if(m_driverController.getTriggerAxis(Hand.kRight) == 1) {
        shooterSubsystem.ShooterRun(1);
    }
    else if (m_driverController.getTriggerAxis(Hand.kRight) != 1)
        shooterSubsystem.ShooterStop();

    SmartDashboard.putBoolean("isQuickTurn", isQuickTurn);
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
  /*
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
  */
  public Command getAutonomousCommand()
  {
      return new ShootMoveBack(2,.25,.25);
  }

  public Trajectory getTrajectory() {
      return exampleTrajectory;
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
}
