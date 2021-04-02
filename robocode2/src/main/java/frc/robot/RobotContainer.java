// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.driveSubsystem.*;
import frc.robot.commands.mimicking.*;
import frc.robot.commands.searchSystem.GetAverageDistance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
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
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final SearchSystem searchSystem = new SearchSystem();
  public final IndexSubsystem indexSubsystem = new IndexSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final Mimicking mimicking = new Mimicking();

  public Trajectory exampleTrajectory;
  public int index;
  public boolean isQuickTurn = false;

  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

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
                m_robotDrive.cheesyDrive(
                    (m_driverController.getY(GenericHID.Hand.kLeft) > 0.02 || m_driverController.getY(GenericHID.Hand.kLeft) < -0.02)? m_driverController.getY(GenericHID.Hand.kLeft):0,
                    (m_driverController.getX(GenericHID.Hand.kRight) > 0.02 || m_driverController.getX(GenericHID.Hand.kRight) < -0.02)? -m_driverController.getX(GenericHID.Hand.kRight):0,
                    isQuickTurn),
            m_robotDrive));
        
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
        .whenPressed(() -> shooterSubsystem.IncreaseHoodPosition(.1))
        .whenReleased(() -> shooterSubsystem.StopHood());

    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(() -> shooterSubsystem.LowerHoodPosition(.1))
        .whenReleased(() -> shooterSubsystem.StopHood());

    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(() -> indexSubsystem.AllIndexReverse(.4))
        .whenReleased(() -> indexSubsystem.AllIndexStop());

    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(() -> intakeSubsystem.IntakeReverse(0.85))
        .whenReleased(() -> intakeSubsystem.IntakeStop());

    new JoystickButton(m_driverController, 6)
        .whenPressed(() -> shooterSubsystem.KickerRun(0.85))
        .whenReleased(() -> shooterSubsystem.KickerStop());

    new JoystickButton(m_driverController, 5)
        .whenPressed(() -> intakeSubsystem.IntakeRun(0.65))
        .whenReleased(() -> intakeSubsystem.IntakeStop());

    new JoystickButton(m_driverController, Button.kStart.value)
        .whenPressed(() -> m_robotDrive.emergencyStop(.15));

    new JoystickButton(m_driverController, Button.kStickRight.value)
      .whenPressed(() -> isQuickTurn = !isQuickTurn);
  
    // Adds the GetAveragedistance command to SmartDashboard
    SmartDashboard.putData(new GetAverageDistance(searchSystem, 3));
  }

  /**
   * Runs periodically to allow for path recording
   */
  public void recordingPeriodic() {
    mimicking.record(this);
  }

  public void periodic() {
    shooterSubsystem.periodic();

    if(m_driverController.getTriggerAxis(Hand.kLeft) == 1) {
        indexSubsystem.AllIndexRun(.25);
    }
    else if(m_driverController.getTriggerAxis(Hand.kLeft) != 1)
        indexSubsystem.AllIndexStop();

    if(m_driverController.getTriggerAxis(Hand.kRight) == 1) {
        shooterSubsystem.ShooterRun(1);
    }
    else if (m_driverController.getTriggerAxis(Hand.kRight) != 1)
        shooterSubsystem.ShooterStop();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Playback(mimicking, this);
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
