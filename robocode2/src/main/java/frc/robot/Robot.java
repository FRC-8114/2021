// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.driveSubsystem.*;
import frc.robot.subsystems.SearchSystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final SendableChooser<Command> right_motors_inverted = new SendableChooser<Command>();
  private final SendableChooser<Command> right_encoder_inverted = new SendableChooser<Command>();
  private final SendableChooser<Command> left_motors_inverted = new SendableChooser<Command>();
  private final SendableChooser<Command> left_encoder_inverted = new SendableChooser<Command>();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Send values to Shuffleboard
    m_robotContainer.getDriveSystem().sendEncodersToShuffleboard();
    m_robotContainer.getSearchSystem().sendEstimatedDistance();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // Sets up Chooser for inverting of right motors
    right_motors_inverted.setDefaultOption("False", new SetRightMotorsInverted(m_robotContainer.getDriveSystem(), false));
    right_motors_inverted.addOption("True", new SetRightMotorsInverted(m_robotContainer.getDriveSystem(), true));
    SmartDashboard.putData("Right Motors Inverted", right_motors_inverted);

    // Sets up Chooser for inverting of right leader encoder
    right_encoder_inverted.setDefaultOption("False", new SetRightEncoderInverted(m_robotContainer.getDriveSystem(), false));
    right_encoder_inverted.addOption("True", new SetRightEncoderInverted(m_robotContainer.getDriveSystem(), true));
    SmartDashboard.putData("Right Encoder Inverted", right_encoder_inverted);

    // Sets up Chooser for inverting of left motors
    left_motors_inverted.setDefaultOption("False", new SetLeftMotorsInverted(m_robotContainer.getDriveSystem(), false));
    left_motors_inverted.addOption("True", new SetLeftMotorsInverted(m_robotContainer.getDriveSystem(), true));
    SmartDashboard.putData("Left Motors Inverted", left_motors_inverted);

    // Sets up Chooser for inverting of left leader encoder
    left_encoder_inverted.setDefaultOption("False", new SetLeftEncoderInverted(m_robotContainer.getDriveSystem(), false));
    left_encoder_inverted.addOption("True", new SetLeftEncoderInverted(m_robotContainer.getDriveSystem(), true));
    SmartDashboard.putData("Left Encoder Inverted", left_encoder_inverted);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    right_motors_inverted.getSelected().schedule();
    right_encoder_inverted.getSelected().schedule();
    left_motors_inverted.getSelected().schedule();
    left_encoder_inverted.getSelected().schedule();

    m_robotContainer.getDriveSystem().getDefaultCommand();
  }
}
