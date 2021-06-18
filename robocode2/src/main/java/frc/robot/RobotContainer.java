// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
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
import frc.robot.commands.limelight.TurnOffLED;
import frc.robot.commands.limelight.TurnOnLED;
import frc.robot.commands.searchSystem.GetAverageDistance;
import frc.robot.commands.shooterSubsystem.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.ShootMoveBack;
import frc.robot.commands.auto.ShootMoveUp;
import frc.robot.commands.auto.ShootPushUp;
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
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public final SearchSystem searchSystem = new SearchSystem();
  public final IndexSubsystem indexSubsystem = new IndexSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final KickerSubsystem kickerSubsystem = new KickerSubsystem();
  public final Limelight limelightSubsystem = new Limelight("limelight-eleven");

  public final NetworkTableEntry wallShotAngle, lineShotAngle, intakeRun, intakeReverse, shooterRPM, indexRun, indexReverse, runAllIndex, reverseAllIndex, runIndexFront, reverseIndexFront, runBallTube, reverseBallTube, climberUp, climberDown, autoAimSpeed, autoAimOffsetAngle, autoAimMOE, autoShootIndexSpeed, autoAimToggle, hoodMOE, hoodSpeed;

  public Trajectory exampleTrajectory;
  public boolean isQuickTurn = false, climberExtending = false, climberRetracting = false, intaking = false, reverseIntaking = false, autoShooting = false, driverIntaking = false, operatorIntaking = false;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  XboxController m_operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    wallShotAngle = Shuffleboard.getTab("Robot Control").add("Hood/Wall Shot Angle", 14.0).getEntry();
    lineShotAngle = Shuffleboard.getTab("Robot Control").add("Hood/Initiation Shot Angle", 44.2).getEntry();
    intakeRun = Shuffleboard.getTab("Robot Control").add("Intake/Run Intake Speed", .35).getEntry();
    intakeReverse = Shuffleboard.getTab("Robot Control").add("Intake/Reverse Intake Speed", 0.3).getEntry();
    shooterRPM = Shuffleboard.getTab("Robot Control").add("Shooter/Shooter RPM", 3500).getEntry();
    indexRun = Shuffleboard.getTab("Robot Control").add("Index/Run Index Speed", 0.6).getEntry();
    indexReverse = Shuffleboard.getTab("Robot Control").add("Index/Reverse Index Speed", 0.6).getEntry();
    runAllIndex = Shuffleboard.getTab("Robot Control").add("Index/Run All Index", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    reverseAllIndex = Shuffleboard.getTab("Robot Control").add("Index/Reverse All Index", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    climberUp = Shuffleboard.getTab("Robot Control").add("Climber/Climber Up", 1).getEntry();
    climberDown = Shuffleboard.getTab("Robot Control").add("Climber/Climber Down", 0.5).getEntry();
    hoodMOE = Shuffleboard.getTab("Robot Control").add("Hood/Hood Angle MOE", 0.5).getEntry();
    hoodSpeed = Shuffleboard.getTab("Robot Control").add("Hood/Hood Speed", 0.1).getEntry();

    autoAimSpeed = Shuffleboard.getTab("Robot Control").add("Shooter/Auto Aim Speed", 0.15).getEntry();
    autoAimOffsetAngle = Shuffleboard.getTab("Robot Control").add("Shooter/Auto Aim Offset Angle", LimelightConstants.SHOOTER_LIMELIGHT_OFFSET_ANGLE).getEntry();
    autoAimMOE = Shuffleboard.getTab("Robot Control").add("Shooter/Auto Aim MOE", LimelightConstants.AUTO_CENTER_TOLERANCE).getEntry();
    autoShootIndexSpeed = Shuffleboard.getTab("Robot Control").add("Shooter/Auto Shoot Index Speed", 0.85).getEntry();

    // Shuffleboard independent index functions
    runBallTube = Shuffleboard.getTab("Robot Control").add("Index/Run Ball Tube", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    runBallTube.addListener(event -> {
        if(event.value.getBoolean()) {
            indexSubsystem.TowerIndexRun(indexRun.getDouble(0.6));
        } else {
            indexSubsystem.TowerIndexStop();;
        }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    reverseBallTube = Shuffleboard.getTab("Robot Control").add("Index/Reverse Ball Tube", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    reverseBallTube.addListener(event -> {
        if(event.value.getBoolean()) {
            indexSubsystem.TowerIndexReverse(indexReverse.getDouble(0.6));
        } else {
            indexSubsystem.TowerIndexStop();;
        }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    runIndexFront = Shuffleboard.getTab("Robot Control").add("Index/Run Front Index", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    runIndexFront.addListener(event -> {
        if(event.value.getBoolean()) {
            indexSubsystem.FrontIndexReverse(indexRun.getDouble(0.6));
        } else {
            indexSubsystem.FrontIndexStop();
        }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    reverseIndexFront = Shuffleboard.getTab("Robot Control").add("Index/Reverse Front Index", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    reverseIndexFront.addListener(event -> {
        if(event.value.getBoolean()) {
            indexSubsystem.FrontIndexRun(indexReverse.getDouble(0.6));
        } else {
            indexSubsystem.FrontIndexStop();;
        }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Toggle for autoaim
    // Shuffleboard Auto Center Emergency Stop
    autoAimToggle = Shuffleboard.getTab("Robot Control").add("Auto Center Toggle", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    autoAimToggle.addListener(event -> {
        DriveSubsystem.canAutoCenter = !event.value.getBoolean();
        DriveSubsystem.driverControl = true;
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

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
  public void configureButtonBindings() {
    // Increment drive speed when the right bumper is pressed
    // new JoystickButton(m_driverController, Button.kBumperRight.value)
    //     .whenPressed(() -> m_robotDrive.incMaxSpeed());

    // // Decrement drive speed when the left bumper is pressed
    // new JoystickButton(m_driverController, Button.kBumperLeft.value)
    //     .whenPressed(() -> m_robotDrive.decMaxSpeed());

    // Y Button
    // Climbs
    new JoystickButton(m_driverController, Button.kY.value)
        .whileHeld(() -> climberSubsystem.ClimberUp(0.5))
        .whenReleased(() -> climberSubsystem.ClimberStop());

    // B Button
    // Far shot hood angle
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(new Forward(10, .45));

    // X Button
    // Wall shot hood angle
    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new SetHoodPosition(wallShotAngle.getDouble(14.0), hoodSpeed.getDouble(0.1), hoodMOE.getDouble(0.5)))
        .whenPressed(() -> shooterRPM.forceSetDouble(2800))
        .whenPressed(() -> autoAimToggle.forceSetBoolean(false));
    Shuffleboard.getTab("Robot Control").add("Hood/Set Wall Shot Angle", new SetHoodPosition(wallShotAngle.getDouble(14.0), hoodSpeed.getDouble(0.1), hoodMOE.getDouble(0.5)));
        
    // A Button
    // Initiation line shot hood angle
    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new SetHoodPosition(lineShotAngle.getDouble(44.2), hoodSpeed.getDouble(0.1), hoodMOE.getDouble(0.5)))
        .whenPressed(() -> shooterRPM.forceSetDouble(3300))
        .whenPressed(() -> autoAimToggle.forceSetBoolean(true));
    Shuffleboard.getTab("Robot Control").add("Hood/Set Initiation Shot Angle", new SetHoodPosition(lineShotAngle.getDouble(44.2), hoodSpeed.getDouble(0.1), hoodMOE.getDouble(0.5)));

    new JoystickButton(m_driverController, Button.kStart.value)
        .whenPressed(new SetHoodPosition(10, hoodSpeed.getDouble(0.1), hoodMOE.getDouble(0.5)));
    
    // Right Bumper
    // Runs intake forwards
    new JoystickButton(m_driverController, 6)
        .whenPressed(() -> driverIntaking = !driverIntaking);

    Shuffleboard.getTab("Robot Control").add("Intake/Run Intake", false).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        operatorIntaking = event.value.getBoolean();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Left Bumper
    // Runs intake backwards
    new JoystickButton(m_driverController, 5)
    .whenPressed(() -> reverseIntaking = true)
    .whenReleased(() -> reverseIntaking = false);
    Shuffleboard.getTab("Robot Control").add("Intake/Reverse Intake", false).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        reverseIntaking = event.value.getBoolean();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Right Joystick Button
    // Toggles arcade/curvature drive
    new JoystickButton(m_driverController, Button.kStickRight.value)
      .whenPressed(() -> isQuickTurn = !isQuickTurn);  
    Shuffleboard.getTab("Robot Control").add("Drive/Arcade-Curvature", false).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        isQuickTurn = event.value.getBoolean();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    
    // Shuffleboard control for F/B toggle
    Shuffleboard.getTab("Robot Control").add("Drive/Reverse Drive", false).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        m_robotDrive.back = event.value.getBoolean();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Shuffleboard Control for Index Run
    runAllIndex.addListener(event -> {
        if(event.value.getBoolean()) {
            indexSubsystem.AllIndexRun(indexRun.getDouble(0.6));
        } else {
            indexSubsystem.AllIndexStop();
        }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Shuffleboard Control for Index Reverse
    reverseAllIndex.addListener(event -> {
        if(event.value.getBoolean()) {
            indexSubsystem.AllIndexReverse(indexReverse.getDouble(0.6));
        } else {
            indexSubsystem.AllIndexStop();
        }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Shuffleboard Controls for Climber
    Shuffleboard.getTab("Robot Control").add("Climber/Extend Climber", false).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        climberExtending = event.value.getBoolean();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    Shuffleboard.getTab("Robot Control").add("Climber/Retract Climber", false).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        climberRetracting = event.value.getBoolean();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Shuffleboard Limelight Toggle
    Shuffleboard.getTab("Robot Control").add("Limelight Toggle", false).withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
        if(event.value.getBoolean()) {
            new TurnOnLED(limelightSubsystem).schedule();
        } else {
            new TurnOffLED(limelightSubsystem).schedule();
        }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Shuffleboard Auto Aim
    Shuffleboard.getTab("Robot Control").add("Shooter/Auto Aim", new AutoCenter(autoAimSpeed.getDouble(0.15), autoAimOffsetAngle.getDouble(LimelightConstants.SHOOTER_LIMELIGHT_OFFSET_ANGLE), autoAimMOE.getDouble(LimelightConstants.AUTO_CENTER_TOLERANCE)));

    // Shuffleboard to run shooter
    Shuffleboard.getTab("Robot Control").add("Shooter/Run Shooter", false).getEntry().addListener(event -> {
        if(event.value.getBoolean()) {
            ShooterSubsystem.ShooterRun(shooterRPM.getDouble(3500));
        } else {
            new StopShooter().schedule();
        }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    Shuffleboard.getTab("Robot Control").add("Shooter/Auto Shoot", false).getEntry().addListener(event -> {
        autoShooting = event.value.getBoolean();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Adds the GetAveragedistance command to SmartDashboard
    SmartDashboard.putData(new GetAverageDistance(searchSystem, 3));
  }

  public void periodic() {
    shooterSubsystem.periodic();
    limelightSubsystem.periodic();
    // Runs the climber
    if(climberExtending) {
        climberSubsystem.ClimberUp(climberUp.getDouble(0.5));
    } else if(climberRetracting) {
        climberSubsystem.ClimberDown(climberDown.getDouble(0.5));
    } else {
        climberSubsystem.ClimberStop();
    }

    // Intake running logic
    if(driverIntaking || operatorIntaking) {
        intakeSubsystem.IntakeRun(intakeRun.getDouble(0.35));
        intaking = true;
    } else if(reverseIntaking) {
        intakeSubsystem.IntakeReverse(intakeReverse.getDouble(0.25));
        intaking = false;
    } else if(!driverIntaking && !operatorIntaking) {
        intakeSubsystem.IntakeStop();
        indexSubsystem.FrontIndexStop();
        intaking = false;
    } 

    boolean runningFrontIndex = runIndexFront.getBoolean(false) || reverseIndexFront.getBoolean(false);
    boolean runningBallTube = runBallTube.getBoolean(false) || reverseBallTube.getBoolean(false);
    boolean runningAllIndex = runAllIndex.getBoolean(false) || reverseAllIndex.getBoolean(false) || runningFrontIndex || runningBallTube;
    // Left Trigger
    // Runs the index
    if(m_driverController.getTriggerAxis(Hand.kLeft) == 1) {
        indexSubsystem.AllIndexRun(indexRun.getDouble(0.6));
    } else if((m_driverController.getTriggerAxis(Hand.kLeft) != 1) && !runningAllIndex) {
        indexSubsystem.AllIndexStop();
    } else if(!runningAllIndex && !runningFrontIndex) {
        indexSubsystem.FrontIndexStop();
    } else if((!runningAllIndex && !runningBallTube) || (!runningBallTube && intaking)) {
        indexSubsystem.TowerIndexStop();
    } 

    // Right Trigger
    // Runs the auto-shoot (and auto center) routines while held
    if((m_driverController.getTriggerAxis(Hand.kRight) == 1 || autoShooting) && (SmartDashboard.getNumber("Flywheel SetPoint", 0) == 0)) {
        new TeleopShooting(m_driverController, shooterRPM.getDouble(3600), autoAimSpeed.getDouble(0.15), autoAimOffsetAngle.getDouble(LimelightConstants.SHOOTER_LIMELIGHT_OFFSET_ANGLE), autoAimMOE.getDouble(LimelightConstants.AUTO_CENTER_TOLERANCE), autoShootIndexSpeed.getDouble(0.85)).schedule();
    } else if (m_driverController.getTriggerAxis(Hand.kRight) != 1 && !autoShooting) {
        ShooterSubsystem.ShooterStop();
    }

    // D-pad Controls
    // Down reverses the drivetrain
    if(m_driverController.getPOV() == 180) {
        m_robotDrive.back = !m_robotDrive.back;
        ShooterSubsystem.ShooterRun(2800);
    }

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
      return new ShootMoveUp(60,.5);
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
