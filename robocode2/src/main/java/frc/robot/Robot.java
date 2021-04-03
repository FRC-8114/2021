// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.*;
import java.nio.file.*;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.math.Num;
import frc.robot.subsystems.Mimicking;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public Command m_autonomousCommand;

  public RobotContainer m_robotContainer;

  public final SendableChooser<String> recordingChooser = new SendableChooser<String>();
  public boolean isRecording, wasRecording;
  public File recording, driveSystemFile;
  public FileOutputStream driveSystemWriter;
  public String recordingName;
  public int recordingTicks;
  public double[] controllerInputs;

  public Scanner playbackScanner;
  public ArrayList<String> toFollow;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    recordingName = "default";
    Mimicking.updateRecordingName(this);
    Mimicking.setupRecordingChooser(this);
    Shuffleboard.getTab("Mimicking").add("Is recording", false).withWidget(BuiltInWidgets.kToggleButton).getEntry()
        .addListener(event -> {
          isRecording = event.value.getBoolean();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    Shuffleboard.getTab("Mimicking").add("Update The Recording Name", false).withWidget(BuiltInWidgets.kToggleButton)
        .getEntry().addListener(event -> {
          Mimicking.updateRecordingName(this);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    Shuffleboard.getTab("Mimicking").add("Recording Name", "default").withWidget(BuiltInWidgets.kTextView).getEntry()
        .addListener(event -> {
          recordingName = event.value.getString();
          System.out.println(event.value.getString());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    Shuffleboard.getTab("Mimicking").add(recordingChooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Send values to Shuffleboard
    m_robotContainer.getDriveSystem().sendOdometryToShuffleboard();
    m_robotContainer.getSearchSystem().sendEstimatedDistance();

    if (isRecording && driveSystemFile != null && recordingName != null) {
      try {
        String toWrite = "";
        toWrite += ((Math.abs(m_robotContainer.m_driverController.getY(GenericHID.Hand.kLeft)) > .02)
            ? m_robotContainer.m_driverController.getY(GenericHID.Hand.kLeft)
            : 0);
        toWrite += "," + ((Math.abs(m_robotContainer.m_driverController.getX(GenericHID.Hand.kRight)) > .02)
            ? m_robotContainer.m_driverController.getX(GenericHID.Hand.kRight)
            : 0);
        toWrite += "," + m_robotContainer.isQuickTurn;
        toWrite += "," + Timer.getFPGATimestamp() + "," + recordingTicks;
        toWrite += "," + m_robotContainer.getDriveSystem().getAverageEncoderDistance();
        toWrite += "," + m_robotContainer.getDriveSystem().getAverageEncoderVelocity();
        toWrite += "," + m_robotContainer.getDriveSystem().getHeading();
        toWrite += "," + m_robotContainer.getDriveSystem().getTurnRate() + "\n";
        System.out.print(toWrite);

        driveSystemWriter = new FileOutputStream(driveSystemFile, true);

        driveSystemWriter.write(toWrite.getBytes());
        driveSystemWriter.flush();
        driveSystemWriter.close();

        wasRecording = true;
        recordingTicks++;
      } catch (IOException e) {
        System.out.println("Error: " + e.getMessage());
      }
    } else if (wasRecording) {
      wasRecording = false;
      recordingTicks = 0;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    try {
      toFollow = new ArrayList<String>();
      playbackScanner = new Scanner(new File(recordingChooser.getSelected() +"/driveSystem"));

      while(playbackScanner.hasNextLine()) {
        toFollow.add(playbackScanner.nextLine());
      }
    } catch (FileNotFoundException e) {
      System.out.println("Error: "+ e.getMessage());
    }

    controllerInputs = new double[2];
    controllerInputs[0] = 0;
    controllerInputs[1] = 0;

    NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Recording_Speed").forceSetDouble(0);
    NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Recording_Quick_Turn").forceSetDouble(0);
    NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Recording_Arcade?").forceSetBoolean(false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    boolean quickTurn = false;

    if(toFollow.size() != 0) {
      try {
        String[] line = toFollow.remove(0).split(",");

        controllerInputs[0] = Double.parseDouble(line[0]);
        controllerInputs[1] = Double.parseDouble(line[1]);
        quickTurn = Boolean.parseBoolean(line[2]);

        NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Recording_Speed").forceSetDouble(controllerInputs[0]);
        NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Recording_Quick_Turn").forceSetDouble(controllerInputs[1]);
        NetworkTableInstance.getDefault().getTable("Mimicking").getEntry("Recording_Arcade?").forceSetBoolean(quickTurn);
        //System.out.println(controllerInputs[0] +" "+ controllerInputs[1] +" "+ quickTurn);
      } catch(NumberFormatException e) {
        System.out.println("Error: "+ e.getMessage());
      }
    }

    m_robotContainer.getDriveSystem().cheesyDrive(controllerInputs[0], controllerInputs[1], quickTurn);
    //m_robotContainer.getDriveSystem().tankDrive(controllerInputs[0], controllerInputs[0]);
  }

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
  public void teleopPeriodic() {
    m_robotContainer.periodic();
    m_robotContainer.getDriveSystem().cheesyDrive(
                    (m_robotContainer.m_driverController.getY(GenericHID.Hand.kLeft) > 0.02 || m_robotContainer.m_driverController.getY(GenericHID.Hand.kLeft) < -0.02)? m_robotContainer.m_driverController.getY(GenericHID.Hand.kLeft):0,
                    (m_robotContainer.m_driverController.getX(GenericHID.Hand.kRight) > 0.02 || m_robotContainer.m_driverController.getX(GenericHID.Hand.kRight) < -0.02)? m_robotContainer.m_driverController.getX(GenericHID.Hand.kRight):0,
                    m_robotContainer.isQuickTurn);
  }

  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}