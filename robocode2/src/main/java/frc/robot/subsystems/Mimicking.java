package frc.robot.subsystems;

import java.io.*;
import java.util.Scanner;
import java.util.Stack;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.mimicking.UpdateRecordingName;

public class Mimicking extends SubsystemBase {
    /*
     * Variables for recording
     */
    // Used to reset ticks and timer
    public boolean isRecording, wasRecording;

    // Unused by playback, but useful for analyzing recording data
    public int recordingTicks;

    // The name of the current recording
    public String recordingName;

    // The directory and files to which data is stored, and the outputStreams through which the recording is done
    public File recording, driveSystemFile;
    public FileOutputStream driveSystemWriter;

    /*
     * Variables for playback
     */

    // Sendable chooser for selecting the recording to play back
    public final SendableChooser<String> recordingChooser = new SendableChooser<String>();
    
    // Debugging NetworkTableEntries
    public final NetworkTableEntry velocityEntry, curvatureEntry, isArcadeEntry;

    // Stings for storing the csv files 
    public Scanner playbackScanner;
    public Stack<String> toFollow;

    // Variables for storing cheesyDrive values
    public boolean isArcade;
    public double velocity, curvature;

    /*
     * Network Table Entries for Shuffleboard values
     */
    public NetworkTableEntry isRecordingEntry, recordingNameEntry;

    /**
     * Runs the initial mimicking setup a process that includes: 
     */
    public Mimicking() {
        // Initializes networktable entries for playback debugging
        velocityEntry = Shuffleboard.getTab("Mimicking").add("recording_velocity", 0).getEntry();
        curvatureEntry = Shuffleboard.getTab("Mimicking").add("recording_curvature", 0).getEntry();
        isArcadeEntry = Shuffleboard.getTab("Mimicking").add("recording_isArcade", false).getEntry();

        // Setting default values for various recording variables  
        recordingName = "default";
        updateRecordingName();

        // Boolean to turn recording functionality on/off
        isRecordingEntry = Shuffleboard.getTab("Mimicking").add("is_recording?", false)
                                                           .withWidget(BuiltInWidgets.kToggleButton)
                                                           .getEntry();
        isRecordingEntry.addListener(event -> {
            isRecording = event.value.getBoolean();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // String input for recording name, must run "UpdateRecordingName" to apply
        recordingNameEntry = Shuffleboard.getTab("Mimicking").add("recording_name", "default")
                                                           .withWidget(BuiltInWidgets.kTextView)
                                                           .getEntry();
        recordingNameEntry.addListener(event -> {
            recordingName = event.value.getString();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // Runs the initial setup for the recording chooser and adds it to Shuffleboard
        setupRecordingChooser();
        Shuffleboard.getTab("Mimicking").add(recordingChooser);
    }

    /**
     * Updates the paths of all recording files to 
     */
    public void updateRecordingName() {
        try {
            // Updates the recording directory
            recording = new File("/home/lvuser/recordings/"+ recordingName);
            if(recording.exists()) {
            recording.delete();
            }
            recording.mkdir();
          
            // Updates the driveSystem file
            driveSystemFile = new File("/home/lvuser/recordings/"+ recordingName +"/driveSystem");
            //System.out.println(driveSystemFile.toPath());
            driveSystemWriter = new FileOutputStream(driveSystemFile);
            if(driveSystemFile.exists()) {
                driveSystemFile.delete();
            }
            driveSystemFile.createNewFile();

            // Adds the recording to the chooser so it can be played back without code restart
            addRecordingToChooser(recording);
        } catch(IOException e) {
            System.out.println("Error: "+ e.getMessage());
        }
    }

    /**
     * Initializes the recording chooser with all existing paths
     */
    public void setupRecordingChooser() {
        // Adds the default as the default option
        recordingChooser.setDefaultOption("default", "/home/lvuser/recordings/default");

        // Loops through all files/directories in the recordings directory
        File recordings = new File("/home/lvuser/recordings/");
        for (File recording : recordings.listFiles()) {
            if (recording.isDirectory()) {
                addRecordingToChooser(recording);
            }
        }
    }

    /**
     * Adds a recording to the recording chooser, where the recordings's name is the option
     * key and the recordings path is the associated value
     * 
     * @param recording the file to add
     */
    public void addRecordingToChooser(File recording) {
        recordingChooser.addOption(recording.getName(), recording.getAbsolutePath());
    }

    /**
     * Writes data to the recording files
     */
    public void record(RobotContainer robotContainer) {
        if (isRecording) {
            if(!wasRecording) {
                updateRecordingName();
            }
            try {
              String toWrite = "";

              // Timestamps for data analysis
              toWrite += recordingTicks +","+ Timer.getFPGATimestamp();

              // Inputs, actually played back to recreate motion
              toWrite += ","+ ((Math.abs(robotContainer.m_driverController.getY(GenericHID.Hand.kLeft)) > .02)?
                robotContainer.m_driverController.getY(GenericHID.Hand.kLeft):0);
              toWrite += ","+ ((Math.abs(robotContainer.m_driverController.getX(GenericHID.Hand.kRight)) > .02)? 
                robotContainer.m_driverController.getX(GenericHID.Hand.kRight):0);  
              toWrite += ","+ robotContainer.isQuickTurn;

              // Used for analyzing the correctness of the path
              toWrite += ","+ robotContainer.getDriveSystem().getAverageEncoderDistance();
              toWrite += ","+ robotContainer.getDriveSystem().getAverageEncoderVelocity();
              toWrite += ","+ robotContainer.getDriveSystem().getHeading();
              toWrite += ","+ robotContainer.getDriveSystem().getTurnRate() +"\n";
              
              // For debugging
              System.out.print(toWrite);
      
              // Sets up the output stream for writing the string above
              driveSystemWriter = new FileOutputStream(driveSystemFile, true);
      
              // Writes the data
              driveSystemWriter.write(toWrite.getBytes());
              driveSystemWriter.flush();
              driveSystemWriter.close();

              // Increments ticks
              recordingTicks++;
            } catch (IOException e) {
              System.out.println("Error: " + e.getMessage());
            }
        }
    }

    /**
     * Converts the recording csv files into a stack (in chronological order top to bottom)
     */
    public void readInRecording() {
        try {
            // Resets the stack in case it wasn't already empty
            toFollow = new Stack<String>();
            // Sets up a scanner for the driveSystem file
            playbackScanner = new Scanner(new File(recordingChooser.getSelected() +"/driveSystem"));
        
            // Transfers the contents of the csv to the stack
            while(playbackScanner.hasNextLine()) {
                toFollow.add(0, playbackScanner.nextLine());
            }
        } catch (FileNotFoundException e) {
            System.out.println("Error: "+ e.getMessage());
        }
    }

    /**
     * Updates the various subsystems with the values from the recording being played back
     * if there is another line of data. Returns false if there isn't.
     * 
     * @param robotContainer the robot container to acess the necessary subsystems
     * @return whether or not there is another line to execute
     */
    public boolean playbackRecording(RobotContainer robotContainer) {
        if(toFollow.size() != 0) {
            try {
                parseData();

                robotContainer.getDriveSystem().cheesyDrive(velocity, curvature, isArcade);
            } catch(NumberFormatException e) {
                System.out.println("Error: "+ e.getMessage());
            }
            return true;
          }
        return false;
    }

    /**
     * Converts the next line of the recording into the appropriate datatypes
     */
    public void parseData() {
        String[] line = toFollow.pop().split(",");
        
        // Velocity, Curvature, and isArcade are the 3rd, 4th, and 5th values in their rows
        velocity = Double.parseDouble(line[2]);
        curvature = Double.parseDouble(line[3]) * -1;
        isArcade = Boolean.parseBoolean(line[4]);
    }
}
