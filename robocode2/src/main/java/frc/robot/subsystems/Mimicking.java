package frc.robot.subsystems;

import java.io.*;

import frc.robot.Robot;

public class Mimicking {
    public static void updateRecordingName(Robot robot) {
        try {
            robot.recording = new File("/home/lvuser/recordings/"+ robot.recordingName);
            if(robot.recording.exists()) {
                robot.recording.delete();
            }
            robot.recording.mkdir();
          
            robot.driveSystemFile = new File("/home/lvuser/recordings/"+ robot.recordingName +"/driveSystem");
            System.out.println(robot.driveSystemFile.toPath());
            robot.driveSystemWriter = new FileOutputStream(robot.driveSystemFile);
            if(robot.driveSystemFile.exists()) {
                robot.driveSystemFile.delete();
            }
            robot.driveSystemFile.createNewFile();
          } catch(IOException e) {
            System.out.println("Error: "+ e.getMessage());
          }
    }

    public static void setupRecordingChooser(Robot robot) {
        robot.recordingChooser.setDefaultOption("default", "/home/lvuser/recordings/default");

        File recordings = new File("/home/lvuser/recordings/");
        for (File recording : recordings.listFiles()) {
            if (recording.isDirectory()) {
                robot.recordingChooser.addOption(recording.getName(), recording.getAbsolutePath());
            }
        }
    }
}
