package frc.robot.commands.mimicking;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Mimicking;

public class Playback extends CommandBase{
    public RobotContainer robotContainer;
    public Mimicking mimicking;
    public boolean moreRecording;
    
    /**
     * Stores the Mimicking and RobotContainer to use for recording
     * 
     * @param mimicking the Mimicking object used for recording
     * @param robotContainer the RobotContainer object used for recording
     */
    public Playback(Mimicking mimicking, RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.mimicking = mimicking;

        m_requirements.add(mimicking);
    }

    public void initialize() {
        // Ensures the command doesn't immediately stop
        moreRecording = true;

        // Reads the recording to play back
        mimicking.readInRecording();

        // Ensures all playback variables start at 0, prevents movement due to canceled playback
        mimicking.velocity = 0;
        mimicking.curvature = 0;
        mimicking.isArcade = false;

        // Sets all playback debugging value displays to default
        
    }

    public void periodic() {
        mimicking.playbackRecording(robotContainer);

        // Updates all playback debugging value displays
        mimicking.velocityEntry.forceSetDouble(mimicking.velocity);
        mimicking.curvatureEntry.forceSetDouble(mimicking.curvature);
        mimicking.isArcadeEntry.forceSetBoolean(mimicking.isArcade);
    }
    
    /**
     * Ends if the recording has been fully played back
     * 
     * @return if the command is done running
     */
    public boolean isFinished() {
        return !moreRecording;
    }
}