package frc.robot.commands.mimicking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Mimicking;

public class UpdateRecordingName extends CommandBase {
    Mimicking mimicking;

    public UpdateRecordingName(Mimicking mimicking) {
        System.out.println("AAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHhh");

        this.mimicking = mimicking;
        mimicking.updateRecordingName();
    }

    /**
     * Ends the command
     */
    public boolean isFinished() {
        return true;
    }
}
