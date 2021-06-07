package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class TurnOnLED extends CommandBase {
    private final Limelight limelight;

    public TurnOnLED(Limelight limelight) {
        this.limelight = limelight;

        addRequirements(limelight);
    }

    public void execute() {
        limelight.turnOnLED();
    }

    public boolean isFinished() {
        return true;
    }
}
