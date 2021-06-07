package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class TurnOffLED extends CommandBase {
    private final Limelight limelight;

    public TurnOffLED(Limelight limelight) {
        this.limelight = limelight;

        addRequirements(limelight);
    }

    public void execute() {
        limelight.turnOffLED();
    }

    public boolean isFinished() {
        return true;
    }
}
