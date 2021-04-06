package frc.robot.commands.indexSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;

public class StopIndex extends CommandBase {
    IndexSubsystem indexSubsystem;

    public StopIndex(IndexSubsystem indexSubsystem) {
        this.indexSubsystem = indexSubsystem;

        addRequirements(indexSubsystem);
    }

    public void initialize() {
        indexSubsystem.AllIndexStop();
    }

    public boolean isFinished() {
        return true;
    }
}
