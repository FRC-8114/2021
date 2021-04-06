package frc.robot.commands.indexSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;

public class IndexRun extends CommandBase {
    double speed;
    IndexSubsystem indexSubsystem;

    public IndexRun(IndexSubsystem indexSubsystem, double speed) {
        this.indexSubsystem = indexSubsystem;
        this.speed = speed;

        addRequirements(indexSubsystem);
    }

    public void initialize() {
        indexSubsystem.AllIndexRun(speed);
    }

    public void execute() {
        indexSubsystem.AllIndexRun(speed);
    }

    public boolean isFinished() {
        return true;
    }
}
