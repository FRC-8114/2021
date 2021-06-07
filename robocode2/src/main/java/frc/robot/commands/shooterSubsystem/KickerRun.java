package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KickerSubsystem;

public class KickerRun extends CommandBase {
    double speed;
    
    public KickerRun(double speed) {
        this.speed = speed;
    }

    public void initialize() {
        KickerSubsystem.KickerRun(speed);
    }

    public void execute() {
        KickerSubsystem.KickerRun(speed);
    }

    public boolean isFinished() {
        return true;
    }
}
