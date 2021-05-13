package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class KickerRun extends CommandBase {
    double speed;
    KickerSubsystem kickerSubsystem;
    
    public KickerRun(KickerSubsystem kickerSubsystem, double speed) {
        this.speed = speed;
        this.kickerSubsystem = kickerSubsystem;

        addRequirements(kickerSubsystem);
    }

    public void initialize() {
        kickerSubsystem.KickerRun(speed);
    }

    public void execute() {
        kickerSubsystem.KickerRun(speed);
    }

    public boolean isFinished() {
        return true;
    }
}
