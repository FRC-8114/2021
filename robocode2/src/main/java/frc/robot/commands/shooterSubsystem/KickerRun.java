package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class KickerRun extends CommandBase {
    double speed;
    ShooterSubsystem shooterSubsystem;
    
    public KickerRun(ShooterSubsystem shooterSubsystem, double speed) {
        this.speed = speed;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    public void initialize() {
        shooterSubsystem.KickerRun(speed);
    }

    public void execute() {
        shooterSubsystem.KickerRun(speed);
    }

    public boolean isFinished() {
        return true;
    }
}
