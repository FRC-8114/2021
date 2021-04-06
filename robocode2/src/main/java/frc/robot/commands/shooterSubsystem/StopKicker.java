package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopKicker extends CommandBase {
    ShooterSubsystem shooterSubsystem;

    public StopKicker (ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    public void initialize() {
        
    }

    public void execute() {
        shooterSubsystem.KickerStop();
    }

    public boolean isFinished() {
        return true;
    }
}
