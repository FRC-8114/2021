package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends CommandBase {
    ShooterSubsystem shooterSubsystem;

    public StopShooter(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    public void initialize() {
        
    }
    
    public void execute() {
        shooterSubsystem.ShooterStop();
    }

    public boolean isFinished() {
        return true;
    }
}
