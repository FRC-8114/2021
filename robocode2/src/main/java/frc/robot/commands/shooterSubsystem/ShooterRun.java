package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRun extends CommandBase {
    
    ShooterSubsystem shooterSubsystem;
    double speed;
    int ticks;
    
    public ShooterRun (ShooterSubsystem shooterSubsystem, double speed) {
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;

        addRequirements(shooterSubsystem);
    }

    public void initialize() {
        shooterSubsystem.ShooterRun(speed);
        ticks = 0;
    }
    
    public void execute() {
        shooterSubsystem.ShooterRun(speed);
        ticks++;
    }

    public void end() {
        shooterSubsystem.leftShooterController.set(speed);
    }

    public boolean isFinished() {
        if(ticks > 200) {
            return true;
        }
        return false;
    }
}
