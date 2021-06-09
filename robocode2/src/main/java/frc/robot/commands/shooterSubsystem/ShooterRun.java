package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShooterRun extends CommandBase {
    double speed;
    
    public ShooterRun (double speed) {
        this.speed = speed;
    }

    public void initialize() {
        ShooterSubsystem.ShooterRun(speed);      
    }
    
    public void execute() {
    }

    public void end() {
        ShooterSubsystem.ShooterStop();
    }

    public boolean isFinished() {
        return true;
    }
}
