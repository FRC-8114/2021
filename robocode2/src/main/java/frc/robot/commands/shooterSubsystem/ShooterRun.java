package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShooterRun extends CommandBase {
    private double time;
    private Timer a;
    double speed;
    
    public ShooterRun (double speed, double time) {
        this.speed = speed;
        this.time = time;
    }

    public void initialize() {
        ShooterSubsystem.ShooterRun(speed);
        a = new Timer();
        a.start();
        
    }
    
    public void execute() {
        ShooterSubsystem.ShooterRun(speed);
    }

    public void end() {
        ShooterSubsystem.ShooterStop();
        a.reset();
    }

    public boolean isFinished() {
        if(a.get() > time) {
            ShooterSubsystem.ShooterStop();
            return true;
        }
        return false;
    }
}
