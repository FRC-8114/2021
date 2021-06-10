package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoShooting extends CommandBase {
    int shots_fired = 0;
    double rpm;
    Timer timer = new Timer();
    boolean shotPreviously = false;

    public AutoShooting(double rpm) {
        this.rpm = rpm;
    }

    public void initialize() {
        timer.start();
        //new ShooterRun(3600).schedule();
    }

    public void execute() {
        double shooterRPM = SmartDashboard.getNumber("Flywheel Process Variable", 0);
        if (shooterRPM >= rpm-45 && shooterRPM <= rpm)
        {
            IndexSubsystem.TowerIndexRun(0.6);
            KickerSubsystem.KickerRun(0.8);
            shotPreviously = true;
        }

        if (shooterRPM < rpm-60 && shotPreviously) {
            shots_fired++;
            IndexSubsystem.TowerIndexStop();
            KickerSubsystem.KickerStop();
            shotPreviously = false;
        }
    }

    public void end(boolean interrupted) {
        new StopShooter().schedule();
        IndexSubsystem.TowerIndexStop();
        KickerSubsystem.KickerStop();
        timer.stop();
        timer.reset();
    }

    public boolean isFinished() {
        if (shots_fired >= 3 || timer.get() > 12)
            return true;
        return false;
    }
    
}
