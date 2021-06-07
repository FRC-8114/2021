package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoShooting extends CommandBase {
    int shots_fired = 0;
    boolean shotPreviously = false;

    public AutoShooting() {
    }

    public void initialize() {
        new ShooterRun(3500);
    }

    public void execute() {
        double shooterRPM = SmartDashboard.getNumber("Flywheel Set Position", 0);
        if (shooterRPM >= 3500)
        {
            IndexSubsystem.TowerIndexRun(0.75);
            KickerSubsystem.KickerRun(0.8);
            shotPreviously = true;
        }

        if (shooterRPM < 3400 && shotPreviously) {
            shots_fired++;
            IndexSubsystem.TowerIndexStop();
            KickerSubsystem.KickerStop();
        }
    }

    public void end() {
        new StopShooter();
    }

    public boolean isFinished() {
        if (shots_fired >= 3)
            return true;
        return false;
    }
    
}
