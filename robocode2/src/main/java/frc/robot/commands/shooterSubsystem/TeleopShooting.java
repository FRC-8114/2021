package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class TeleopShooting extends CommandBase {
    public TeleopShooting() {

    }

    public void initialize() {
        new ShooterRun(3500);
    }

    public void execute() {
        double shooterRPM = SmartDashboard.getNumber("Flywheel Set Position", 0);
        if (shooterRPM >= 3500) {
            IndexSubsystem.TowerIndexRun(0.75);
            KickerSubsystem.KickerRun(0.8);
        }

        if (shooterRPM < 3450) {
            IndexSubsystem.TowerIndexStop();
            KickerSubsystem.KickerStop();
        }
    }

    public void end() {
    }

    public boolean isFinished() {
        return true;
    }
}