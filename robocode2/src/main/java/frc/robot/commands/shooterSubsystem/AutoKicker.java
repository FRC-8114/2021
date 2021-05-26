package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoKicker extends CommandBase {
    private double desiredShooterRPM;

    public AutoKicker(double desiredShooterRPM) {
        this.desiredShooterRPM = desiredShooterRPM;
    }

    public void initialize() {
        new ShooterRun(1*ShooterConstants.MAX_INPUT, 15);
    }

    public void execute() {
        if (ShooterSubsystem.ShooterRPM >= desiredShooterRPM - 200) { KickerSubsystem.KickerRun(0.6); IndexSubsystem.TowerIndexRun(0.5);}
    }

    public void end() {

    }

    public boolean isFinished() {
        return true;
    }
}
