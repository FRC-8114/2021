package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopKicker extends CommandBase {
    KickerSubsystem kickerSubsystem;

    public StopKicker (KickerSubsystem kickerSubsystem) {
        this.kickerSubsystem = kickerSubsystem;

        addRequirements(kickerSubsystem);
    }

    public void initialize() {

    }

    public void execute() {
        kickerSubsystem.KickerStop();
    }

    public boolean isFinished() {
        return true;
    }
}
