package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveSubsystem.Forward;
import frc.robot.commands.shooterSubsystem.ShooterRun;

public class ShootPushUp extends SequentialCommandGroup{
    public ShootPushUp(double d, double speedL, double speedR)
    {
        addCommands(
            new ShooterRun(.6),

            new Forward(d, speedL, speedR)
        );
    }
}
