package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveSubsystem.Backward;
import frc.robot.commands.shooterSubsystem.ShooterRun;

public class ShootPushBack extends SequentialCommandGroup{
    public ShootPushBack(double d, double speedL, double speedR)
    {
        addCommands(
            new ShooterRun(.6),

            new Backward(d, speedL, speedR)
        );
    }
}