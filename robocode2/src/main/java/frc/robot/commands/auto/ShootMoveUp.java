package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Wait;
import frc.robot.commands.driveSubsystem.Forward;
import frc.robot.commands.shooterSubsystem.ShooterRun;

public class ShootMoveUp extends SequentialCommandGroup{
    public ShootMoveUp(double d, double speedL, double speedR)
    {
        addCommands(
            new ShooterRun(.6, 15),

            new Forward(d, speedL, speedR),

            new Wait(1)
        );
    }
}
