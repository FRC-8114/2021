package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Wait;
import frc.robot.commands.driveSubsystem.Backward;
import frc.robot.commands.shooterSubsystem.ShooterRun;

public class ShootMoveBack extends SequentialCommandGroup{
    public ShootMoveBack(double d, double speedL, double speedR)
    {
        addCommands(
            new ShooterRun(.6, 15),

            new Backward(d, speedL, speedR),

            new Wait(1)
        );
    }
}
