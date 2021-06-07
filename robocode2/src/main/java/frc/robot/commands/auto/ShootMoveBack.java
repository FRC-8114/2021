package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Wait;
import frc.robot.commands.driveSubsystem.Backward;
import frc.robot.commands.shooterSubsystem.AutoShooting;

public class ShootMoveBack extends SequentialCommandGroup{
    public ShootMoveBack(double d, double speedL, double speedR)
    {
        addCommands(
            new AutoShooting(),

            new Backward(d, speedL, speedR),

            new Wait(1)
        );
    }
}
