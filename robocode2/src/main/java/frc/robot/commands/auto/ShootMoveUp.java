package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Wait;
import frc.robot.commands.driveSubsystem.Backward;
import frc.robot.commands.driveSubsystem.Forward;
import frc.robot.commands.shooterSubsystem.AutoShooting;
import frc.robot.commands.shooterSubsystem.SetHoodPosition;
import frc.robot.commands.shooterSubsystem.ShooterRun;

public class ShootMoveUp extends SequentialCommandGroup{
    public ShootMoveUp(double d, double speedL, double speedR)
    {
        addCommands(
            new SetHoodPosition(44.2),
            
            new ShooterRun(3000),

            new AutoShooting(3000),

            new Backward(d, speedL, speedR),

            new Wait(1)
        );
    }
}
