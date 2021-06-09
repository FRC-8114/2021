package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Wait;
import frc.robot.commands.driveSubsystem.Backward;
import frc.robot.commands.driveSubsystem.Forward;
import frc.robot.commands.shooterSubsystem.AutoShooting;
import frc.robot.commands.shooterSubsystem.SetHoodPosition;
import frc.robot.commands.shooterSubsystem.ShooterRun;

public class ShootPushBack extends SequentialCommandGroup{
    public ShootPushBack(double d, double speedL, double speedR)
    {
        addCommands(
            new SetHoodPosition(44.2),
            
            new ShooterRun(3600),

            new AutoShooting(3600),

            new Forward(d, speedL, speedR),

            new Wait(1)
        );
    }
}
