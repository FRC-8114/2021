package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Wait;
import frc.robot.commands.driveSubsystem.Backward;
import frc.robot.commands.driveSubsystem.Forward;
import frc.robot.commands.driveSubsystem.ResetOdometry;
import frc.robot.commands.shooterSubsystem.AutoShooting;
import frc.robot.commands.shooterSubsystem.SetHoodPosition;
import frc.robot.commands.shooterSubsystem.ShooterRun;

public class ShootPushUp extends SequentialCommandGroup{
    public ShootPushUp(double d, double speed)
    {
        addCommands(
            new ResetOdometry(),

            new SetHoodPosition(44.2),
            
            new ShooterRun(3500),

            new AutoShooting(3500),

            new Backward(d, speed),

            new Wait(1)
        );
    }
}
