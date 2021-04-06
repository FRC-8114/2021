package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Wait;
import frc.robot.commands.indexSubsystem.IndexRun;
import frc.robot.commands.indexSubsystem.StopIndex;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class SequentialShooting extends ParallelCommandGroup {
    public SequentialShooting(ShooterSubsystem shooterSubsystem, IndexSubsystem indexSubsystem, Limelight limelightSubsystem) {
        double x = limelightSubsystem.approximateDistance();
        double y = 96;
        
        double velocity = shooterSubsystem.CalculateAutoVelocity(x, y, shooterSubsystem.GetHoodEncoderPosition());
        if (velocity > ShooterConstants.BALL_VELOCITY)
            velocity = ShooterConstants.BALL_VELOCITY;
        double desired_angle = shooterSubsystem.CalculateAutoAngle(x, y, velocity);
        
        addCommands(
            new SetHoodPosition(shooterSubsystem, desired_angle),

            new Wait(5000),

            new KickerRun(shooterSubsystem, 0.85),

            new IndexRun(indexSubsystem, 0.5),

            new Wait(5000),

            new StopIndex(indexSubsystem),

            new StopKicker(shooterSubsystem)
        );
    }
}
