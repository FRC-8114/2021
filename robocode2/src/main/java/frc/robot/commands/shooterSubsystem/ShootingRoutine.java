package frc.robot.commands.shooterSubsystem;

import javax.swing.GroupLayout.ParallelGroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.*;
import frc.robot.commands.indexSubsystem.IndexRun;
import frc.robot.commands.indexSubsystem.StopIndex;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingRoutine extends ParallelDeadlineGroup {
    public ShootingRoutine(KickerSubsystem kickerSubsystem, ShooterSubsystem shooterSubsystem, Limelight limelightSubsystem, IndexSubsystem indexSubsystem) {
        super(new SequentialShooting(kickerSubsystem, shooterSubsystem, indexSubsystem, limelightSubsystem), new ShooterRun(shooterSubsystem, shooterSubsystem.CalculateAutoVelocity(limelightSubsystem.approximateDistance(), 96, shooterSubsystem.GetHoodEncoderPosition())));
        double x = limelightSubsystem.approximateDistance();
        double y = 96;
        
        double velocity = shooterSubsystem.CalculateAutoVelocity(x, y, shooterSubsystem.GetHoodEncoderPosition());
        if (velocity > ShooterConstants.BALL_VELOCITY)
            velocity = ShooterConstants.BALL_VELOCITY;


        addCommands(
            new ShooterRun(velocity),

            new SequentialShooting(kickerSubsystem, shooterSubsystem, indexSubsystem, limelightSubsystem)

        );  
    }
    
}
